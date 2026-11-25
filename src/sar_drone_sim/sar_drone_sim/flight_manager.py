"""Flight manager node: mode arbitration, RTH failsafe, battery swap, orbit & gimbal."""

import json
import math
import time
from typing import Dict, List, Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from sar_drone_sim.usar_common import (
    COMMAND_POST_POS,
    HELIPAD_POS,
    normalize_angle_rad,
    quat_to_euler,
)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


class FlightManager(Node):
    """Arbitrates manual flight, takeoff, confirmation orbit, RTH, battery, and gimbal."""

    def __init__(self) -> None:
        super().__init__('flight_manager')

        self.declare_parameter('battery_drain_scale', 1.5)
        self.declare_parameter('rth_altitude_m', 7.5)
        self.declare_parameter('min_safe_alt_m', 2.2)
        self.declare_parameter('max_link_range_m', 45.0)
        self.declare_parameter('orbit_radius_m', 2.8)

        self.battery_drain_scale = float(self.get_parameter('battery_drain_scale').value)
        self.rth_altitude_m = float(self.get_parameter('rth_altitude_m').value)
        self.min_safe_alt_m = float(self.get_parameter('min_safe_alt_m').value)
        self.max_link_range_m = float(self.get_parameter('max_link_range_m').value)
        self.orbit_radius_m = float(self.get_parameter('orbit_radius_m').value)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/sar_drone/enable', 10)
        self.gimbal_pub = self.create_publisher(Float64, '/sar_drone/gimbal/pitch_cmd', 10)
        self.status_pub = self.create_publisher(String, '/sar_drone/status', 10)

        # Subscribers
        self.manual_sub = self.create_subscription(
            Twist, '/sar_drone/cmd_vel_manual', self._manual_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/sar_drone/odom', self._odom_cb, 10
        )
        self.findings_sub = self.create_subscription(
            String, '/sar_drone/findings', self._findings_cb, 10
        )
        self.mission_cmd_sub = self.create_subscription(
            String, '/sar_drone/mission_cmd', self._mission_cmd_cb, 10
        )

        # State
        self.mode = 'STANDBY'
        self.rth_stage = 'NONE'
        self.rth_reason = ''
        self.battery_pct = 100.0
        self.battery_num = 1
        self.swapping_until = 0.0
        self.mission_start = time.monotonic()

        self.pos = [0.0, 0.0, 0.14]
        self.rpy = [0.0, 0.0, 0.0]
        self.speed_mps = 0.0

        self.manual_twist = Twist()
        self.last_manual_time = 0.0
        self.takeoff_until = 0.0

        self.gimbal_pitch_rad = 0.48
        self.forced_link_loss = False
        self.link_lost_since: Optional[float] = None
        self.link_state = 'PRIMARY'
        self.rssi_pct = 100.0

        self.gust_until = 0.0
        self.gust_dir_world = (0.0, 0.0)

        self.tracks: List[Dict] = []
        self.orbit_target_id: Optional[str] = None
        self.orbit_target_xy = (0.0, 0.0)
        self.orbit_alt_m = 3.5
        self.orbit_prev_angle: Optional[float] = None
        self.orbit_swept_rad = 0.0

        self._last_tick = time.monotonic()
        self.ctrl_timer = self.create_timer(0.05, self._control_loop)
        self.status_timer = self.create_timer(0.10, self._publish_status)

        for _ in range(3):
            self.enable_pub.publish(Bool(data=True))
            self.gimbal_pub.publish(Float64(data=self.gimbal_pitch_rad))

        self.get_logger().info(
            f'FlightManager activo [RTH={self.rth_altitude_m}m, '
            f'min_alt={self.min_safe_alt_m}m, max_link={self.max_link_range_m}m]'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = [float(p.x), float(p.y), float(p.z)]
        self.rpy = list(quat_to_euler(q.x, q.y, q.z, q.w))
        v = msg.twist.twist.linear
        self.speed_mps = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _findings_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.tracks = list(data.get('tracks', []))
        except Exception:
            pass

    def _manual_cb(self, msg: Twist) -> None:
        self.manual_twist = msg
        self.last_manual_time = time.monotonic()
        mag = (
            abs(msg.linear.x)
            + abs(msg.linear.y)
            + abs(msg.linear.z)
            + abs(msg.angular.z)
        )
        if mag > 0.05 and self.mode in ('STANDBY', 'TAKEOFF', 'ORBIT_CONFIRM'):
            self.mode = 'MANUAL'
            self.orbit_target_id = None

    def _mission_cmd_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        cmd = str(payload.get('cmd', '')).strip()
        now = time.monotonic()

        if cmd == 'takeoff':
            if self.mode == 'BATTERY_SWAP':
                return
            self.enable_pub.publish(Bool(data=True))
            self.takeoff_until = now + 2.2
            self.mode = 'TAKEOFF'
            self.rth_stage = 'NONE'
            self.rth_reason = ''
            self.orbit_target_id = None
            self.get_logger().info('Despegue automatico iniciado.')

        elif cmd == 'hover':
            self.manual_twist = Twist()
            if self.mode not in ('BATTERY_SWAP',):
                self.mode = 'MANUAL'
            self.orbit_target_id = None
            self.rth_stage = 'NONE'
            self.rth_reason = ''

        elif cmd == 'rth_toggle':
            if self.mode == 'RTH':
                self.mode = 'MANUAL'
                self.rth_stage = 'NONE'
                self.rth_reason = ''
                self.get_logger().info('RTH cancelado por operador.')
            else:
                self._start_rth('OPERADOR')

        elif cmd == 'orbit_toggle':
            if self.mode == 'ORBIT_CONFIRM':
                self.mode = 'MANUAL'
                self.orbit_target_id = None
                self.get_logger().info('Orbita de confirmacion cancelada.')
            else:
                self._start_orbit()

        elif cmd == 'link_toggle':
            self.forced_link_loss = not self.forced_link_loss
            self.get_logger().info(
                f'Falla de enlace simulada: {"ACTIVA" if self.forced_link_loss else "RESTAURADA"}'
            )

        elif cmd == 'battery_swap':
            dist_home = math.hypot(self.pos[0] - HELIPAD_POS[0], self.pos[1] - HELIPAD_POS[1])
            if dist_home < 2.2 and self.pos[2] < 0.45:
                self.mode = 'BATTERY_SWAP'
                self.swapping_until = now + 3.0
                self.rth_stage = 'NONE'
                self.rth_reason = ''
                self.get_logger().info('Iniciando relevo de bateria en helipuerto...')
            else:
                self.get_logger().warning(
                    'Cambio de bateria rechazado: aterrice en el helipuerto (<2.2m, alt<0.45m).'
                )

        elif cmd == 'gust_trigger':
            self.gust_until = now + 4.0
            angle = self.rpy[2] + math.pi / 2.0
            self.gust_dir_world = (math.cos(angle), math.sin(angle))
            self.get_logger().info('Rafaga lateral de viento aplicada (4 s).')

        elif cmd == 'gimbal_delta':
            delta = float(payload.get('delta', 0.0))
            self.gimbal_pitch_rad = _clamp(self.gimbal_pitch_rad + delta, 0.0, 1.40)
            self.gimbal_pub.publish(Float64(data=self.gimbal_pitch_rad))

        elif cmd == 'gimbal_set':
            val = float(payload.get('pitch_rad', 0.48))
            self.gimbal_pitch_rad = _clamp(val, 0.0, 1.40)
            self.gimbal_pub.publish(Float64(data=self.gimbal_pitch_rad))

    def _start_rth(self, reason: str) -> None:
        if self.mode == 'BATTERY_SWAP':
            return
        self.orbit_target_id = None
        self.mode = 'RTH'
        self.rth_reason = reason
        dist_home = math.hypot(self.pos[0] - HELIPAD_POS[0], self.pos[1] - HELIPAD_POS[1])
        if dist_home < 0.55 and self.pos[2] < 0.5:
            self.rth_stage = 'DESCEND'
        elif self.pos[2] < self.rth_altitude_m - 0.6 and dist_home >= 0.8:
            self.rth_stage = 'CLIMB'
        else:
            self.rth_stage = 'RETURN'
        self.get_logger().warning(f'RTH activado ({reason}) [etapa={self.rth_stage}]')

    def _start_orbit(self) -> None:
        if not self.tracks:
            self.get_logger().info('Orbita omitida: no hay victimas sospechosas en mapa.')
            return
        candidates = [t for t in self.tracks if t.get('status') != 'CONFIRMED']
        pool = candidates if candidates else self.tracks
        best = min(
            pool,
            key=lambda t: math.hypot(t['x'] - self.pos[0], t['y'] - self.pos[1]),
        )
        self.orbit_target_id = str(best['id'])
        self.orbit_target_xy = (float(best['x']), float(best['y']))
        self.orbit_alt_m = max(self.pos[2], 3.2)
        self.orbit_prev_angle = math.atan2(
            self.pos[1] - self.orbit_target_xy[1],
            self.pos[0] - self.orbit_target_xy[0],
        )
        self.orbit_swept_rad = 0.0
        self.mode = 'ORBIT_CONFIRM'
        self.get_logger().info(
            f'Orbita de doble confirmacion hacia {self.orbit_target_id} '
            f'en ({self.orbit_target_xy[0]:.1f}, {self.orbit_target_xy[1]:.1f})'
        )

    def _world_to_body_xy(self, vx_w: float, vy_w: float) -> tuple[float, float]:
        yaw = self.rpy[2]
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return cy * vx_w + sy * vy_w, -sy * vx_w + cy * vy_w

    def _update_link_and_battery(self, now: float, dt: float) -> None:
        # Link quality relative to Command Post (-4.8, -4.5)
        dist_cp = math.hypot(self.pos[0] - COMMAND_POST_POS[0], self.pos[1] - COMMAND_POST_POS[1])
        if self.forced_link_loss:
            self.rssi_pct = 0.0
            self.link_state = 'LOST'
        else:
            ratio = _clamp(1.0 - (dist_cp / max(self.max_link_range_m, 1.0)), 0.0, 1.0)
            self.rssi_pct = round(100.0 * (ratio ** 0.75), 1)
            if self.rssi_pct > 40.0:
                self.link_state = 'PRIMARY'
            elif self.rssi_pct >= 15.0:
                self.link_state = 'BACKUP'
            else:
                self.link_state = 'LOST'

        if self.link_state == 'LOST':
            if self.link_lost_since is None:
                self.link_lost_since = now
            elif (
                now - self.link_lost_since >= 2.0
                and self.mode not in ('RTH', 'STANDBY', 'BATTERY_SWAP')
                and self.pos[2] > 0.35
            ):
                self._start_rth('ENLACE PERDIDO')
        else:
            self.link_lost_since = None

        # Battery swap / drain
        if self.mode == 'BATTERY_SWAP':
            if now >= self.swapping_until:
                self.battery_num += 1
                self.battery_pct = 100.0
                self.mode = 'STANDBY'
                self.get_logger().info(f'Bateria #{self.battery_num} instalada (100%).')
            return

        if self.mode != 'STANDBY' and self.pos[2] > 0.28:
            drain_rate = (0.085 + 0.015 * min(self.speed_mps, 5.0)) * self.battery_drain_scale
            self.battery_pct = max(0.0, self.battery_pct - drain_rate * dt)
            if self.battery_pct < 15.0 and self.mode != 'RTH':
                self._start_rth('BATERIA CRITICA')

    def _compute_rth_twist(self) -> Twist:
        twist = Twist()
        dx = HELIPAD_POS[0] - self.pos[0]
        dy = HELIPAD_POS[1] - self.pos[1]
        dist_xy = math.hypot(dx, dy)

        if self.rth_stage == 'CLIMB':
            dz = self.rth_altitude_m - self.pos[2]
            twist.linear.z = _clamp(1.2 * dz, -1.0, 1.8)
            if self.pos[2] >= self.rth_altitude_m - 0.35:
                self.rth_stage = 'RETURN'

        elif self.rth_stage == 'RETURN':
            if dist_xy < 0.45:
                self.rth_stage = 'DESCEND'
            else:
                speed = min(2.4, max(0.6, 0.9 * dist_xy))
                vx_w = speed * (dx / dist_xy)
                vy_w = speed * (dy / dist_xy)
                vx_b, vy_b = self._world_to_body_xy(vx_w, vy_w)
                twist.linear.x = vx_b
                twist.linear.y = vy_b
                twist.linear.z = _clamp(1.0 * (self.rth_altitude_m - self.pos[2]), -0.8, 1.2)
                if dist_xy > 1.5:
                    target_yaw = math.atan2(dy, dx)
                    yaw_err = normalize_angle_rad(target_yaw - self.rpy[2])
                    twist.angular.z = _clamp(1.2 * yaw_err, -1.2, 1.2)

        if self.rth_stage == 'DESCEND':
            vx_w = _clamp(0.8 * dx, -0.6, 0.6)
            vy_w = _clamp(0.8 * dy, -0.6, 0.6)
            vx_b, vy_b = self._world_to_body_xy(vx_w, vy_w)
            twist.linear.x = vx_b
            twist.linear.y = vy_b
            twist.linear.z = -0.55
            if self.pos[2] < 0.24:
                self.mode = 'STANDBY'
                self.rth_stage = 'NONE'
                self.rth_reason = ''
                self.get_logger().info('RTH completado: dron aterrizado en helipuerto.')
                return Twist()

        return twist

    def _compute_orbit_twist(self) -> Twist:
        # Refresh target position if track updated, and stop if confirmed
        for tr in self.tracks:
            if str(tr.get('id')) == self.orbit_target_id:
                self.orbit_target_xy = (float(tr['x']), float(tr['y']))
                if tr.get('status') == 'CONFIRMED':
                    self.mode = 'MANUAL'
                    self.orbit_target_id = None
                    self.get_logger().info('Objetivo confirmado: saliendo de orbita.')
                    return Twist()
                break

        tx, ty = self.orbit_target_xy
        rx = self.pos[0] - tx
        ry = self.pos[1] - ty
        dist = max(math.hypot(rx, ry), 0.1)

        angle = math.atan2(ry, rx)
        if self.orbit_prev_angle is not None:
            self.orbit_swept_rad += abs(normalize_angle_rad(angle - self.orbit_prev_angle))
        self.orbit_prev_angle = angle

        if self.orbit_swept_rad >= math.radians(345.0):
            self.mode = 'MANUAL'
            self.orbit_target_id = None
            self.get_logger().info('Orbita 360 completada.')
            return Twist()

        # Radial + CCW tangential velocity in world frame
        ux, uy = rx / dist, ry / dist
        tx_tan, ty_tan = -uy, ux
        v_radial = _clamp(0.9 * (self.orbit_radius_m - dist), -1.2, 1.2)
        v_tan = 1.15
        vx_w = v_radial * ux + v_tan * tx_tan
        vy_w = v_radial * uy + v_tan * ty_tan
        vx_b, vy_b = self._world_to_body_xy(vx_w, vy_w)

        twist = Twist()
        twist.linear.x = vx_b
        twist.linear.y = vy_b
        twist.linear.z = _clamp(1.0 * (self.orbit_alt_m - self.pos[2]), -0.8, 1.0)

        # Keep drone nose and gimbal aimed at target
        desired_yaw = math.atan2(ty - self.pos[1], tx - self.pos[0])
        yaw_err = normalize_angle_rad(desired_yaw - self.rpy[2])
        twist.angular.z = _clamp(1.8 * yaw_err, -1.5, 1.5)

        desired_pitch = _clamp(math.atan2(max(self.pos[2] - 0.3, 0.2), dist), 0.25, 1.35)
        if abs(desired_pitch - self.gimbal_pitch_rad) > 0.03:
            self.gimbal_pitch_rad = desired_pitch
            self.gimbal_pub.publish(Float64(data=self.gimbal_pitch_rad))

        return twist

    def _control_loop(self) -> None:
        now = time.monotonic()
        dt = max(0.001, min(0.2, now - self._last_tick))
        self._last_tick = now

        self._update_link_and_battery(now, dt)

        out_twist = Twist()
        if self.mode in ('STANDBY', 'BATTERY_SWAP'):
            out_twist = Twist()
        elif self.mode == 'TAKEOFF':
            self.enable_pub.publish(Bool(data=True))
            if now < self.takeoff_until and self.pos[2] < 2.8:
                out_twist.linear.z = 1.3
            else:
                self.mode = 'MANUAL'
                out_twist = Twist()
        elif self.mode == 'MANUAL':
            if now - self.last_manual_time <= 0.40:
                out_twist = Twist()
                out_twist.linear.x = self.manual_twist.linear.x
                out_twist.linear.y = self.manual_twist.linear.y
                out_twist.linear.z = self.manual_twist.linear.z
                out_twist.angular.z = self.manual_twist.angular.z
        elif self.mode == 'ORBIT_CONFIRM':
            out_twist = self._compute_orbit_twist()
        elif self.mode == 'RTH':
            out_twist = self._compute_rth_twist()

        # Apply wind gust perturbation if active (C05)
        if now < self.gust_until and self.mode not in ('STANDBY', 'BATTERY_SWAP'):
            rem = (self.gust_until - now) / 4.0
            mag = 0.9 * math.sin(math.pi * rem)
            gx_b, gy_b = self._world_to_body_xy(
                self.gust_dir_world[0] * mag, self.gust_dir_world[1] * mag
            )
            out_twist.linear.x += gx_b
            out_twist.linear.y += gy_b

        self.cmd_pub.publish(out_twist)

    def _publish_status(self) -> None:
        now = time.monotonic()
        dist_home = math.hypot(self.pos[0] - HELIPAD_POS[0], self.pos[1] - HELIPAD_POS[1])
        low_alt = self.mode in ('MANUAL', 'ORBIT_CONFIRM') and 0.45 < self.pos[2] < self.min_safe_alt_m
        payload = {
            'mode': self.mode,
            'rth_stage': self.rth_stage,
            'rth_reason': self.rth_reason,
            'battery_pct': round(self.battery_pct, 1),
            'battery_num': self.battery_num,
            'mission_elapsed_s': round(now - self.mission_start, 1),
            'swapping_battery': self.mode == 'BATTERY_SWAP',
            'link_state': self.link_state,
            'rssi_pct': self.rssi_pct,
            'forced_link_loss': self.forced_link_loss,
            'distance_home_m': round(dist_home, 2),
            'altitude_m': round(self.pos[2], 2),
            'speed_mps': round(self.speed_mps, 2),
            'gimbal_pitch_rad': round(self.gimbal_pitch_rad, 3),
            'min_safe_alt_m': self.min_safe_alt_m,
            'low_alt_warning': low_alt,
            'gust_active': now < self.gust_until,
            'orbit_target_id': self.orbit_target_id,
            'pos': [round(v, 3) for v in self.pos],
            'rpy': [round(v, 4) for v in self.rpy],
        }
        self.status_pub.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
