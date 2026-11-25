"""Interactive terminal keyboard teleoperation & USAR mission console."""

import json
import math
import select
import sys
import termios
import time
import tty
from typing import Dict, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SPEED_PRESETS: Dict[int, Tuple[str, float, float, float]] = {
    1: ('Lento', 1.2, 0.8, 0.7),
    2: ('Normal', 2.4, 1.4, 1.2),
    3: ('Rapido', 4.0, 2.2, 1.8),
}

BANNER = """
================================================================================
        DRON USAR - TELEOPERACION Y CONSOLA DE MISION (C07 / C11 / C18)
================================================================================
  Vuelo horizontal        :  W / S (Adelante / Atras) | A / D (Izq / Der)
  Altitud y rotacion      :  R / F o Flechas Arr/Aba  | Q / E o Flechas Izq/Der
  Despegue / Hover        :  T (despegue 2.2s)       | ESPACIO (hover)
  Velocidad               :  1=Lento | 2=Normal | 3=Rapido (+ / -)
  Gimbal biespectral      :  U (subir vista) | J (bajar vista)
  Doble confirmacion      :  O (orbita 2do angulo)   | C (confirmar y guardar)
  Seguridad / Contingencia:  H (RTH casa) | L (falla enlace) | B (relevo bat)
  Pruebas / Visualizacion :  G (rafaga viento)       | P (paleta termica)
  Salir                   :  X o Ctrl+C
================================================================================
"""


class KeyboardTeleop(Node):
    """Publishes manual Twist commands and discrete USAR mission commands."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.manual_pub = self.create_publisher(Twist, '/sar_drone/cmd_vel_manual', 10)
        self.mission_pub = self.create_publisher(String, '/sar_drone/mission_cmd', 10)

        self.status_sub = self.create_subscription(
            String, '/sar_drone/status', self._status_cb, 10
        )
        self.findings_sub = self.create_subscription(
            String, '/sar_drone/findings', self._findings_cb, 10
        )

        self.preset_idx = 2
        self.status: Dict = {
            'mode': 'STANDBY',
            'altitude_m': 0.14,
            'battery_pct': 100.0,
            'battery_num': 1,
            'link_state': 'PRIMARY',
            'gimbal_pitch_rad': 0.48,
            'pos': [0.0, 0.0, 0.14],
        }
        self.counts: Dict = {'suspect': 0, 'ready': 0, 'confirmed': 0}

        self.current_twist = Twist()
        self.last_input_time = 0.0

    def _status_cb(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def _findings_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.counts = dict(data.get('counts', self.counts))
        except Exception:
            pass

    def _send_mission_cmd(self, cmd: str, **extra) -> None:
        payload = {'cmd': cmd, **extra}
        self.mission_pub.publish(String(data=json.dumps(payload)))

    def _read_key(self, timeout: float = 0.05) -> str:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return ''
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            r2, _, _ = select.select([sys.stdin], [], [], 0.01)
            if r2:
                ch += sys.stdin.read(1)
                r3, _, _ = select.select([sys.stdin], [], [], 0.01)
                if r3:
                    ch += sys.stdin.read(1)
        return ch

    def _print_status(self) -> None:
        mode_name, v_xy, _, _ = SPEED_PRESETS[self.preset_idx]
        pos = self.status.get('pos', [0.0, 0.0, 0.0])
        g_deg = math.degrees(float(self.status.get('gimbal_pitch_rad', 0.48)))
        line = (
            f"\r[{self.status.get('mode', '?'):<13} | {mode_name} {v_xy:.1f}] "
            f"XYZ=({pos[0]:5.1f},{pos[1]:5.1f},{pos[2]:4.1f})m "
            f"Bat#{self.status.get('battery_num', 1)}:{self.status.get('battery_pct', 100.0):4.0f}% "
            f"Link:{self.status.get('link_state', 'PRIMARY'):<7} "
            f"Gmb:{g_deg:2.0f}d | "
            f"V[S:{self.counts.get('suspect', 0)} L:{self.counts.get('ready', 0)} C:{self.counts.get('confirmed', 0)}]   "
        )
        sys.stdout.write(line)
        sys.stdout.flush()

    def run(self) -> None:
        print(BANNER)
        if not sys.stdin.isatty():
            self.get_logger().error(
                'Se requiere una terminal interactiva (TTY) para keyboard_teleop.'
            )
            return

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            last_status = 0.0
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                key = self._read_key(timeout=0.05)
                now = time.monotonic()

                _, v_xy, v_z, v_yaw = SPEED_PRESETS[self.preset_idx]

                if key:
                    if key == '\x03' or key.lower() == 'x':
                        break
                    elif key == ' ':
                        self.current_twist = Twist()
                        self._send_mission_cmd('hover')
                    elif key.lower() == 't':
                        self._send_mission_cmd('takeoff')
                    elif key.lower() == 'u':
                        self._send_mission_cmd('gimbal_delta', delta=-0.12)
                    elif key.lower() == 'j':
                        self._send_mission_cmd('gimbal_delta', delta=0.12)
                    elif key.lower() == 'o':
                        self._send_mission_cmd('orbit_toggle')
                    elif key.lower() == 'c':
                        self._send_mission_cmd('confirm_finding')
                    elif key.lower() == 'h':
                        self._send_mission_cmd('rth_toggle')
                    elif key.lower() == 'l':
                        self._send_mission_cmd('link_toggle')
                    elif key.lower() == 'b':
                        self._send_mission_cmd('battery_swap')
                    elif key.lower() == 'g':
                        self._send_mission_cmd('gust_trigger')
                    elif key.lower() == 'p':
                        self._send_mission_cmd('palette_next')
                    elif key == '1':
                        self.preset_idx = 1
                    elif key == '2':
                        self.preset_idx = 2
                    elif key == '3':
                        self.preset_idx = 3
                    elif key in ('+', '='):
                        self.preset_idx = min(3, self.preset_idx + 1)
                    elif key in ('-', '_'):
                        self.preset_idx = max(1, self.preset_idx - 1)
                    else:
                        twist = Twist()
                        matched = True
                        kl = key.lower()
                        if kl == 'w':
                            twist.linear.x = v_xy
                        elif kl == 's':
                            twist.linear.x = -v_xy
                        elif kl == 'a':
                            twist.linear.y = v_xy
                        elif kl == 'd':
                            twist.linear.y = -v_xy
                        elif kl == 'r' or key == '\x1b[A':
                            twist.linear.z = v_z
                        elif kl == 'f' or key == '\x1b[B':
                            twist.linear.z = -v_z
                        elif kl == 'q' or key == '\x1b[D':
                            twist.angular.z = v_yaw
                        elif kl == 'e' or key == '\x1b[C':
                            twist.angular.z = -v_yaw
                        else:
                            matched = False

                        if matched:
                            self.current_twist = twist
                            self.last_input_time = now
                else:
                    if now - self.last_input_time > 0.35:
                        self.current_twist = Twist()

                self.manual_pub.publish(self.current_twist)

                if now - last_status >= 0.15:
                    self._print_status()
                    last_status = now
        finally:
            self.manual_pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print('\nTeleop finalizado. Dron en hover.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
