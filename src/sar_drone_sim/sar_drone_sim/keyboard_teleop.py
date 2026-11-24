"""Interactive terminal keyboard teleoperation node for the SAR drone."""

import json
import select
import sys
import termios
import time
import tty
from typing import Dict, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

SPEED_PRESETS: Dict[int, Tuple[str, float, float, float]] = {
    1: ('Lento', 1.2, 0.8, 0.7),
    2: ('Normal', 2.4, 1.4, 1.2),
    3: ('Rapido', 4.0, 2.2, 1.8),
}

BANNER = """
====================================================================
      SAR DRONE - CONTROL MANUAL POR TECLADO (/cmd_vel)
====================================================================
  Movimiento horizontal :  W / S  (Adelante / Atras)
                           A / D  (Izquierda / Derecha)
  Altitud (Subir/Bajar) :  R / F  o Flechas Arriba / Abajo
  Rotacion (Yaw)        :  Q / E  o Flechas Izquierda / Derecha
  Despegue automatico   :  T      (ascenso suave ~2 s)
  Hover inmediato       :  ESPACIO
  Velocidad (1/2/3,+/-) :  1=Lento | 2=Normal | 3=Rapido
  Salir                 :  X  o  Ctrl+C
====================================================================
"""


class KeyboardTeleop(Node):
    """Publishes Twist velocity commands and displays live drone telemetry."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/sar_drone/enable', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/sar_drone/odom', self._odom_cb, 10
        )
        self.det_sub = self.create_subscription(
            String, '/sar_drone/detections', self._det_cb, 10
        )

        self.preset_idx = 2
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.victims_in_view = 0

        self.current_twist = Twist()
        self.last_input_time = 0.0
        self.auto_takeoff_until = 0.0

        # Ensure controller is enabled on startup
        for _ in range(3):
            self.enable_pub.publish(Bool(data=True))

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos_x = p.x
        self.pos_y = p.y
        self.pos_z = p.z

    def _det_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.victims_in_view = int(data.get('count', 0))
        except Exception:
            pass

    def _read_key(self, timeout: float = 0.05) -> str:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return ''
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Read escape sequence for arrow keys if available
            r2, _, _ = select.select([sys.stdin], [], [], 0.01)
            if r2:
                ch += sys.stdin.read(1)
                r3, _, _ = select.select([sys.stdin], [], [], 0.01)
                if r3:
                    ch += sys.stdin.read(1)
        return ch

    def _publish_hover(self) -> None:
        self.current_twist = Twist()
        self.cmd_pub.publish(self.current_twist)

    def _print_status(self) -> None:
        mode_name, v_xy, v_z, v_yaw = SPEED_PRESETS[self.preset_idx]
        alert_txt = (
            f'ALERTA: {self.victims_in_view} persona(s)'
            if self.victims_in_view > 0
            else '0 personas'
        )
        line = (
            f'\r[Modo: {mode_name} xy={v_xy:.1f} z={v_z:.1f}] '
            f'Pos: X={self.pos_x:6.2f}m Y={self.pos_y:6.2f}m Alt={self.pos_z:5.2f}m | '
            f'Termica: {alert_txt:<22}'
        )
        sys.stdout.write(line)
        sys.stdout.flush()

    def run(self) -> None:
        print(BANNER)
        if not sys.stdin.isatty():
            self.get_logger().error('Se requiere una terminal interactiva (TTY) para keyboard_teleop.')
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
                        self.auto_takeoff_until = 0.0
                        self.current_twist = Twist()
                    elif key.lower() == 't':
                        self.auto_takeoff_until = now + 2.0
                        self.enable_pub.publish(Bool(data=True))
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
                            self.auto_takeoff_until = 0.0
                            self.current_twist = twist
                            self.last_input_time = now
                else:
                    if now < self.auto_takeoff_until:
                        twist = Twist()
                        twist.linear.z = 1.3
                        self.current_twist = twist
                    elif now - self.last_input_time > 0.35:
                        self.current_twist = Twist()

                self.cmd_pub.publish(self.current_twist)

                if now - last_status >= 0.15:
                    self._print_status()
                    last_status = now
        finally:
            self._publish_hover()
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
