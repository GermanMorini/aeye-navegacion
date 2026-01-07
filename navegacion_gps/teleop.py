import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


MSG = """
Teleop teclado (WASD)
---------------------------
W/X : adelante/atras
A/D : girar izquierda/derecha
Q/Z : subir/bajar velocidad lineal
E/C : subir/bajar velocidad angular
S o espacio : stop
CTRL+C para salir
"""

MOVE_BINDINGS = {
    'w': (1.0, 0.0),
    'x': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    's': (0.0, 0.0),
    ' ': (0.0, 0.0),
}

LINEAR_BINDINGS = {
    'w': 1.0,
    'x': -1.0,
}

ANGULAR_BINDINGS = {
    'a': 1.0,
    'd': -1.0,
}

STOP_BINDINGS = {
    's',
    ' ',
}

SPEED_BINDINGS = {
    'q': (1.1, 1.0),
    'z': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('ackermann_mode', False)

        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.turn = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.ackermann_mode = (
            self.get_parameter('ackermann_mode').get_parameter_value().bool_value
        )

        self.pub = self.create_publisher(Twist, topic, 10)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        x = 0.0
        th = 0.0

        try:
            print(MSG)
            if self.ackermann_mode:
                print("Modo Ackermann: W/X mantiene lineal, A/D cambia giro")
            print(self._speed_msg())
            while rclpy.ok():
                key = get_key(settings)
                if key in SPEED_BINDINGS:
                    lin_scale, ang_scale = SPEED_BINDINGS[key]
                    self.speed *= lin_scale
                    self.turn *= ang_scale
                    print(self._speed_msg())
                elif key == '\x03':
                    break
                elif self.ackermann_mode:
                    if key in STOP_BINDINGS:
                        x = 0.0
                        th = 0.0
                    elif key in LINEAR_BINDINGS:
                        x = LINEAR_BINDINGS[key]
                    elif key in ANGULAR_BINDINGS:
                        th = ANGULAR_BINDINGS[key]
                    else:
                        x = 0.0
                        th = 0.0
                else:
                    if key in MOVE_BINDINGS:
                        x, th = MOVE_BINDINGS[key]
                    else:
                        x = 0.0
                        th = 0.0

                twist = Twist()
                twist.linear.x = x * self.speed
                twist.angular.z = th * self.turn
                self.pub.publish(twist)
        finally:
            twist = Twist()
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def _speed_msg(self):
        return f"vel lineal: {self.speed:.2f} | vel angular: {self.turn:.2f}"


def main():
    rclpy.init()
    node = TeleopKeyboard()
    node.run()


if __name__ == '__main__':
    main()
