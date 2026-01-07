import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/ackermann_cmd')
        self.declare_parameter('wheelbase', 0.60)
        self.declare_parameter('steering_limit', 0.5235987756)
        self.declare_parameter('min_speed_for_steering', 0.05)
        self.declare_parameter('frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.steering_limit = self.get_parameter('steering_limit').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed_for_steering').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(AckermannDriveStamped, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self._twist_cb, 10)

    def _twist_cb(self, msg: Twist):
        speed = msg.linear.x
        steering_angle = 0.0

        if abs(speed) >= self.min_speed:
            steering_angle = math.atan(self.wheelbase * msg.angular.z / speed)
            steering_angle = max(-self.steering_limit, min(self.steering_limit, steering_angle))

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.drive.speed = speed
        out.drive.steering_angle = steering_angle
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToAckermann()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
