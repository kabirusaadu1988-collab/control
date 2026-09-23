import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WamvController(Node):

    def __init__(self):
        super().__init__('wamv_controller')

        self.declare_parameter('goal_x', 20.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('k_distance', 2.0)
        self.declare_parameter('k_heading', 4.0)
        self.declare_parameter('max_thrust', 10.0)
        self.declare_parameter('goal_tolerance', 1.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_distance = self.get_parameter('k_distance').value
        self.k_heading = self.get_parameter('k_heading').value
        self.max_thrust = self.get_parameter('max_thrust').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.x = None
        self.y = None
        self.yaw = None

        self.pose_sub = self.create_subscription(
            Pose2D,
            '/wamv/local_pose',
            self.pose_callback,
            10
        )

        self.left_thrust_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/left/thrust',
            10
        )

        self.right_thrust_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/right/thrust',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Controller started. Goal: ({self.goal_x}, {self.goal_y})'
        )

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta

    def publish_thrust(self, left, right):
        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = float(left)
        right_msg.data = float(right)

        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)

    def control_loop(self):
        if self.x is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance_error = math.sqrt(dx * dx + dy * dy)

        if distance_error < self.goal_tolerance:
            self.publish_thrust(0.0, 0.0)
            return

        desired_heading = math.atan2(dy, dx)

        heading_error = wrap_to_pi(desired_heading - self.yaw)

        forward_command = self.k_distance * distance_error
        turn_command = self.k_heading * heading_error

        left_thrust = forward_command - turn_command
        right_thrust = forward_command + turn_command

        left_thrust = clamp(
            left_thrust,
            -self.max_thrust,
            self.max_thrust
        )

        right_thrust = clamp(
            right_thrust,
            -self.max_thrust,
            self.max_thrust
        )

        self.publish_thrust(left_thrust, right_thrust)


def main(args=None):
    rclpy.init(args=args)

    node = WamvController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_thrust(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
