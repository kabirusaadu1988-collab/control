import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D


class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')

        # Earth radius [m]
        self.earth_radius = 6371000.0

        # GPS origin
        self.lat0 = None
        self.lon0 = None

        # Current local position
        self.x = 0.0
        self.y = 0.0

        # Current yaw
        self.yaw = 0.0

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            Pose2D,
            '/wamv/local_pose',
            10
        )

        self.get_logger().info('State estimator started.')

    def gps_callback(self, msg):

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # First GPS reading becomes local origin
        if self.lat0 is None:
            self.lat0 = lat
            self.lon0 = lon

            self.get_logger().info(
                f'GPS origin set: '
                f'lat={msg.latitude:.8f}, '
                f'lon={msg.longitude:.8f}'
            )

            return

        d_lat = lat - self.lat0
        d_lon = lon - self.lon0

        # Local East/North approximation
        self.x = (
            self.earth_radius
            * math.cos(self.lat0)
            * d_lon
        )

        self.y = (
            self.earth_radius
            * d_lat
        )

        self.publish_pose()

    def imu_callback(self, msg):

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Quaternion -> yaw
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_pose(self):

        pose = Pose2D()

        pose.x = self.x
        pose.y = self.y
        pose.theta = self.yaw

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    node = StateEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
