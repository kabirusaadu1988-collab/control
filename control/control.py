import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from std_msgs.msg import Float64


class WaypointController(Node):

    def __init__(self, manual_target=None):
        super().__init__('waypoint_controller')

        # =====================================================
        # Controller parameters
        # =====================================================

        self.declare_parameter('kp_distance', 8.0)
        self.declare_parameter('kp_heading', 40.0)

        self.declare_parameter('max_thrust', 100.0)

        self.declare_parameter(
            'waypoint_tolerance',
            1.0
        )

        # Safety:
        # False = controller calculates everything,
        # but actual thrusters receive zero.
        self.declare_parameter(
            'enable_motion',
            False
        )

        self.kp_distance = float(
            self.get_parameter(
                'kp_distance'
            ).value
        )

        self.kp_heading = float(
            self.get_parameter(
                'kp_heading'
            ).value
        )

        self.max_thrust = float(
            self.get_parameter(
                'max_thrust'
            ).value
        )

        self.waypoint_tolerance = float(
            self.get_parameter(
                'waypoint_tolerance'
            ).value
        )

        self.enable_motion = bool(
            self.get_parameter(
                'enable_motion'
            ).value
        )

        # =====================================================
        # Waypoint storage
        # =====================================================

        self.waypoints = []

        self.current_waypoint_index = 0

        self.path_source = 'none'

        self.path_complete = False

        # Manual waypoint supplied at startup
        if manual_target is not None:

            self.waypoints = [
                manual_target
            ]

            self.path_source = 'manual'

        # =====================================================
        # Current robot state
        # =====================================================

        self.current_x = None
        self.current_y = None
        self.current_psi = None

        # =====================================================
        # ROS subscriptions
        # =====================================================

        # State estimator output
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/wamv/local_pose',
            self.pose_callback,
            10
        )

        # Planner trajectory
        self.path_sub = self.create_subscription(
            Path,
            '/planner/path',
            self.path_callback,
            10
        )

        # =====================================================
        # Thruster publishers
        # =====================================================

        self.left_thruster_pub = (
            self.create_publisher(
                Float64,
                '/wamv/thrusters/left/thrust',
                10
            )
        )

        self.right_thruster_pub = (
            self.create_publisher(
                Float64,
                '/wamv/thrusters/right/thrust',
                10
            )
        )

        # Logging limiter
        self.last_log_time = 0.0

        # =====================================================
        # Startup messages
        # =====================================================

        self.get_logger().info(
            'Waypoint controller started.'
        )

        if manual_target is not None:

            self.get_logger().info(
                f'Manual target loaded: '
                f'X={manual_target[0]:.2f} m, '
                f'Y={manual_target[1]:.2f} m'
            )

        else:

            self.get_logger().info(
                'No manual target supplied.'
            )

            self.get_logger().info(
                'Waiting for planner trajectory '
                'on /planner/path...'
            )

        if self.enable_motion:

            self.get_logger().warn(
                'MOTION ENABLED.'
            )

        else:

            self.get_logger().info(
                'DRY RUN: thrusters disabled.'
            )

    # =========================================================
    # Planner path callback
    # =========================================================

    def path_callback(self, msg):

        if len(msg.poses) == 0:

            self.get_logger().warn(
                'Received empty planner path.'
            )

            return

        new_waypoints = []

        for pose_stamped in msg.poses:

            x = (
                pose_stamped
                .pose
                .position
                .x
            )

            y = (
                pose_stamped
                .pose
                .position
                .y
            )

            new_waypoints.append(
                (float(x), float(y))
            )

        # Replace existing path
        self.waypoints = new_waypoints

        self.current_waypoint_index = 0

        self.path_source = 'planner'

        self.path_complete = False

        self.get_logger().info(
            f'Planner path received: '
            f'{len(self.waypoints)} waypoints.'
        )

        first = self.waypoints[0]

        final = self.waypoints[-1]

        self.get_logger().info(
            f'First waypoint: '
            f'({first[0]:.2f}, '
            f'{first[1]:.2f})'
        )

        self.get_logger().info(
            f'Final waypoint: '
            f'({final[0]:.2f}, '
            f'{final[1]:.2f})'
        )

    # =========================================================
    # State callback
    # =========================================================

    def pose_callback(self, msg):

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_psi = msg.theta

        # No target yet
        if len(self.waypoints) == 0:

            self.publish_thrusters(
                0.0,
                0.0
            )

            return

        if self.path_complete:

            self.publish_thrusters(
                0.0,
                0.0
            )

            return

        self.run_controller()

    # =========================================================
    # Main control algorithm
    # =========================================================

    def run_controller(self):

        x = self.current_x
        y = self.current_y
        psi = self.current_psi

        target_x, target_y = (
            self.waypoints[
                self.current_waypoint_index
            ]
        )

        # -----------------------------------------------------
        # 1. World-frame position error
        # -----------------------------------------------------

        error_x_world = (
            target_x - x
        )

        error_y_world = (
            target_y - y
        )

        # -----------------------------------------------------
        # 2. Distance to current waypoint
        # -----------------------------------------------------

        distance_error = math.hypot(
            error_x_world,
            error_y_world
        )

        # -----------------------------------------------------
        # 3. Check if waypoint reached
        # -----------------------------------------------------

        if (
            distance_error
            <= self.waypoint_tolerance
        ):

            self.advance_waypoint(
                distance_error
            )

            return

        # -----------------------------------------------------
        # 4. Desired heading
        # -----------------------------------------------------

        desired_heading = math.atan2(
            error_y_world,
            error_x_world
        )

        # -----------------------------------------------------
        # 5. Heading error
        # -----------------------------------------------------

        heading_error = (
            self.wrap_angle(
                desired_heading - psi
            )
        )

        # -----------------------------------------------------
        # 6. World -> body-frame error
        #
        # e_body = R^T(psi) e_world
        # -----------------------------------------------------

        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)

        error_forward = (
            cos_psi
            * error_x_world
            +
            sin_psi
            * error_y_world
        )

        error_lateral = (
            -sin_psi
            * error_x_world
            +
            cos_psi
            * error_y_world
        )

        # -----------------------------------------------------
        # 7. P controller
        # -----------------------------------------------------

        forward_command = (
            self.kp_distance
            * distance_error
        )

        turning_command = (
            self.kp_heading
            * heading_error
        )

        # -----------------------------------------------------
        # 8. Reduce forward thrust if badly misaligned
        # -----------------------------------------------------

        heading_alignment = max(
            0.0,
            math.cos(
                heading_error
            )
        )

        forward_command *= (
            heading_alignment
        )

        # -----------------------------------------------------
        # 9. Differential thrust
        # -----------------------------------------------------

        left_thrust = (
            forward_command
            - turning_command
        )

        right_thrust = (
            forward_command
            + turning_command
        )

        # -----------------------------------------------------
        # 10. Thruster saturation
        # -----------------------------------------------------

        left_thrust = self.clamp(
            left_thrust,
            -self.max_thrust,
            self.max_thrust
        )

        right_thrust = self.clamp(
            right_thrust,
            -self.max_thrust,
            self.max_thrust
        )

        # -----------------------------------------------------
        # 11. Logging
        # -----------------------------------------------------

        self.log_state(
            target_x,
            target_y,
            error_x_world,
            error_y_world,
            error_forward,
            error_lateral,
            distance_error,
            desired_heading,
            heading_error,
            left_thrust,
            right_thrust
        )

        # -----------------------------------------------------
        # 12. Publish thruster commands
        # -----------------------------------------------------

        if self.enable_motion:

            self.publish_thrusters(
                left_thrust,
                right_thrust
            )

        else:

            self.publish_thrusters(
                0.0,
                0.0
            )

    # =========================================================
    # Waypoint progression
    # =========================================================

    def advance_waypoint(
        self,
        distance_error
    ):

        target = self.waypoints[
            self.current_waypoint_index
        ]

        self.get_logger().info(
            f'Waypoint '
            f'{self.current_waypoint_index + 1}'
            f'/{len(self.waypoints)} '
            f'reached: '
            f'({target[0]:.2f}, '
            f'{target[1]:.2f}) '
            f'error={distance_error:.2f} m'
        )

        # Is another waypoint available?
        if (
            self.current_waypoint_index
            < len(self.waypoints) - 1
        ):

            self.current_waypoint_index += 1

            next_target = self.waypoints[
                self.current_waypoint_index
            ]

            self.get_logger().info(
                f'Next waypoint: '
                f'({next_target[0]:.2f}, '
                f'{next_target[1]:.2f})'
            )

        else:

            self.path_complete = True

            self.publish_thrusters(
                0.0,
                0.0
            )

            self.get_logger().info(
                'FINAL TARGET REACHED. '
                'Trajectory complete.'
            )

    # =========================================================
    # Angle helper
    # =========================================================

    @staticmethod
    def wrap_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # =========================================================
    # Clamp helper
    # =========================================================

    @staticmethod
    def clamp(
        value,
        minimum,
        maximum
    ):

        return max(
            minimum,
            min(value, maximum)
        )

    # =========================================================
    # Thruster publisher
    # =========================================================

    def publish_thrusters(
        self,
        left,
        right
    ):

        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = float(left)
        right_msg.data = float(right)

        self.left_thruster_pub.publish(
            left_msg
        )

        self.right_thruster_pub.publish(
            right_msg
        )

    # =========================================================
    # Logging
    # =========================================================

    def log_state(
        self,
        target_x,
        target_y,
        error_x_world,
        error_y_world,
        error_forward,
        error_lateral,
        distance_error,
        desired_heading,
        heading_error,
        left_thrust,
        right_thrust
    ):

        current_time = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        # Log twice per second
        if (
            current_time
            - self.last_log_time
            < 0.5
        ):

            return

        self.last_log_time = (
            current_time
        )

        self.get_logger().info(
            '\n'
            f'Path source     : '
            f'{self.path_source}\n'
            f'Waypoint       : '
            f'{self.current_waypoint_index + 1}'
            f'/{len(self.waypoints)}\n'
            f'Position       : '
            f'X={self.current_x:7.2f} '
            f'Y={self.current_y:7.2f}\n'
            f'Heading        : '
            f'{self.current_psi:7.3f} rad '
            f'({math.degrees(self.current_psi):.2f} deg)\n'
            f'Target         : '
            f'X={target_x:7.2f} '
            f'Y={target_y:7.2f}\n'
            f'World error    : '
            f'ex={error_x_world:7.2f} '
            f'ey={error_y_world:7.2f}\n'
            f'Body error     : '
            f'forward={error_forward:7.2f} '
            f'lateral={error_lateral:7.2f}\n'
            f'Distance       : '
            f'{distance_error:7.2f} m\n'
            f'Desired heading: '
            f'{desired_heading:7.3f} rad '
            f'({math.degrees(desired_heading):.2f} deg)\n'
            f'Heading error  : '
            f'{heading_error:7.3f} rad '
            f'({math.degrees(heading_error):.2f} deg)\n'
            f'Thrust         : '
            f'L={left_thrust:7.2f} '
            f'R={right_thrust:7.2f}\n'
            f'Motion enabled : '
            f'{self.enable_motion}'
        )

    # =========================================================
    # Safe shutdown
    # =========================================================

    def stop(self):

        self.publish_thrusters(
            0.0,
            0.0
        )


# =============================================================
# Startup target input
# =============================================================

def ask_for_manual_target():

    print()
    print(
        '=== WAM-V CONTROLLER ==='
    )

    print(
        'Coordinates use the LOCAL map frame.'
    )

    print(
        'Robot start = (0, 0) metres.'
    )

    print()

    print(
        'Enter a test target as: X Y'
    )

    print(
        'Example: 10 0'
    )

    print()

    print(
        'Press ENTER without typing '
        'anything to wait for /planner/path.'
    )

    print()

    while True:

        user_input = input(
            'Target X Y: '
        ).strip()

        # Planner mode
        if user_input == '':

            print(
                'Planner mode selected.'
            )

            return None

        parts = user_input.replace(
            ',',
            ' '
        ).split()

        if len(parts) != 2:

            print(
                'Please enter exactly '
                'two numbers.'
            )

            continue

        try:

            x = float(
                parts[0]
            )

            y = float(
                parts[1]
            )

            print(
                f'Manual target selected: '
                f'({x:.2f}, {y:.2f}) m'
            )

            return (
                x,
                y
            )

        except ValueError:

            print(
                'Invalid coordinate. '
                'Use numbers such as: 10 5'
            )


# =============================================================
# Main
# =============================================================

def main(args=None):

    manual_target = (
        ask_for_manual_target()
    )

    rclpy.init(
        args=args
    )

    node = WaypointController(
        manual_target
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.stop()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
