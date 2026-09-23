import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class WaypointController(Node):

    def __init__(self, manual_target=None):

        super().__init__('waypoint_controller')

        # =====================================================
        # Controller parameters
        # =====================================================

        # Forward-position controller
        self.declare_parameter(
            'kp_distance',
            8.0
        )

        # -----------------------------------------------------
        # Heading PD controller
        # -----------------------------------------------------

        self.declare_parameter(
            'kp_heading',
            40.0
        )

        self.declare_parameter(
            'kd_yaw_rate',
            15.0
        )

        # Increase heading authority in DRIVE mode as
        # heading error becomes larger.
        self.declare_parameter(
            'drive_heading_boost',
            2.0
        )

        # Maximum allowed turning command before
        # differential-thrust mixing.
        self.declare_parameter(
            'max_turning_command',
            70.0
        )

        # -----------------------------------------------------
        # Thruster limits
        # -----------------------------------------------------

        self.declare_parameter(
            'max_thrust',
            100.0
        )

        self.declare_parameter(
            'max_forward_thrust',
            80.0
        )

        self.declare_parameter(
            'min_forward_thrust',
            15.0
        )

        # -----------------------------------------------------
        # Turn-before-drive hysteresis
        # -----------------------------------------------------

        self.declare_parameter(
            'rotate_enter_deg',
            45.0
        )

        self.declare_parameter(
            'rotate_exit_deg',
            20.0
        )

        # -----------------------------------------------------
        # Distance slowdown
        # -----------------------------------------------------

        self.declare_parameter(
            'slowdown_distance',
            5.0
        )

        self.declare_parameter(
            'min_approach_scale',
            0.20
        )

        # -----------------------------------------------------
        # Waypoint tolerance
        # -----------------------------------------------------

        self.declare_parameter(
            'waypoint_tolerance',
            1.0
        )

        # -----------------------------------------------------
        # Active braking
        # -----------------------------------------------------

        self.declare_parameter(
            'brake_gain',
            35.0
        )

        self.declare_parameter(
            'max_brake_thrust',
            40.0
        )

        # -----------------------------------------------------
        # Final settling
        # -----------------------------------------------------

        self.declare_parameter(
            'settle_speed_threshold',
            0.15
        )

        self.declare_parameter(
            'settle_yaw_rate_threshold',
            0.08
        )

        self.declare_parameter(
            'settle_time',
            1.5
        )

        # -----------------------------------------------------
        # Safety
        # -----------------------------------------------------

        self.declare_parameter(
            'enable_motion',
            False
        )

        # =====================================================
        # Read parameters
        # =====================================================

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

        self.kd_yaw_rate = float(
            self.get_parameter(
                'kd_yaw_rate'
            ).value
        )

        self.drive_heading_boost = float(
            self.get_parameter(
                'drive_heading_boost'
            ).value
        )

        self.max_turning_command = float(
            self.get_parameter(
                'max_turning_command'
            ).value
        )

        self.max_thrust = float(
            self.get_parameter(
                'max_thrust'
            ).value
        )

        self.max_forward_thrust = float(
            self.get_parameter(
                'max_forward_thrust'
            ).value
        )

        self.min_forward_thrust = float(
            self.get_parameter(
                'min_forward_thrust'
            ).value
        )

        self.rotate_enter_threshold = math.radians(
            float(
                self.get_parameter(
                    'rotate_enter_deg'
                ).value
            )
        )

        self.rotate_exit_threshold = math.radians(
            float(
                self.get_parameter(
                    'rotate_exit_deg'
                ).value
            )
        )

        self.slowdown_distance = float(
            self.get_parameter(
                'slowdown_distance'
            ).value
        )

        self.min_approach_scale = float(
            self.get_parameter(
                'min_approach_scale'
            ).value
        )

        self.waypoint_tolerance = float(
            self.get_parameter(
                'waypoint_tolerance'
            ).value
        )

        self.brake_gain = float(
            self.get_parameter(
                'brake_gain'
            ).value
        )

        self.max_brake_thrust = float(
            self.get_parameter(
                'max_brake_thrust'
            ).value
        )

        self.settle_speed_threshold = float(
            self.get_parameter(
                'settle_speed_threshold'
            ).value
        )

        self.settle_yaw_rate_threshold = float(
            self.get_parameter(
                'settle_yaw_rate_threshold'
            ).value
        )

        self.settle_time = float(
            self.get_parameter(
                'settle_time'
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

        self.manual_mode = (
            manual_target is not None
        )

        if manual_target is not None:

            self.waypoints = [
                manual_target
            ]

            self.path_source = 'manual'

        # =====================================================
        # Robot state
        # =====================================================

        self.current_x = None
        self.current_y = None
        self.current_psi = None

        self.yaw_rate = 0.0

        # =====================================================
        # Velocity estimation
        # =====================================================

        self.previous_x = None
        self.previous_y = None
        self.previous_pose_time = None

        self.ground_speed = 0.0

        self.forward_speed = 0.0

        # =====================================================
        # Controller state
        # =====================================================

        self.rotate_only = False

        # =====================================================
        # Final settling state
        # =====================================================

        self.settle_start_time = None

        # =====================================================
        # ROS subscriptions
        # =====================================================

        self.pose_sub = self.create_subscription(
            Pose2D,
            '/wamv/local_pose',
            self.pose_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10
        )

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

        # =====================================================
        # Logging
        # =====================================================

        self.last_log_time = 0.0

        # =====================================================
        # Startup messages
        # =====================================================

        self.get_logger().info(
            'Waypoint controller v2 started.'
        )

        self.get_logger().info(
            'PD heading + heading gain scheduling '
            '+ slowdown + rotate hysteresis + active braking'
        )

        self.get_logger().info(
            f'Kp distance       = '
            f'{self.kp_distance:.2f}'
        )

        self.get_logger().info(
            f'Kp heading        = '
            f'{self.kp_heading:.2f}'
        )

        self.get_logger().info(
            f'Kd yaw rate       = '
            f'{self.kd_yaw_rate:.2f}'
        )

        self.get_logger().info(
            f'Drive boost       = '
            f'{self.drive_heading_boost:.2f}'
        )

        self.get_logger().info(
            f'Max turn command  = '
            f'{self.max_turning_command:.2f}'
        )

        self.get_logger().info(
            f'Max thrust        = '
            f'{self.max_thrust:.2f}'
        )

        self.get_logger().info(
            f'Max forward       = '
            f'{self.max_forward_thrust:.2f}'
        )

        self.get_logger().info(
            f'Min forward       = '
            f'{self.min_forward_thrust:.2f}'
        )

        self.get_logger().info(
            f'Rotate enter      = '
            f'{math.degrees(self.rotate_enter_threshold):.1f} deg'
        )

        self.get_logger().info(
            f'Rotate exit       = '
            f'{math.degrees(self.rotate_exit_threshold):.1f} deg'
        )

        self.get_logger().info(
            f'Slowdown distance = '
            f'{self.slowdown_distance:.2f} m'
        )

        self.get_logger().info(
            f'Tolerance         = '
            f'{self.waypoint_tolerance:.2f} m'
        )

        self.get_logger().info(
            f'Brake gain        = '
            f'{self.brake_gain:.2f}'
        )

        if manual_target is not None:

            self.get_logger().info(
                f'Manual target loaded: '
                f'X={manual_target[0]:.2f} m, '
                f'Y={manual_target[1]:.2f} m'
            )

        else:

            self.get_logger().info(
                'Waiting for planner trajectory.'
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
    # IMU callback
    # =========================================================

    def imu_callback(self, msg):

        self.yaw_rate = float(
            msg.angular_velocity.z
        )

    # =========================================================
    # Planner callback
    # =========================================================

    def path_callback(self, msg):

        if self.manual_mode:
            return

        if len(msg.poses) == 0:

            self.get_logger().warn(
                'Received empty planner path.'
            )

            return

        new_waypoints = []

        for pose_stamped in msg.poses:

            x = float(
                pose_stamped.pose.position.x
            )

            y = float(
                pose_stamped.pose.position.y
            )

            new_waypoints.append(
                (x, y)
            )

        self.waypoints = new_waypoints

        self.current_waypoint_index = 0

        self.path_source = 'planner'

        self.path_complete = False

        self.rotate_only = False

        self.settle_start_time = None

        self.get_logger().info(
            f'Planner path received: '
            f'{len(self.waypoints)} waypoints.'
        )

    # =========================================================
    # Pose callback
    # =========================================================

    def pose_callback(self, msg):

        now = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        new_x = float(msg.x)
        new_y = float(msg.y)
        new_psi = float(msg.theta)

        # -----------------------------------------------------
        # Estimate translational velocity
        # -----------------------------------------------------

        if (
            self.previous_x is not None
            and
            self.previous_y is not None
            and
            self.previous_pose_time is not None
        ):

            dt = (
                now
                - self.previous_pose_time
            )

            if dt > 0.001:

                vx_world = (
                    new_x
                    - self.previous_x
                ) / dt

                vy_world = (
                    new_y
                    - self.previous_y
                ) / dt

                measured_ground_speed = (
                    math.hypot(
                        vx_world,
                        vy_world
                    )
                )

                measured_forward_speed = (
                    math.cos(new_psi)
                    * vx_world
                    +
                    math.sin(new_psi)
                    * vy_world
                )

                # Low-pass filter
                alpha = 0.30

                self.ground_speed = (
                    alpha
                    * measured_ground_speed
                    +
                    (1.0 - alpha)
                    * self.ground_speed
                )

                self.forward_speed = (
                    alpha
                    * measured_forward_speed
                    +
                    (1.0 - alpha)
                    * self.forward_speed
                )

        self.current_x = new_x
        self.current_y = new_y
        self.current_psi = new_psi

        self.previous_x = new_x
        self.previous_y = new_y
        self.previous_pose_time = now

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
    # Main controller
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
        # Position error
        # -----------------------------------------------------

        error_x_world = (
            target_x - x
        )

        error_y_world = (
            target_y - y
        )

        distance_error = math.hypot(
            error_x_world,
            error_y_world
        )

        # -----------------------------------------------------
        # Is this the final waypoint?
        # -----------------------------------------------------

        is_final_waypoint = (
            self.current_waypoint_index
            ==
            len(self.waypoints) - 1
        )

        # -----------------------------------------------------
        # Waypoint reached
        # -----------------------------------------------------

        if (
            distance_error
            <= self.waypoint_tolerance
        ):

            if not is_final_waypoint:

                self.advance_waypoint(
                    distance_error
                )

                return

            self.brake_and_settle(
                distance_error
            )

            return

        self.settle_start_time = None

        # -----------------------------------------------------
        # Desired heading
        # -----------------------------------------------------

        desired_heading = math.atan2(
            error_y_world,
            error_x_world
        )

        heading_error = self.wrap_angle(
            desired_heading - psi
        )

        # -----------------------------------------------------
        # World -> body error
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

        # =====================================================
        # FORWARD CONTROL
        # =====================================================

        forward_command = (
            self.kp_distance
            * distance_error
        )

        forward_command = self.clamp(
            forward_command,
            0.0,
            self.max_forward_thrust
        )

        # -----------------------------------------------------
        # Distance-based slowdown
        # -----------------------------------------------------

        if (
            distance_error
            < self.slowdown_distance
        ):

            ratio = self.clamp(
                distance_error
                / self.slowdown_distance,
                0.0,
                1.0
            )

            approach_scale = (
                self.min_approach_scale
                +
                (
                    1.0
                    - self.min_approach_scale
                )
                * ratio
            )

        else:

            approach_scale = 1.0

        forward_command *= (
            approach_scale
        )

        forward_command = max(
            forward_command,
            self.min_forward_thrust
        )

        # =====================================================
        # ROTATE / DRIVE HYSTERESIS
        # =====================================================

        if (
            not self.rotate_only
            and
            abs(heading_error)
            >= self.rotate_enter_threshold
        ):

            self.rotate_only = True

        elif (
            self.rotate_only
            and
            abs(heading_error)
            <= self.rotate_exit_threshold
        ):

            self.rotate_only = False

        rotating_only = self.rotate_only

        # -----------------------------------------------------
        # Forward authority based on heading
        # -----------------------------------------------------

        if rotating_only:

            forward_command = 0.0

        else:

            # More aggressive than plain cosine.
            #
            # 0 deg  -> 100%
            # 30 deg -> ~75%
            # 40 deg -> ~59%
            heading_alignment = max(
                0.0,
                math.cos(
                    heading_error
                )
            )

            heading_alignment = (
                heading_alignment ** 2
            )

            forward_command *= (
                heading_alignment
            )

        # =====================================================
        # HEADING PD + GAIN SCHEDULING
        # =====================================================

        # Normalize heading error relative to the point
        # at which ROTATE mode begins.
        heading_ratio = self.clamp(
            abs(heading_error)
            /
            max(
                self.rotate_enter_threshold,
                1e-6
            ),
            0.0,
            1.0
        )

        # In DRIVE mode:
        #
        # small error -> normal Kp
        # large error -> progressively stronger Kp
        #
        # With boost=2:
        #
        # ratio=0.0 -> Kp x 1
        # ratio=0.5 -> Kp x 2
        # ratio=1.0 -> Kp x 3
        if rotating_only:

            heading_gain_scale = 1.0

        else:

            heading_gain_scale = (
                1.0
                +
                self.drive_heading_boost
                * heading_ratio
            )

        effective_kp_heading = (
            self.kp_heading
            * heading_gain_scale
        )

        # Proportional heading correction
        heading_p = (
            effective_kp_heading
            * heading_error
        )

        # Derivative damping:
        # oppose existing rotational velocity
        heading_d = (
            -self.kd_yaw_rate
            * self.yaw_rate
        )

        turning_command = (
            heading_p
            + heading_d
        )

        # Prevent excessively large differential commands
        turning_command = self.clamp(
            turning_command,
            -self.max_turning_command,
            self.max_turning_command
        )

        # =====================================================
        # THRUSTER MIXING
        # =====================================================

        left_thrust = (
            forward_command
            - turning_command
        )

        right_thrust = (
            forward_command
            + turning_command
        )

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

        # =====================================================
        # LOGGING
        # =====================================================

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
            heading_gain_scale,
            effective_kp_heading,
            heading_p,
            heading_d,
            turning_command,
            forward_command,
            approach_scale,
            rotating_only,
            left_thrust,
            right_thrust
        )

        # =====================================================
        # OUTPUT
        # =====================================================

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
    # Final braking and settling
    # =========================================================

    def brake_and_settle(
        self,
        distance_error
    ):

        current_time = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        if (
            self.ground_speed
            > self.settle_speed_threshold
            or
            abs(self.yaw_rate)
            > self.settle_yaw_rate_threshold
        ):

            self.settle_start_time = None

            brake_command = (
                -self.brake_gain
                * self.forward_speed
            )

            brake_command = self.clamp(
                brake_command,
                -self.max_brake_thrust,
                self.max_brake_thrust
            )

            turning_damping = (
                -self.kd_yaw_rate
                * self.yaw_rate
            )

            left_thrust = (
                brake_command
                - turning_damping
            )

            right_thrust = (
                brake_command
                + turning_damping
            )

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

            self.log_braking(
                distance_error,
                brake_command,
                left_thrust,
                right_thrust
            )

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

            return

        # Nearly stationary
        self.publish_thrusters(
            0.0,
            0.0
        )

        if self.settle_start_time is None:

            self.settle_start_time = (
                current_time
            )

            self.get_logger().info(
                'Inside final tolerance and nearly stationary. '
                'Starting settle timer.'
            )

            return

        if (
            current_time
            - self.settle_start_time
            >= self.settle_time
        ):

            self.path_complete = True

            self.get_logger().info(
                'FINAL TARGET REACHED AND SETTLED.'
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
            f'/{len(self.waypoints)} reached: '
            f'({target[0]:.2f}, '
            f'{target[1]:.2f}) '
            f'error={distance_error:.2f} m'
        )

        if (
            self.current_waypoint_index
            < len(self.waypoints) - 1
        ):

            self.current_waypoint_index += 1

            self.rotate_only = False

            next_target = self.waypoints[
                self.current_waypoint_index
            ]

            self.get_logger().info(
                f'Next waypoint: '
                f'({next_target[0]:.2f}, '
                f'{next_target[1]:.2f})'
            )

    # =========================================================
    # Helpers
    # =========================================================

    @staticmethod
    def wrap_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    @staticmethod
    def clamp(
        value,
        minimum,
        maximum
    ):

        return max(
            minimum,
            min(
                value,
                maximum
            )
        )

    # =========================================================
    # Thruster publisher
    # =========================================================

    def publish_thrusters(
        self,
        left,
        right
    ):

        if not rclpy.ok():
            return

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
    # Normal controller logging
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
        heading_gain_scale,
        effective_kp_heading,
        heading_p,
        heading_d,
        turning_command,
        forward_command,
        approach_scale,
        rotating_only,
        left_thrust,
        right_thrust
    ):

        current_time = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        if (
            current_time
            - self.last_log_time
            < 0.5
        ):
            return

        self.last_log_time = (
            current_time
        )

        control_mode = (
            'ROTATE'
            if rotating_only
            else 'DRIVE'
        )

        self.get_logger().info(
            '\n'
            f'Path source      : '
            f'{self.path_source}\n'
            f'Waypoint         : '
            f'{self.current_waypoint_index + 1}'
            f'/{len(self.waypoints)}\n'
            f'Control mode     : '
            f'{control_mode}\n'
            f'Position         : '
            f'X={self.current_x:7.2f} '
            f'Y={self.current_y:7.2f}\n'
            f'Ground speed     : '
            f'{self.ground_speed:7.3f} m/s\n'
            f'Forward speed    : '
            f'{self.forward_speed:7.3f} m/s\n'
            f'Heading          : '
            f'{self.current_psi:7.3f} rad '
            f'({math.degrees(self.current_psi):.2f} deg)\n'
            f'Yaw rate         : '
            f'{self.yaw_rate:7.3f} rad/s\n'
            f'Target           : '
            f'X={target_x:7.2f} '
            f'Y={target_y:7.2f}\n'
            f'World error      : '
            f'ex={error_x_world:7.2f} '
            f'ey={error_y_world:7.2f}\n'
            f'Body error       : '
            f'forward={error_forward:7.2f} '
            f'lateral={error_lateral:7.2f}\n'
            f'Distance         : '
            f'{distance_error:7.2f} m\n'
            f'Desired heading  : '
            f'{desired_heading:7.3f} rad '
            f'({math.degrees(desired_heading):.2f} deg)\n'
            f'Heading error    : '
            f'{heading_error:7.3f} rad '
            f'({math.degrees(heading_error):.2f} deg)\n'
            f'Heading gain     : '
            f'{heading_gain_scale:7.3f} x\n'
            f'Effective Kp     : '
            f'{effective_kp_heading:7.2f}\n'
            f'Heading P        : '
            f'{heading_p:7.2f}\n'
            f'Heading damping  : '
            f'{heading_d:7.2f}\n'
            f'Turning command  : '
            f'{turning_command:7.2f}\n'
            f'Approach scale   : '
            f'{approach_scale:7.3f}\n'
            f'Forward command  : '
            f'{forward_command:7.2f}\n'
            f'Thrust           : '
            f'L={left_thrust:7.2f} '
            f'R={right_thrust:7.2f}\n'
            f'Motion enabled   : '
            f'{self.enable_motion}'
        )

    # =========================================================
    # Braking logging
    # =========================================================

    def log_braking(
        self,
        distance_error,
        brake_command,
        left_thrust,
        right_thrust
    ):

        current_time = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

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
            'Control mode     : BRAKE\n'
            f'Distance         : '
            f'{distance_error:7.2f} m\n'
            f'Ground speed     : '
            f'{self.ground_speed:7.3f} m/s\n'
            f'Forward speed    : '
            f'{self.forward_speed:7.3f} m/s\n'
            f'Yaw rate         : '
            f'{self.yaw_rate:7.3f} rad/s\n'
            f'Brake command    : '
            f'{brake_command:7.2f}\n'
            f'Thrust           : '
            f'L={left_thrust:7.2f} '
            f'R={right_thrust:7.2f}'
        )

    # =========================================================
    # Safe shutdown
    # =========================================================

    def stop(self):

        if rclpy.ok():

            self.publish_thrusters(
                0.0,
                0.0
            )


# =============================================================
# Manual target input
# =============================================================

def ask_for_manual_target():

    print()
    print('=== WAM-V CONTROLLER v2 ===')
    print()
    print(
        'Coordinates use the LOCAL frame.'
    )
    print(
        'Robot start = (0, 0) metres.'
    )
    print()

    print(
        'Enter target as: X Y'
    )

    print(
        'Example: 10 0'
    )

    print()

    print(
        'Press ENTER to wait for '
        'a planner path.'
    )

    print()

    while True:

        user_input = input(
            'Target X Y: '
        ).strip()

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
                'Invalid coordinate.'
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

        if rclpy.ok():

            node.stop()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()