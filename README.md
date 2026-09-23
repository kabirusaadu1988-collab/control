WAM-V Control Package

ROS 2 control package for the VRX autonomous surface vehicle platooning project.

This package currently implements the single-robot control stage of the project. It estimates the WAM-V state from simulated GPS and IMU measurements, computes waypoint-tracking errors, and commands the left and right thrusters.

The current controller should be treated as a working baseline / rough controller. It successfully closes the loop and moves the WAM-V toward a target, but further tuning is still required to reduce overshoot, circling, and drift near the final waypoint.

1. Current Architecture

VRX / Gazebo
    |
    | GPS
    | IMU
    v
state_estimator.py
    |
    | /wamv/local_pose
    | geometry_msgs/Pose2D
    v
control.py
    |
    | left/right thrust commands
    v
WAM-V thrusters

The controller can currently use either:

a manually entered waypoint; or

a ROS nav_msgs/Path trajectory from the Planner team.

Planner integration is temporarily waiting for a few coordinate-frame and CSV-publishing corrections on the Planner side.

2. Package Structure

control/
├── control/
│   ├── __init__.py
│   ├── control.py
│   └── state_estimator.py
├── resource/
├── test/
├── package.xml
├── setup.cfg
└── setup.py

Current ROS executables:

state_estimator
control_node

3. State Estimator

File:

control/state_estimator.py

The state estimator converts the simulated VRX sensor data into a simple local planar robot state:

x
y
psi

where:

x = local X position in metres;

y = local Y position in metres;

psi = WAM-V yaw / heading in radians.

Inputs

GPS:

/wamv/sensors/gps/gps/fix
sensor_msgs/msg/NavSatFix

IMU:

/wamv/sensors/imu/imu/data
sensor_msgs/msg/Imu

Output

/wamv/local_pose
geometry_msgs/msg/Pose2D

The first valid GPS reading becomes the local origin:

x = 0
y = 0

GPS latitude and longitude differences are converted into local Cartesian distances in metres.

The IMU quaternion is converted into yaw.

4. Coordinate Frame

The local state-estimator frame was experimentally compared with Gazebo motion.

The tests showed that the frames are aligned:

Gazebo +X = Local GPS +X
Gazebo +Y = Local GPS +Y

psi = 0       -> +X
psi = +pi/2   -> +Y
psi = pi      -> -X
psi = -pi/2   -> -Y

Therefore, no axis swapping, sign inversion, or rotation is currently required between Gazebo XY and the controller's local XY frame.

Only the origin must be translated so that the initial WAM-V position becomes:

(0, 0)

5. Controller

File:

control/control.py

The controller currently performs waypoint tracking.

It subscribes to:

/wamv/local_pose

and publishes:

/wamv/thrusters/left/thrust
/wamv/thrusters/right/thrust

Both thruster topics use:

std_msgs/msg/Float64

6. Manual Waypoint Mode

When the controller is launched normally:

ros2 run control control_node

it asks:

Target X Y:

Example:

10 0

This commands the WAM-V toward:

X = 10 m
Y = 0 m

in the local coordinate frame.

Pressing Enter without entering coordinates selects Planner mode.

7. Current Planner Interface

The controller was originally prepared to receive:

nav_msgs/msg/Path

from the Planner team.

The Planner team's current publisher uses:

/coverage_path

The control package will be aligned with this topic during the next controller revision.

Important

Planner integration should not be enabled for autonomous motion until the Planner team finishes the following corrections:

publish local coordinates instead of raw Gazebo world coordinates;

use the agreed local frame_id instead of world;

fix the coverage CSV parsing so waypoint_id is not interpreted as X;

add nav_msgs and geometry_msgs dependencies to the Planner package.

Once these are corrected, Control should only follow the path supplied by Planner. Path generation, A*, occupancy mapping, and trajectory creation remain Planner responsibilities.

8. Current Control Mathematics

For the current waypoint:

Target = (Xd, Yd)
Robot  = (X, Y)

World-frame error:

ex = Xd - X
ey = Yd - Y

Distance error:

distance = sqrt(ex^2 + ey^2)

Desired heading:

psi_d = atan2(ey, ex)

Heading error:

e_psi = wrap(psi_d - psi)

The heading error is wrapped to:

[-pi, pi]

9. Body-Frame Error

The position error is also transformed from the world frame into the WAM-V body frame using:

e_body = R(psi)^T * e_world

which gives approximately:

error_forward
error_lateral

This tells the controller whether the target is:

ahead or behind the robot;

left or right of the robot.

10. Current P Controller

The current rough controller uses proportional control.

Forward command:

F = Kp_distance * distance

Turning command:

M = Kp_heading * heading_error

Differential thrust:

Left  = F - M
Right = F + M

Thruster commands are limited to:

[-max_thrust, +max_thrust]

11. Heading-Based Forward Reduction

The current controller reduces forward thrust when the WAM-V is poorly aligned with the target.

It uses the heading error to scale the forward command.

This prevents the boat from applying maximum forward thrust while pointing far away from the waypoint.

However, the current implementation still allows too much forward motion during large heading corrections, so a stronger turn-before-drive rule will be added in Controller v2.

12. Motion Safety Mode

The controller contains an enable_motion ROS parameter.

Default:

enable_motion = false

In this mode, all control calculations are performed and displayed, but zero thrust is sent to Gazebo.

To enable actual motion:

ros2 run control control_node --ros-args \
-p enable_motion:=true

Maximum thrust can also be changed:

ros2 run control control_node --ros-args \
-p enable_motion:=true \
-p max_thrust:=100.0

A maximum thrust of 100 has already been sufficient to move the WAM-V during testing.

13. Current Experimental Result

The current P controller successfully performs closed-loop autonomous movement.

Verified chain:

GPS
 -> state estimator
 -> local position

IMU
 -> heading

local state
 -> control error

control error
 -> thrust commands

thrusters
 -> WAM-V movement

new sensor measurements
 -> controller feedback

Therefore the basic ROS control loop is operational.

14. Known Controller Limitation

During the first autonomous test toward:

(10, 0)

the WAM-V approached the target but eventually began circling around it.

The boat typically struggled to reduce the final error below approximately a few metres and performed repeated orbiting / "donut" motion.

This is mainly caused by:

WAM-V inertia;

forward motion while strongly misaligned;

no derivative damping;

excessive approach speed near the target;

the desired bearing changing rapidly after overshooting the waypoint.

This is now considered a controller tuning problem, not a ROS communication problem.

15. Planned Controller v2

The next controller revision will introduce several improvements.

A. PD Heading Control

Instead of differentiating heading numerically, the controller will use the IMU's measured yaw rate:

/wamv/sensors/imu/imu/data
angular_velocity.z

The heading controller will become:

M = Kp_heading * e_psi - Kd_yaw_rate * r

where:

r = measured yaw rate

The derivative term should provide damping and reduce oscillation and wild heading swings.

Integral action will not be added initially.

The planned sequence is:

P
 -> PD
 -> tune
 -> add I only if required
 -> investigate self tuning

B. Turn Before Driving

If the WAM-V is badly misaligned with the target:

|heading error| > threshold

forward thrust will temporarily be set to zero.

Example initial threshold:

30 degrees

Behaviour:

large heading error
 -> rotate first

heading becomes acceptable
 -> move forward

C. Distance Gain Scheduling

Forward thrust will decrease as the WAM-V approaches the waypoint.

Conceptually:

far from waypoint
 -> high forward authority

medium distance
 -> reduced thrust

close to waypoint
 -> low thrust

This should significantly reduce final overshoot.

D. Heading Gain Scheduling

Heading control authority may also vary with heading error.

Conceptually:

large heading error
 -> stronger heading correction

small heading error
 -> gentler heading correction

This is a simple gain-scheduling approach and is different from full automatic/self tuning.

E. Final Settling Logic

The final target should not immediately be declared complete just because the WAM-V briefly enters the waypoint tolerance.

The planned logic will also consider:

position error
ground speed
yaw rate
settling time

The target should only be considered complete once the WAM-V is close enough and approximately stationary for a short period.

If inertia carries the boat back outside the tolerance, the controller should resume correction.

16. Controller Parameters

Controller parameters should remain configurable through ROS instead of being permanently hard-coded.

Planned tunable parameters include:

kp_distance
kp_heading
kd_yaw_rate

max_thrust
max_forward_thrust

rotate_threshold_deg

slowdown_distance
minimum_approach_scale

heading_gain_schedule

waypoint_tolerance
final_tolerance

settle_speed_threshold
settle_yaw_rate_threshold
settle_time

The initial values will be manually tuned.

Once the baseline controller works reliably, the team plans to investigate self-tuning controller gains.

17. Dead Reckoning / Accelerometer Use

Accelerometer-based dead reckoning is not currently used as the main position source.

The planned sensor roles are:

GPS
 -> absolute X,Y position

IMU orientation
 -> heading psi

IMU angular velocity
 -> yaw-rate damping

accelerometer
 -> possible future velocity/state-estimation enhancement

Pure accelerometer dead reckoning is not planned as a replacement for GPS because integrating acceleration to velocity and position introduces drift.

Sensor fusion or improved velocity estimation may be investigated later.

18. Building the Package

From the workspace:

cd ~/vrx_ws

Source ROS 2:

source /opt/ros/jazzy/setup.bash

Build:

colcon build --merge-install

Source the workspace:

source install/setup.bash

19. Current Manual Experiment Procedure

At the moment the system is normally run with three terminals.

Terminal 1 - Gazebo / VRX

cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

Wait for the WAM-V and world to finish loading.

Terminal 2 - State Estimator

cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run control state_estimator

Optional check:

ros2 topic echo /wamv/local_pose

Terminal 3 - Controller

Dry run:

cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run control control_node

Motion enabled:

ros2 run control control_node --ros-args \
-p enable_motion:=true \
-p max_thrust:=100.0

Example target:

10 0

20. Important State-Estimator Note

The first GPS reading defines:

(0,0)

Therefore, during a test, do not restart the state estimator after the WAM-V has already moved unless the simulation is also reset.

Restarting only the state estimator would create a new local origin at the robot's current position and shift the controller coordinate frame.

21. Planned Launch File

A launch file is planned so that the experiment does not require multiple terminals.

Target structure:

control/
├── launch/
│   └── wamv_experiment.launch.py
└── config/
    └── controller.yaml

The launch file should eventually start:

VRX / Gazebo
state_estimator
control_node
optional Planner trajectory publisher

from one command.

Example planned usage:

ros2 launch control wamv_experiment.launch.py \
input_mode:=manual \
target_x:=10.0 \
target_y:=0.0 \
enable_motion:=true

Planner mode will be enabled after the Planner interface corrections are completed.

22. Current Development Status

Completed:

ROS 2 workspace
VRX / Gazebo simulation
WAM-V launch
GPS topic identification
IMU topic identification
thruster topic identification
manual thruster tests
local GPS conversion
IMU yaw conversion
coordinate-frame validation
state_estimator node
manual waypoint input
world-frame error
body-frame error
desired heading
heading error wrapping
basic P controller
differential thrust
thruster saturation
motion safety mode
first autonomous waypoint test

In progress:

PD damping
approach-speed reduction
turn-before-drive
gain scheduling
final settling
controller tuning
launch automation
Planner integration

Future:

straight trajectory tracking
curved trajectory tracking
simulation data recording
tracking-error analysis
performance metrics
self-tuning controller
leader-follower control
three-robot platooning
disturbance/current testing

23. Git Workflow

Before making major controller changes, commit the working baseline.

Example:

cd ~/vrx_ws/src/control

git status

git add .

git commit -m "Add rough single-robot waypoint controller"

git push origin master

Future controller revisions should be committed separately so the progression from the basic P controller to the improved PD / gain-scheduled controller remains visible in Git history.
