# Tutorial 2: Robot Bringup & Mock Simulation

In this tutorial, you will launch the LeKiwi robot model using mock hardware components, inspect kinematic transforms in RViz2, and send your first joint commands to the `ros2_control` stack.

---

## What You Will Learn
- How to launch LeKiwi with `hardware_type:=mock`.
- How to inspect the complete robot Coordinate Transform (TF) tree.
- How to publish target positions to the arm joint trajectory controller.

---

## Step 1: Launch Mock Bringup

Run the master bringup launch file with mock hardware enabled:

```bash
source ~/lekiwi_ws/install/setup.bash
ros2 launch lekiwi_bringup robot.launch.py hardware_type:=mock
```

To visualize the robot in RViz2 in a separate terminal:

```bash
rviz2 -d $(ros2 pkg prefix lekiwi_description)/share/lekiwi_description/rviz/view_robot.rviz
```

### What is happening behind the scenes?
1. `lekiwi_description` parses `lekiwi_robot.urdf.xacro`, expanding joint limits and inertial properties with `hardware_type:=mock`.
2. `robot_state_publisher` begins publishing link transforms on `/tf` and `/tf_static`.
3. `ros2_control_node` starts mock system components instead of the physical serial bus `/dev/lekiwi_serial`.
4. The controller spawner brings up `joint_state_broadcaster`, `arm_trajectory_controller`, and `omni_base_controller`.

---

## Step 2: Verify Active Controllers and Topics

Open a second terminal window and query the ROS 2 Controller Manager:

```bash
ros2 control list_controllers
```

**Expected Output:**
```text
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
arm_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
omni_base_controller[omni_wheel_drive_controller/OmniWheelDriveController] active
```

Inspect published joint states:

```bash
ros2 topic echo /joint_states --once
```

You should observe position, velocity, and effort values for all 6 arm joints (`arm_shoulder_pan`, `arm_shoulder_lift`, `arm_elbow_flex`, `arm_wrist_flex`, `arm_wrist_roll`, `arm_gripper`) and 3 omni wheel joints (`base_left_wheel`, `base_back_wheel`, `base_right_wheel`).

---

## Step 3: Verify the Coordinate Transform (TF) Tree

Confirm that all frames connect properly from `base_footprint` and `baseframe` down to the gripper tool frame `gripperframe`:

```bash
ros2 run tf2_tools view_frames
```

This generates `frames.pdf`. Alternatively, check the live transform between base and gripper tip:

```bash
ros2 run tf2_ros tf2_echo baseframe gripperframe
```

**Expected Output:**
```text
At time 1728638400.123456789
- Translation: [0.185, 0.000, 0.224]
- Rotation: in Quaternion [0.000, 0.707, 0.000, 0.707]
```

---

## Step 4: Command a Test Motion

Test the controller response by sending a trajectory point to `arm_trajectory_controller` using the robot's authentic joint identifiers:

```bash
ros2 topic pub --once /arm_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: "baseframe"},
  joint_names: [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper"
  ],
  points: [{
    positions: [0.0, 0.2, -0.3, 0.1, 0.0, 0.0],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}'
```

In the RViz2 window, observe the simulated 6-DoF arm smoothly articulating to the target position.

---

## Summary
You have verified that the robot kinematic chain, controller manager, and coordinate transforms function correctly in mock mode.

**Next Step:** Proceed to [Tutorial 3: Autonomous Chess Mission](03-autonomous-chess-match.md) to explore the game orchestration system.
