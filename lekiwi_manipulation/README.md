# `lekiwi_manipulation`

Physical-AI policy execution, LeRobot imitation learning bridge, and robotic arm trajectory manipulation for the LeKiwi robot.

---

## 🏛️ Architecture & Design Patterns (GoF & SOLID)

`lekiwi_manipulation` is crafted adhering to Clean Code and GoF Design Patterns:

1. **Strategy Pattern (`ITrajectoryStrategy` in `trajectory_generator.py`)**:
   - `QuinticSplineStrategy`: Computes smooth, C2-continuous (Zero-Jerk) 5th order polynomial trajectories ($v(0)=v(1)=0, a(0)=a(1)=0$) for safe, non-abrupt arm motion.
   - Decoupled from ROS Graph: mathematically deterministic and testable.
2. **Adapter Pattern (`LeRobotArmBridge`)**:
   - Adapts between HuggingFace LeRobot's raw tick ($[0..4095]$) / degrees representation and standard ROS 2 `control_msgs/action/FollowJointTrajectory` and `sensor_msgs/msg/JointState`.
   - Protects downstream nodes from sensor/servo hardware communication specifics.
3. **Action Server & Phased Execution (`MockPolicyServer`)**:
   - Implements the `/manipulation/execute_chess_move` Action Server (`lekiwi_interfaces/action/ExecuteChessMove`).
   - Governs the sequential atomic phases:
     `APPROACH_PICK` $\to$ `DESCEND_PICK` $\to$ `GRASP` $\to$ `LIFT` $\to$ `TRANSIT_PLACE` $\to$ `DESCEND_PLACE` $\to$ `RELEASE` $\to$ `RETRACT_STOW`.
   - Supports preemption / graceful cancel and periodic progress feedback.
4. **Physical-AI Skeleton (`SmolVlaPolicyServer`)**:
   - Production runner for PyTorch / LeRobot SmolVLA policy weights with wrist camera visual conditioning.

---

## 🧩 Nodes & Executables

| Executable | Class | Description |
|---|---|---|
| `mock_policy_server` | `MockPolicyServer` | Headless & simulation action server executing multi-phase chess trajectories |
| `lerobot_arm_bridge` | `LeRobotArmBridge` | Bridges `/lerobot/arm_action` and `/joint_states` to `FollowJointTrajectory` |
| `smolvla_policy_server` | `SmolVlaPolicyServer` | Physical-AI policy inference node (SmolVLA / LeRobot) |

---

## 📡 Interfaces & Communication

### Action Servers
- `/manipulation/execute_chess_move` (`lekiwi_interfaces/action/ExecuteChessMove`):
  - **Goal**: `instruction`, `pick_point`, `place_point`, `is_capture`, `pick_ik_hint`, `place_ik_hint`.
  - **Result**: `success`, `message`, `execution_time_sec`.
  - **Feedback**: `current_phase`, `progress_percent`.

### Action Clients
- `/arm_trajectory_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`):
  - Dispatches interpolated trajectory waypoints to ros2_control hardware interface.

### Topics
- `/lerobot/arm_action` (`sensor_msgs/msg/JointState`): Incoming policy actions.
- `/lerobot/arm_observation` (`sensor_msgs/msg/JointState`): Calibrated arm observations for LeRobot.
- `/joint_states` (`sensor_msgs/msg/JointState`): Hardware feedback from joint state broadcaster.
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Telemetry on manipulation health and active phase.

---

## 🚀 Launch & Usage

```bash
# Launch manipulation layer in Docker simulation mode (Mock)
ros2 launch lekiwi_manipulation manipulation.launch.py use_mock:=true

# Launch on physical robot with real SmolVLA AI policy
ros2 launch lekiwi_manipulation manipulation.launch.py use_mock:=false
```

---

## 🧪 Testing

Run pytest automated unit tests:

```bash
colcon test --packages-select lekiwi_manipulation --event-handlers console_cohesion+
colcon test-result --test-result-base build/lekiwi_manipulation --verbose
```

