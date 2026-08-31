# `lekiwi_control`

Python control package providing lifecycle boot orchestration, finite-state machine (FSM) mode transitions, and LeRobot robotic arm imitation learning policy bridging.

---

## 🧩 Modules & Executables

### 1. `task_orchestrator` (`TaskOrchestratorNode`)
Central coordinator managing startup and subsystem modes:
- Automatically bootstraps lifecycle nodes (`lekiwi_perception` components) from `unconfigured` -> `inactive` -> `active`.
- Hosts the `/orchestrator/set_mode` service to safely handle mode switching requests.
- Validates transitions against the 4-mode FSM rule matrix.
- Publishes the system-wide latched topic `/camera_mode` (`transient_local` QoS).

### 2. `lerobot_arm_bridge` (`LeRobotArmBridge`)
Translates LeRobot named arm joint observations and actions without requiring direct hardware device ownership:
- Listens for LeRobot actions on `/lerobot/arm_action` (supports `raw`, `radians`, or `degrees`).
- Computes goal points using calibration midpoints from `lekiwi_joints.yaml` and sends action goals to `/arm_controller/follow_joint_trajectory`.
- Listens to `/joint_states` and publishes calibrated raw observations on `/lerobot/arm_observation`.

### 3. `fsm.py` (Finite State Machine)
Defines legal operational state transitions:

```text
       +------------+
       |  STANDBY   | <----------+
       +-----+------+            |
             |                   |
             v                   |
       +------------+            |
+----> | NAVIGATING | -----------+
|      +-----+------+            |
|            |                   |
|            v                   |
|      +----------------+        |
+----> | CHESS_THINKING | -------+
|      +-----+----------+        |
|            |                   |
|            v                   |
|      +------------------------+|
+----> | MANIPULATION_LEROBOT   |+
       +------------------------+
```

---

## 📡 Topics & Services

### Published Topics
| Topic | Type | Description |
|---|---|---|
| `/camera_mode` | `lekiwi_interfaces/msg/CameraMode` | Latched current system camera mode. |
| `/lerobot/arm_observation` | `sensor_msgs/msg/JointState` | Calibrated raw arm joint feedback for LeRobot policies. |

### Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `/lerobot/arm_action` | `sensor_msgs/msg/JointState` | Action trajectory target from LeRobot. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Real-time joint positions from `joint_state_broadcaster`. |
| `/hailo_chess_inference/status` | `lekiwi_interfaces/msg/HailoInferenceStatus` | Perception status updates for health monitoring. |

### Services & Actions
| Name | Type | Description |
|---|---|---|
| `/orchestrator/set_mode` | `lekiwi_interfaces/srv/SetCamMode` | Service endpoint to trigger mode transitions. |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Action client executing arm trajectories on ros2_control. |

---

## 🧪 Testing

Run Python pytest unit tests for the FSM and orchestrator:

```bash
colcon test --packages-select lekiwi_control --event-handlers console_direct+
colcon test-result --verbose
```

