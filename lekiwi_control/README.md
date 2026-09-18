# `lekiwi_control`

Kinematics engine, workspace reachability checking, and base standoff calculation package for LeKiwi.

---

## 🧩 Modules & Executables

### 1. `kinematics_engine.py`
Pure Python closed-form analytical kinematics and reachability math:
- Forward kinematics (FK) and 3-DOF planar arm inverse kinematics (IK).
- Joint angle limit checking against hardware constraints.
- Base standoff pose calculation (`compute_standoff_pose`, `find_common_standoff_pose`) for mobile repositioning when a chess target square is out of manipulator reach.

### 2. `workspace_checker` (`WorkspaceCheckerNode`)
ROS 2 service node providing workspace feasibility evaluation:
- Service `/workspace/check_reachability` (`lekiwi_interfaces/srv/CheckMoveFeasibility`):
  - Validates pick and place target reachability via TF2 and analytical IK.
  - Generates recommended base repositioning poses (`recommended_base_pose`, `secondary_base_pose`) if targets are unreachable from current base pose.
  - Returns joint position hints (`pick_ik_hint`, `place_ik_hint`) for policy servers.
- Dynamic parameter re-synchronization with URDF and TF transforms.
- Diagnostic reporting on `/diagnostics`.

---

## 📡 Topics & Services

### Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Frame transforms for base and manipulator links. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Arm joint states for current position feedback. |

### Published Topics
| Topic | Type | Description |
|---|---|---|
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | System diagnostics on kinematics engine readiness. |

### Services
| Name | Type | Description |
|---|---|---|
| `/workspace/check_reachability` | `lekiwi_interfaces/srv/CheckMoveFeasibility` | Feasibility check and IK hints / base standoff calculation. |

---

## 🧪 Testing

```bash
colcon test --packages-select lekiwi_control --event-handlers console_cohesion+
colcon test-result --test-result-base build/lekiwi_control --verbose
```
