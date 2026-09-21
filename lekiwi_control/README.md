# lekiwi_control

C++17 `ament_cmake` package: three composable nodes (`tf_gatekeeper_node`, `workspace_checker_node`, `torque_manager`) in `src/` and `include/`, with GTests in `test/`. Launch and robot configuration belong to `lekiwi_bringup`.

## Launch

```bash
ros2 launch lekiwi_bringup control.launch.py
ros2 launch lekiwi_bringup control.launch.py enable_readiness_checks:=false
```

Only `enable_readiness_checks` and `use_sim_time` are launch arguments. Torque manager always starts; the combined switch controls both workspace checker and TF gatekeeper within the multithreaded component container `lekiwi_control_container`. This launch does not start hardware, orchestration, or manipulation.

| Component | Generated executable |
|---|---|
| `lekiwi_control::TorqueManagerNode` | `torque_manager` |
| `lekiwi_control::WorkspaceCheckerNode` | `workspace_checker_node` |
| `lekiwi_control::TfGatekeeperNode` | `tf_gatekeeper_node` |

`rclcpp_components_register_node` generates executable entry points using a single-threaded executor. All components can be loaded into an `rclcpp_components` container with intra-process communication.

For composition, load either plugin into an `rclcpp_components` container and pass the same parameters produced by the bringup configuration loader. Torque manager requires `joint_names`, `arm_joints`, `base_joints`, `torque_controller_topic`, and `startup_policy=preserve`. Its default mutually exclusive callback group serializes command state access even in a multithreaded container. Unloading and reloading forgets all requested torque state and emits no command.

## Configuration and boundaries

Edit `lekiwi_bringup/config/control/lekiwi_controllers.yaml`. The launch loader supplies wheel-center radius, base frame, and joint ordering from controller configuration to the relevant nodes. Nodes do not query controller parameters at runtime. Configuration is immutable for each node lifetime.

- `workspace/types.hpp`: configuration and planning values.
- `workspace/kinematics_engine.hpp`: pure analytical IK/FK and planar geometry.
- `workspace/workspace_planner.hpp`: pure zero-navigation, single-base, dual-base, and capture decisions.
- `workspace_checker_node.cpp`: ROS parameters, a consistent TF snapshot, service conversion, and diagnostics; installed as `workspace_checker_node` and the `lekiwi_control::WorkspaceCheckerNode` component.
- `readiness_policy.hpp`: pure freshness, standstill, and covariance predicates used by the lifecycle node.
- `torque_command_state.hpp`: validates joint groups and tracks complete requested command vectors.
- `torque_manager_node.cpp`: read-only configuration, torque service and reliable, volatile depth-one command publisher.

The analytical arm is an approximation with explicit link lengths and operating joint limits; it is not a full URDF solver or collision checker. Wheel-center radius is not the footprint radius: footprint padding and planning clearance are separate. Board height and padding retain existing nominal calibration and must be checked against the robot. Capture planning checks the pick target only, not an onboard drop trajectory.

## Interfaces

| Node | Interface | Behavior |
|---|---|---|
| workspace_checker | `/workspace/check_move_feasibility` | Returns IK hints and map-frame base poses; rejects missing, stale, nonplanar TF and invalid input |
| tf_gatekeeper_node | `/system/tf_ready`, `/system/check_tf_readiness` | Readiness heartbeat and query; fresh odometry, covariance, joints and TF required |
| torque_manager | `/set_torque_enabled` | Submits name-mapped torque commands; see startup policy below |

Gatekeeper configures and activates through launch lifecycle handlers. EKF bootstrap uses `std_srvs/srv/Empty`, bounded waits and retry backoff. Deactivation clears observations and pending bootstrap state; reactivation requires fresh data. Readiness is a heartbeat, not a persistent authorization: consumers must expire it if the publisher disappears.

Torque startup policy is `preserve`: startup and shutdown emit no torque command. State is unknown until an explicit `ALL` request initializes the complete command vector; partial requests are rejected beforehand. A successful response acknowledges command submission, not hardware application. The hardware driver's own startup behavior is unchanged.

## Validation

```bash
colcon test --packages-select lekiwi_control lekiwi_bringup lekiwi_orchestrator
colcon test-result --verbose
```

GTests cover pure planning, invalid configuration, torque ordering and the production C++ readiness policy. Bringup tests cover the installed executables, service/topic contracts, component load/unload and lifecycle transitions on a private ROS graph with fake sensors and services. They do not establish hardware calibration or physical collision safety.
