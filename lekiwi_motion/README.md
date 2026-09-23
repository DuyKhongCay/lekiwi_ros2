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
- `workspace/kinematics_model.hpp` and `src/workspace/kinematics_model.cpp`: strict URDF-chain extraction, typed extraction errors, full-chain FK, and analytical-model compatibility checks.
- `workspace/kinematics_engine.hpp`: analytical IK with both pan/elbow branches, URDF joint mapping, FK residual checks, and two-dimensional edge sampling.
- `workspace/workspace_planner.hpp`: pure zero-navigation, single-base, dual-base, and capture decisions.
- `workspace_checker_node.cpp`: ROS parameters, a consistent TF snapshot, service conversion, and diagnostics; installed as `workspace_checker_node` and the `lekiwi_control::WorkspaceCheckerNode` component.
- `readiness_policy.hpp`: pure freshness, standstill, and covariance predicates used by the lifecycle node.
- `torque_command_state.hpp`: validates joint groups and tracks complete requested command vectors.
- `torque_manager_node.cpp`: read-only configuration, torque service and reliable, volatile depth-one command publisher.

The analytical arm supports a vertical pan, three parallel horizontal pitch axes, and a wrist-roll axis aligned with the configured TCP approach axis. The extractor retains all fixed transforms, shoulder/lateral offsets, joint axes, and URDF angle conventions. Unsupported chains and invalid limits are rejected; no nominal arm model or tool length is substituted. Small CAD axis rounding is bounded by `kinematics.axis_tolerance`, and every successful IK result must pass full-chain position and orientation residual checks. This remains an endpoint feasibility checker, not a collision or trajectory checker.

`robot_description` supplied as a parameter is authoritative. Otherwise the node subscribes to the relative `robot_description` topic with reliable, transient-local QoS. Until a valid model arrives, requests return `feasible=false`. An invalid topic update invalidates the active model; a subsequent valid update restores readiness. Diagnostics report the model source, status, base frame and TCP frame. Model and service callbacks share the default mutually exclusive callback group.

The configured chain runs from `base_frame` to `kinematics.tip_frame` (`gripperframe`). The five `kinematics.joint_names` must match chain order; the moving-jaw joint is not part of this chain. `kinematics.approach_axis` and `kinematics.up_axis` are orthogonal unit vectors expressed in the TCP frame. The deployed frame uses +Y for approach and +Z for zero-roll up. Positive pitch raises the approach axis, negative pitch points it down, and roll rotates about approach. Yaw follows each analytical pan solution. The legacy service value `required_pitch_angle=0` still selects `planning.default_pitch`; the pure planner accepts an explicit zero pitch.

Workspace/model/frame/TF parameters are read-only for the node lifetime. Nonempty paired finite tag arrays determine board bounds and take precedence over `board.w/h`; set both arrays empty to use explicit dimensions. Invalid startup configuration is rejected before the feasibility service is created. `planning.max_samples` (1–1001, default 257) bounds candidate evaluations across all tiers: zero-navigation first, approximately half the remaining budget for a common base, and the rest for separate endpoints. Each candidate requires at most two IK calls. The sampler explores both edge directions and all four edges; search exhaustion means no candidate was found, not a proof of geometric impossibility.

Wheel-center radius is not the footprint radius: footprint padding and planning clearance are separate. Board height and padding must be checked against the robot. Capture planning checks the pick target only, not an onboard drop trajectory.

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
colcon build --base-paths lekiwi_ros2/lekiwi_description lekiwi_ros2/lekiwi_control --packages-select lekiwi_description lekiwi_control --symlink-install --cmake-args -GNinja -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select lekiwi_control --event-handlers console_cohesion+
colcon test-result --test-result-base build/lekiwi_control --verbose
```

Workspace GTests cover extraction failures, both IK branches, independent KDL FK against the expanded production xacro, standoff/dual-base planning, budgets, configuration validation and read-only parameters. Test paths are supplied by CMake. The board-coverage test reads the deployed control YAML and records the reachable count rather than assuming all 64 squares are reachable. The runtime pytest loads/unloads the installed component on a localhost ROS domain with synthetic TF and robot descriptions, testing model updates, stale/tilted/missing TF, retained delivery and service output. Other GTests cover torque ordering and the readiness policy. These checks establish L1–L3 software behavior, not hardware calibration or physical collision safety.
