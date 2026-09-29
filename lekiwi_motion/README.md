# lekiwi_control

C++17 `ament_cmake` package: three composable nodes (`system_readiness_node`, `workspace_checker_node`, `torque_manager`) in `src/` and `include/`, with GTests in `test/`. Launch and robot configuration belong to `lekiwi_bringup`.

## Launch

```bash
ros2 launch lekiwi_bringup control.launch.py
ros2 launch lekiwi_bringup control.launch.py enable_readiness_checks:=false
```

Only `enable_readiness_checks` and `use_sim_time` are launch arguments. Torque manager always starts; the combined switch controls both workspace checker and system readiness supervisor within the multithreaded component container `lekiwi_control_container`. This launch does not start hardware, orchestration, or manipulation.

| Component | Generated executable |
| `lekiwi_motion::TorqueManagerNode` | `torque_manager` |
| `lekiwi_motion::WorkspaceCheckerNode` | `workspace_checker_node` |
| `lekiwi_motion::SystemReadinessNode` | `system_readiness_node` |

`rclcpp_components_register_node` generates executable entry points using a single-threaded executor. All components can be loaded into an `rclcpp_components` container with intra-process communication.

For composition, load either plugin into an `rclcpp_components` container and pass the same parameters produced by the bringup configuration loader. Torque manager requires `joint_names`, `arm_joints`, `base_joints`, `torque_controller_topic`, and `startup_policy=preserve`. Its default mutually exclusive callback group serializes command state access even in a multithreaded container. Unloading and reloading forgets all requested torque state and emits no command.

## Configuration and boundaries

Edit `lekiwi_bringup/config/control/controllers.yaml`. The launch loader supplies wheel-center radius, base frame, and joint ordering from controller configuration to the relevant nodes. Nodes do not query controller parameters at runtime. Configuration is immutable for each node lifetime.

- `workspace_kinematics.hpp`: URDF chain model and closed-form analytical IK solver (`SO101AnalyticalSolver`).
- `workspace_planner.hpp`: Pure domain service for mobile standoff candidate search and multi-tier move planning (`WorkspacePlanner`).
- `workspace_checker_node.hpp` and `workspace_checker_node.cpp`: ROS parameters, a consistent TF snapshot, service conversion, and diagnostics; installed as `workspace_checker_node` and the `lekiwi_motion::WorkspaceCheckerNode` component.
- `system_readiness_state.hpp` and `system_readiness_node.hpp`: Pure dual readiness evaluation (`system_readiness_state.hpp`) for decoupled navigation and manipulation gating, periodic diagnostic updates, and automatic Nav2 lifecycle startup (`system_readiness_node.hpp`).
- `torque_manager_node.hpp` & `torque_manager_node.cpp`: pure joint group tracking with precomputed indexing, read-only configuration, torque service, controller lifecycle management, and reliable command publisher.

The analytical arm supports a vertical pan, three parallel horizontal pitch axes, and a wrist-roll axis aligned with the configured TCP approach axis. The extractor retains all fixed transforms, shoulder/lateral offsets, joint axes, and URDF angle conventions. Unsupported chains and invalid limits are rejected; no nominal arm model or tool length is substituted. Small CAD axis rounding is bounded by `kinematics.axis_tolerance`, and every successful IK result must pass full-chain position and orientation residual checks. This remains an endpoint feasibility checker, not a collision or trajectory checker.

`robot_description` supplied as a parameter is authoritative. Otherwise the node subscribes to the relative `robot_description` topic with reliable, transient-local QoS. Until a valid model arrives, requests return `feasible=false`. An invalid topic update invalidates the active model; a subsequent valid update restores readiness. Diagnostics report the model source, status, base frame and TCP frame. Model and service callbacks share the default mutually exclusive callback group.

The configured chain runs from `base_frame` to `kinematics.tip_frame` (`gripperframe`). The five `kinematics.joint_names` must match chain order; the moving-jaw joint is not part of this chain. `kinematics.approach_axis` and `kinematics.up_axis` are orthogonal unit vectors expressed in the TCP frame. The deployed frame uses +Y for approach and +Z for zero-roll up. Positive pitch raises the approach axis, negative pitch points it down, and roll rotates about approach. Yaw follows each analytical pan solution. The legacy service value `required_pitch_angle=0` still selects `planning.default_pitch`; the pure planner accepts an explicit zero pitch.

Workspace/model/frame/TF parameters are read-only for the node lifetime. Nonempty paired finite tag arrays determine board bounds and take precedence over `board.w/h`; set both arrays empty to use explicit dimensions. Invalid startup configuration is rejected before the feasibility service is created. `planning.max_samples` (1–1001, default 257) bounds candidate evaluations across all tiers: zero-navigation first, approximately half the remaining budget for a common base, and the rest for separate endpoints. Each candidate requires at most two IK calls. The sampler explores both edge directions and all four edges; search exhaustion means no candidate was found, not a proof of geometric impossibility.

Wheel-center radius is not the footprint radius: footprint padding and planning clearance are separate. Board height and padding must be checked against the robot. Capture planning checks the pick target only, not an onboard drop trajectory.

## Interfaces

| Node | Interface | Behavior |
|---|---|---|
| workspace_checker | `/workspace/check_move_feasibility` | Returns IK hints and map-frame base poses; rejects missing, stale, nonplanar TF and invalid input |
| system_readiness_node | `/system/nav_ready`, `/system/grasp_ready` | Dual readiness heartbeats (Transient Local, Reliable); automatically autostarts Nav2 when nav readiness is attained |
| torque_manager | `/set_torque_enabled` | Submits name-mapped torque commands; see startup policy below |

System readiness publishes decoupled health signals: `/system/nav_ready` requires fresh odometry and base TF (and remains active during robot driving), whereas `/system/grasp_ready` requires standstill and fresh chessboard perception transforms prior to arm manipulation.

Torque startup policy is `preserve`: startup and shutdown emit no torque command. State is unknown until an explicit `ALL` request initializes the complete command vector; partial requests are rejected beforehand. A successful response acknowledges command submission, not hardware application. The hardware driver's own startup behavior is unchanged.

## Validation

```bash
colcon build --packages-select lekiwi_motion --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select lekiwi_motion --event-handlers console_cohesion+
colcon test-result --test-result-base build/lekiwi_motion --verbose
```

GTests cover:
- **ChessboardMapper**: UCI move notation parsing, file/rank index mapping, and coordinate symmetry.
- **FeasibilityMarkerBuilder**: 3D RViz visualization markers for single/dual/triple-base plans and HUD text alerts.
- **SystemReadinessEvaluator**: Two-tier readiness evaluation (Navigation Readiness vs Grasp Readiness), EKF variance checks, and stationarity detection.
- **TorqueCommandState**: Multi-group joint slicing, torque state toggles, and parameter validation.
- **WorkspaceKinematics**: Analytical IK roundtrip against URDF link lengths, multi-tier base standoff planning (Zero-Nav, Single-Base, Dual-Base, Triple-Base capture).

These unit tests establish L1 deterministic software behavior without requiring live hardware or physical collision simulators.
