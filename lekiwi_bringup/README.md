# lekiwi_bringup

Robot launch files and deployment configuration for LeKiwi.

## Orchestration & Readiness Subsystem

The readiness and orchestration subsystem (`system_readiness_node`, `workspace_checker`, and optional `chess_mission_orchestrator`) is consolidated in `lekiwi_bringup/launch/orchestrator.launch.py`. Parameters are configured centrally in `config/control/orchestrator.yaml`.

```bash
ros2 launch lekiwi_bringup orchestrator.launch.py
ros2 launch lekiwi_bringup orchestrator.launch.py start_mission:=true use_sim_time:=false
```

`robot.launch.py` automatically includes `orchestrator.launch.py` (guarded by `enable_orchestrator`), providing the system readiness supervisor and workspace feasibility service. Autonomous mission execution can be toggled via `start_mission:=true`.

Use the installed launch descriptions to inspect the complete top-level interface:

```bash
ros2 launch lekiwi_bringup robot.launch.py --show-args
```

The top-level default hardware type is `real`, and the default manipulation mode is `mock` (running `mock_policy_server` for safe bringup without accidental arm motion). Explicitly pass `manipulation:=kinematics` for full physical chess manipulation, or `manipulation:=policy` for LeRobot inference.

## Configuration

`config/control/controllers.yaml` contains ros2_control controller parameters and the torque manager configuration. `config/control/orchestrator.yaml` contains the workspace checker, system readiness supervisor, and chess mission orchestrator sections. The bringup launch files pass these configurations directly before starting nodes. No runtime parameter discovery is required.

Geometry, thresholds, timeouts, and topic configuration belong in YAML. Wheel-center radius, footprint padding, and planning clearance have distinct meanings. See [control documentation](../lekiwi_control/README.md) for model and torque startup contracts.

Other configuration folders hold localization, navigation, perception, sensors, and diagnostics. Calibration values remain deployment-specific.

## Tests

`test/test_control_launch.py` exercises the installed launch with isolated fake ROS sensors and services, including the combined switch, EKF bootstrap, stale odometry, workspace responses, and lifecycle reconfiguration.

```bash
colcon test --packages-select lekiwi_bringup
colcon test-result --verbose
```
