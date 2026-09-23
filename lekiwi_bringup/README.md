# lekiwi_bringup

Robot launch files and deployment configuration for LeKiwi.

## Orchestration & Readiness Subsystem

The readiness and orchestration subsystem (`tf_gatekeeper_node`, `workspace_checker`, `task_orchestrator`, and optional `chess_mission_orchestrator`) is consolidated in `lekiwi_orchestrator/launch/orchestrator.launch.py`. Parameters are configured centrally in `lekiwi_orchestrator/config/orchestrator_params.yaml`.

```bash
ros2 launch lekiwi_orchestrator orchestrator.launch.py
ros2 launch lekiwi_orchestrator orchestrator.launch.py start_mission:=false use_sim_time:=true
```

`robot.launch.py` automatically includes `orchestrator.launch.py` with `start_mission:=false`, providing the full readiness gatekeeper, workspace feasibility service, and lifecycle management. Torque manager is managed via `controllers.launch.py`.

Use the installed launch descriptions to inspect the complete top-level interface:

```bash
ros2 launch lekiwi_bringup robot.launch.py --show-args
```

The top-level default hardware type is `real`; explicitly choose deployment arguments for your environment.

## Configuration

`config/control/controllers.yaml` contains ros2_control controller parameters and the torque manager configuration. `config/control/orchestrator.yaml` contains the workspace checker, TF gatekeeper, and task orchestrator sections. The bringup launch files pass these configurations directly before starting nodes. No runtime parameter discovery is required.

Geometry, thresholds, timeouts, and topic configuration belong in YAML. Wheel-center radius, footprint padding, and planning clearance have distinct meanings. See [control documentation](../lekiwi_control/README.md) for model and torque startup contracts.

Other configuration folders hold localization, navigation, perception, sensors, and diagnostics. Calibration values remain deployment-specific.

## Tests

`test/test_control_launch.py` exercises the installed launch with isolated fake ROS sensors and services, including the combined switch, EKF bootstrap, stale odometry, workspace responses, and lifecycle reconfiguration.

```bash
colcon test --packages-select lekiwi_bringup
colcon test-result --verbose
```
