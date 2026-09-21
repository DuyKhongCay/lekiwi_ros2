# lekiwi_bringup

Robot launch files and deployment configuration for LeKiwi.

## Control migration

`control.launch.py` is the single entry point for `lekiwi_control`: torque manager always runs, while `enable_readiness_checks` enables both TF gatekeeper and workspace checker. It contains no orchestrator or manipulation nodes.

```bash
ros2 launch lekiwi_bringup control.launch.py
ros2 launch lekiwi_bringup control.launch.py enable_readiness_checks:=false use_sim_time:=true
```

These commands launch the control services and monitors only; they do not start ros2_control or hardware. Replace old launches from `lekiwi_control` and separate workspace/gatekeeper switches with this entry point and combined switch.

`robot.launch.py` forwards `enable_readiness_checks` and `use_sim_time`. It starts task orchestration separately. When navigation is enabled, task orchestration starts Nav2 after a fresh readiness heartbeat. Disabling readiness checks does not bypass that readiness requirement.

Use the installed launch descriptions to inspect the complete top-level interface:

```bash
ros2 launch lekiwi_bringup robot.launch.py --show-args
```

The top-level default hardware type is `real`; explicitly choose deployment arguments for your environment.

## Configuration

`config/control/lekiwi_controllers.yaml` contains controller parameters and the workspace checker, torque manager, and TF gatekeeper sections. The control launch derives shared base frame, wheel-center radius, arm/base joint lists, and torque command order directly from the controller sections before starting nodes. No runtime parameter discovery is required.

Geometry, thresholds, timeouts, and topic configuration belong in YAML. Wheel-center radius, footprint padding, and planning clearance have distinct meanings. See [control documentation](../lekiwi_control/README.md) for model and torque startup contracts.

Other configuration folders hold localization, navigation, perception, sensors, and diagnostics. Calibration values remain deployment-specific.

## Tests

`test/test_control_launch.py` exercises the installed launch with isolated fake ROS sensors and services, including the combined switch, EKF bootstrap, stale odometry, workspace responses, and lifecycle reconfiguration.

```bash
colcon test --packages-select lekiwi_bringup
colcon test-result --verbose
```
