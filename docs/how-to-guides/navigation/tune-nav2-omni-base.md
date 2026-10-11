# How-To: Tune Nav2 for 3-Wheel Omni-Base

This guide explains how to configure and tune the ROS 2 Nav2 stack for LeKiwi's 3-wheel omnidirectional mobile base using the MPPI controller.

---

## 1. Kinematics & Velocity Limits (MPPI Controller)

LeKiwi uses an equilateral 3-wheel omnidirectional chassis capable of holonomic motion ($v_x, v_y, \omega_z$). Kinematics in `lekiwi_bringup/config/navigation/nav2_params.yaml` are governed by `nav2_mppi_controller::MPPIController`:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      motion_model: "Omni"             # Holonomic omnidirectional kinematics
      time_steps: 20                   # 20 projection steps optimized for Pi 5
      model_dt: 0.10                   # 100ms integration time step
      batch_size: 500                  # 500 Monte Carlo trajectories
      iteration_count: 2
      temperature: 0.3
      gamma: 0.015

      # --- Kinematic Boundaries ---
      vx_max: 0.80                     # Maximum forward speed (m/s)
      vx_min: -0.80
      vy_max: 0.80                     # Maximum strafe speed (m/s)
      vy_min: -0.80
      wz_max: 1.00                     # Maximum angular speed (rad/s)
      wz_min: -1.00
      ax_max: 1.2
      ay_max: 1.2
      az_max: 2.0
```

---

## 2. Costmap Footprint & Clearance

Set the circular collision boundary matching the physical chassis geometry:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.132520           # Calibrated chassis radius in meters
      inflation_layer:
        inflation_radius: 0.30
        cost_scaling_factor: 8.0
```

---

## 3. Launch Localization and Navigation

Bring up the localization and Nav2 stack:

```bash
source ~/lekiwi_ws/install/setup.bash
ros2 launch lekiwi_bringup localization.launch.py
ros2 launch lekiwi_bringup navigation.launch.py
```

Send a 2D navigation goal using RViz2 or the Nav2 Action client:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.2, y: 0.5, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```
