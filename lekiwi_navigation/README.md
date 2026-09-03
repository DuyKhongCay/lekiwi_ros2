# `lekiwi_navigation`

Autonomous navigation and costmap package for the LeKiwi 3-wheel omnidirectional mobile base operating around a chessboard arena.

---

## 🗺️ Architecture & Design Rationale

This package implements the ROS 2 Navigation 2 (Nav2) stack tailored for a sensorless obstacle avoidance setup:
- **Kinematics**: 3-wheel omnidirectional holonomic base ($v_x, v_y, \omega_z$).
- **Static Map**: 2.0m x 2.0m workspace arena with a central 0.45m x 0.45m forbidden zone (380mm chessboard + safety margin). Loaded via `nav2_map_server`.
- **Global Planner**: `nav2_smac_planner::SmacPlanner2D` (A* on 2D grid with path smoothing).
- **Local Controller**: `dwb_core::DWBLocalPlanner` supporting full holonomic velocities ($v_x, v_y, \omega_z$), allowing strafing and smooth trajectory tracking around the chessboard.
- **Sensorless Costmaps**: Global and Local costmaps run strictly on `StaticLayer` + `InflationLayer` without any LiDAR/Depth sensor layers (no `/scan` timeout/warning).
- **Command Arbitration & E-Stop**: `twist_mux` is integrated and **always active** inside `navigation.launch.py`:
  - Priority 10: `/cmd_vel_nav` (Nav2 autonomous path following).
  - Priority 100: `/cmd_vel_teleop` (Gamepad joystick manual override).
  - Priority 255: `/safety/estop_active` (Emergency stop lock, sets cmd_vel to 0).
- **Localization**: Relies on `lekiwi_tag_localization` (AprilTag pose estimation) and IMU (`/imu/data`) fusing into TF `map -> odom -> base_footprint`.

---

## 📁 Package Structure

```text
lekiwi_navigation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── nav2_params.yaml          # SmacPlanner2D, DWB, Static Costmaps, BT Navigator
│   ├── twist_mux.yaml            # Priority arbitration & E-Stop lock
│   └── nav2_costmap_keepout.yaml # Costmap Keepout filter configuration
├── launch/
│   └── navigation.launch.py      # Unified Nav2 + always-on twist_mux bringup
└── maps/
    ├── chessboard_arena.pgm      # 100x100 grayscale occupancy grid (2.0m x 2.0m)
    ├── chessboard_arena.png      # Converted PNG for visual inspection
    └── chessboard_arena.yaml     # Map metadata (resolution: 0.02m, origin: [-1.0, -1.0, 0.0])
```

---

## 🚀 Running Navigation

### 1. Navigation Launch (Nav2 + Always-On twist_mux):
```bash
ros2 launch lekiwi_navigation navigation.launch.py use_sim_time:=false
```

### 2. Full Robot Bringup with Navigation Enabled:
```bash
ros2 launch lekiwi_bringup robot.launch.py enable_navigation:=true
```

### 3. Verification & Testing:
- Send a 2D Goal Pose via RViz2 or `nav2_simple_commander`.
- To trigger E-Stop:
  ```bash
  ros2 topic pub /safety/estop_active std_msgs/msg/Bool "{data: true}"
  ```
  All `/cmd_vel` output immediately drops to zero.
