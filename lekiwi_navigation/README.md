# `lekiwi_navigation`

Autonomous navigation package for the LeKiwi omnidirectional mobile base (Roadmap / In Development).

---

## 🗺️ Planned Architecture & Scope

This package will integrate the ROS 2 Navigation 2 (Nav2) stack for LeKiwi's 3-wheel omnidirectional base:

- **Localization**: AMCL / `robot_localization` EKF fusing wheel odometry (`/omni_base_controller/odom`), IMU filtered orientation (`/imu/data`), and 2D LiDAR / visual odometry.
- **Global & Local Planners**: Omnidirectional trajectory generation supporting holonomic velocity commands ($v_x, v_y, \omega_z$).
- **Costmaps**: 2D and voxel obstacle clearing using stereo depth and ultrasonic/LiDAR range sensors.
- **Behavior Trees**: Nav2 BT nodes coordinating waypoint following and docking.

---

## 📁 Target Structure

```text
lekiwi_navigation/
├── config/                  # Nav2 params, costmap configs, behavior tree XMLs
├── launch/                  # navigation.launch.py, localization.launch.py
├── maps/                    # Occupancy grid maps (.yaml, .pgm)
├── CMakeLists.txt
└── package.xml
```

