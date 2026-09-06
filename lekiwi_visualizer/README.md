# lekiwi_visualizer

Host PC visualization, inspection, and diagnostic suite for the LeKiwi mobile manipulator robot.

## Features

- **Decoupled Architecture**: Runs entirely on host PC / development workstation without any dependencies on robot-side hardware drivers or controller packages.
- **RViz2 Configurations**:
  - `lekiwi_full.rviz`: Complete visualization setup including Robot Model (URDF), TF frame tree, Odometry, IMU vector/orientation (`rviz_imu_plugin`), AprilTag markers, and camera preview panels.
  - `lekiwi_model_only.rviz`: Lightweight model-only view for fast preview.
  - `lekiwi_perception.rviz`: Camera feeds (`stereo_left`, `usb_wrist`, `usb_side`) and AprilTag markers.
- **Diagnostic Tools**:
  - `rqt_robot_monitor` for inspecting aggregated system health (`/diagnostics_agg`).
  - `rqt_runtime_monitor` for inspecting raw diagnostics (`/diagnostics`).
- **RQt Debugging & Inspection Suite**:
  - `rqt_graph`: Node and topic connectivity graph.
  - `rqt_topic`: Topic inspector with live message echo, publishing frequency (Hz), and bandwidth.
  - `rqt_service_caller`: Interactive ROS 2 service call interface.
  - `rqt_action`: Action server goal submission and feedback viewer.
  - `rqt_console`: Centralized `/rosout` system log viewing and severity filtering.
  - `rqt_tf_tree`: Dynamic TF2 transform tree visualizer.
  - `rqt_reconfigure`: Real-time dynamic parameter configuration.
  - `rqt_plot`: 2D numeric realtime topic plotter.
  - `rqt_publisher`: Manual topic message publisher.
- **Offline URDF Inspector**:
  - `joint_state_publisher_gui` for interactive joint manipulation and CAD verification.

---

## Usage

### 1. Offline URDF & Joint Testing
Inspect the LeKiwi robot model and test joint movements without connecting to a physical robot:
```bash
ros2 launch lekiwi_visualizer view_model.launch.py
```

### 2. Live RViz Visualization (Remote or Local)
Visualize robot state, TF transforms, IMU data, and sensors:
```bash
ros2 launch lekiwi_visualizer visualizer.launch.py
```
To run perception view:
```bash
ros2 launch lekiwi_visualizer visualizer.launch.py rviz_config:=$(ros2 pkg prefix lekiwi_visualizer)/share/lekiwi_visualizer/config/rviz/lekiwi_perception.rviz
```

### 3. Diagnostic Health Monitor
Monitor robot temperature, servo status, CPU/RAM usage, and perception components:
```bash
ros2 launch lekiwi_visualizer diagnostics.launch.py
```

### 4. RQt Debug & Inspection Suite
Launch individual tools or the whole inspection suite:
```bash
# View ROS system logs (/rosout)
ros2 launch lekiwi_visualizer rqt_inspect.launch.py console:=true

# View node connectivity graph and topic monitor
ros2 launch lekiwi_visualizer rqt_inspect.launch.py graph:=true topic:=true

# View TF transform hierarchy
ros2 launch lekiwi_visualizer rqt_inspect.launch.py tf_tree:=true

# Call services & actions interactively
ros2 launch lekiwi_visualizer rqt_inspect.launch.py service:=true action:=true

# Launch unified RQt dashboard perspective
ros2 launch lekiwi_visualizer rqt_inspect.launch.py use_perspective:=true
```

### 5. Full Dashboard
Launch RViz2, `rqt_robot_monitor`, and optionally `rqt_console` or the full perspective simultaneously:
```bash
ros2 launch lekiwi_visualizer full_dashboard.launch.py console:=true
```

---

## Connecting via Zenoh RMW

When running across a local network with Zenoh RMW (`rmw_zenoh_cpp`), configure your host environment prior to launching:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=/path/to/zenoh_pc.json
ros2 launch lekiwi_visualizer visualizer.launch.py
```
