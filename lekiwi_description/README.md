# `lekiwi_description`

Robot description and visualization package for the LeKiwi robot, containing URDF / Xacro kinematic models, CAD STL meshes, calibration configurations, RViz/RQt configurations, and `robot_state_publisher` / RViz2 launch files.

---

## 📂 File Layout

```text
lekiwi_description/
├── assets/                          # 3D STL meshes and part definitions for arm and base
├── config/                          # Central configuration directory
│   ├── calibration/                 # Calibration data
│   │   ├── DuyKhongCay.json         # Raw servo configuration
│   │   ├── handeye_calib_stereo_left.yaml # Calibrated stereo left camera extrinsics
│   │   └── lekiwi_arm_calib.yaml    # Arm joint offset & range calibration
│   ├── rqt/                         # RQt perspectives
│   │   └── lekiwi_debug.perspective # Unified inspection dashboard perspective
│   └── rviz/                        # RViz2 visualization profiles
│       ├── lekiwi_full.rviz         # Full robot visualization (TF, model, sensors, cameras)
│       ├── lekiwi_model_only.rviz   # Model-only inspection view
│       └── lekiwi_perception.rviz   # Camera feeds and perception markers
├── launch/
│   ├── description.launch.py        # Generates xacro and publishes robot_state_publisher
│   └── visualizer.launch.py         # Includes description.launch.py + RViz2 + JSP GUI
├── urdf/
│   ├── duykhongcay_lekiwi.urdf      # Raw CAD kinematic links, joints, and visual/collision geometries
│   ├── ros2_control.xacro           # ros2_control system & sensor interface macros
│   ├── sensors_calibration.xacro    # Calibrated camera extrinsics macro (reads YAML file)
│   └── lekiwi_robot.urdf.xacro      # Top-level composition Xacro entrypoint
├── CMakeLists.txt
└── package.xml
```

---

## 🤖 Robot Kinematic & Joint Model

The robot consists of **9 actuated joints**:

### 1. 6-DoF Robotic Arm Joints (`position` command interface)
- `arm_shoulder_pan` (Servo ID: 1)
- `arm_shoulder_lift` (Servo ID: 2)
- `arm_elbow_flex` (Servo ID: 3)
- `arm_wrist_flex` (Servo ID: 4)
- `arm_wrist_roll` (Servo ID: 5)
- `arm_gripper` (Servo ID: 6)

### 2. 3-Wheel Omnidirectional Base (`velocity` command interface)
- `base_left_wheel` (Servo ID: 7)
- `base_back_wheel` (Servo ID: 8)
- `base_right_wheel` (Servo ID: 9)

---

##  Usage

### 1. Offline URDF & Joint Testing (No Robot Required)
Inspect the URDF model and test joint movements with `joint_state_publisher_gui`:
```bash
ros2 launch lekiwi_description visualizer.launch.py gui:=true rviz_config:=$(ros2 pkg prefix lekiwi_description)/share/lekiwi_description/config/rviz/lekiwi_model_only.rviz
```

### 2. Live RViz Visualization (Remote or Local Robot)
Visualize live robot state, TF transforms, odometry, IMU, and sensors:
```bash
# Full visualization (default: lekiwi_full.rviz)
ros2 launch lekiwi_description visualizer.launch.py

# Perception & camera stream visualization
ros2 launch lekiwi_description visualizer.launch.py rviz_config:=$(ros2 pkg prefix lekiwi_description)/share/lekiwi_description/config/rviz/lekiwi_perception.rviz
```

### 3. Publish Robot Description & State Only
```bash
ros2 launch lekiwi_description description.launch.py hardware_type:=real
```

### 4. Diagnostics & Debugging via RQt
```bash
# Unified perspective
rqt --perspective-file $(ros2 pkg prefix lekiwi_description)/share/lekiwi_description/config/rqt/lekiwi_debug.perspective
```

