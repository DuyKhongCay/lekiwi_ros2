# `lekiwi_description`

Robot description package for the LeKiwi robot, containing URDF / Xacro kinematic models, CAD STL meshes, joint transmission macros, and `ros2_control` hardware interface wiring.

---

## 📂 File Layout

```text
lekiwi_description/
├── assets/                  # 3D STL meshes and part definitions for arm and base
├── launch/                  # (Provided via lekiwi_bringup/launch/description.launch.py)
├── urdf/
│   ├── duykhongcay_lekiwi.urdf    # Base kinematic links, joints, and visual/collision geometries
│   ├── ros2_control.xacro         # ros2_control system & sensor interface macros
│   └── lekiwi_robot.urdf.xacro    # Top-level composition Xacro entrypoint
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

## 🚀 Usage

Check the URDF tree:

```bash
xacro $(ros2 pkg prefix --share lekiwi_description)/urdf/lekiwi_robot.urdf.xacro hardware_type:=mock > /tmp/lekiwi.urdf
check_urdf /tmp/lekiwi.urdf
```

Launch robot visualization and `robot_state_publisher` in RViz:

```bash
ros2 launch lekiwi_bringup description.launch.py hardware_type:=mock start_controller_manager:=true activate_controllers:=true
```

