# LeKiwi bringup

`lekiwi_bringup` owns the runtime launch topology and configuration. The
model-only `lekiwi_description` package contains URDF/Xacro and CAD assets.

```text
robot.launch.py
├── description.launch.py  robot_state_publisher + optional ros2_control
├── cameras.launch.py      MultiCameraHub component container
└── control.launch.py      task orchestrator + optional LeRobot bridge/demo
```

Use the narrow launch files when debugging one subsystem:

```bash
ros2 launch lekiwi_bringup description.launch.py \
  hardware_type:=mock start_controller_manager:=true activate_controllers:=true
ros2 launch lekiwi_bringup cameras.launch.py use_test_sources:=true
ros2 launch lekiwi_bringup control.launch.py start_lerobot_bridge:=true
```

The top-level entrypoint remains:

```bash
ros2 launch lekiwi_bringup robot.launch.py
```

`hardware_type:=mock` is the default and does not open the serial bus. Before
using `hardware_type:=real`, isolate the robot and validate the wheel direction
and velocity scale in `config/hardware/lekiwi_joints.yaml`.
