# `lekiwi_tag_localization`

C++ ROS 2 package providing high-precision visual localization and coordinate frame estimation for the LeKiwi robot using AprilTag 36h11 fiducial marker boards and OpenCV `solvePnP`.

---

## 🎯 Purpose & Key Features

- **Chessboard Arena & World Frame Estimation**:
  - Detects AprilTags positioned at known geometric locations around the arena/chessboard.
  - Computes the 6-DoF transformation between the camera and the world/arena reference frame using OpenCV Perspective-n-Point (`solvePnP` / `solvePnPRansac`).
- **TF2 Broadcasting**:
  - Broadcasts the dynamic transform from world / arena frame to the robot base frame (`map` -> `odom` or `chessboard` -> `stereo_left_optical_frame`).
- **Mathematical Accuracy & Geometric Sanity**:
  - Fully tested 3D-to-2D corner projection, rotation matrix math, and yaw offset transformations in dedicated unit tests.

---

## 📦 Package Structure

```text
lekiwi_tag_localization/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/lekiwi_tag_localization/
│   └── chessboard_pose_estimator.hpp # Header declaring pose estimation pipeline and TF logic
├── src/
│   ├── chessboard_pose_estimator.cpp      # SolvePnP pose calculation and corner mapping
│   └── chessboard_pose_estimator_node.cpp # ROS 2 node wrapper and TF broadcaster
└── test/
    └── test_pose_solver.cpp         # GTest unit tests for SolvePnP math and corner projections
```

*(Note: System configuration files `apriltag_36h11.yaml` and `chessboard_tags.yaml` are managed in `lekiwi_bringup/config/localization/`).*

---

## 📡 Topics & TF Frames

### Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `/cameras/stereo_left/image_raw` | `sensor_msgs/msg/Image` | Left stereo camera video stream. |
| `/cameras/stereo_left/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsic camera matrix and distortion coefficients. |
| `/apriltag_detections` | `apriltag_msgs/msg/AprilTagDetectionArray` | Detected AprilTag corner coordinates and IDs. |

### Published TF Frames
- `map` / `chessboard` -> `base_footprint` / `stereo_left_optical_frame`

---

## 🚀 Usage

### 1. Run Tag Localization Node
```bash
ros2 run lekiwi_tag_localization chessboard_pose_estimator_node --ros-args --params-file src/lekiwi_ros2/lekiwi_bringup/config/localization/chessboard_tags.yaml
```

### 2. Calibrate Tag Coordinates (Offline / Live)
```bash
# Option A: Calibrate from Rosbag
ros2 run lekiwi_tag_localization calibrate_chessboard_tags.py \
  --bag path/to/bag.mcap \
  --camera-info /root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/camera_info/stereo_left.yaml \
  --z-prior 0.0 0.0 0.0 0.0 \
  -o /root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/config/chessboard_tags.yaml

# Option B: Calibrate Live from Active Camera Stream
ros2 run lekiwi_tag_localization calibrate_chessboard_tags.py \
  --live --num-frames 60 \
  --topic /apriltag_detections \
  --camera-info /root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/camera_info/stereo_left.yaml \
  -o /root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/config/chessboard_tags.yaml
```

### 3. Run Unit Tests
```bash
colcon test --packages-select lekiwi_tag_localization --event-handlers console_direct+
colcon test-result --verbose
```
