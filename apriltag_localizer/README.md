# `apriltag_localizer`

C++ ROS 2 package providing high-precision visual localization, coordinate frame estimation, and diagnostics for the LeKiwi robot using OpenCV ArUco (`cv::aruco::ArucoDetector`) and `solvePnP`.

---

## 🎯 Purpose & Key Features

- **Direct In-Process AprilTag/ArUco Detection**:
  - Direct end-to-end processing of `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo` without external detection node middleware overhead.
  - State gating: runs detection only when in `CameraMode::CHESS_THINKING` and before anchor lock (`is_anchored == false`), reducing idle CPU usage to zero.
  - Rate limiting support via `detection_rate_hz` (default: 2.0 Hz).
- **Chessboard Arena & World Frame Estimation**:
  - Detects AprilTags (e.g. 16h5 / 36h11) positioned at known geometric locations around the chessboard.
  - Computes the 6-DoF transformation between camera and the chessboard frame using OpenCV Perspective-n-Point (`solvePnP`).
- **TF2 Broadcasting & Anchor Lock**:
  - Broadcasts static transforms (`map` -> `chessboard_frame`) and dynamic transforms (`map` -> `odom` when anchor is locked).
  - Provides `/chessboard/lock_anchor` and `/chessboard/reset_anchor` services.
- **Diagnostics & Health Monitoring**:
  - Integrates `diagnostic_updater` to publish real-time Processing FPS, End-to-End Latency, Algorithm Processing Time, and Detected Tag IDs to `/diagnostics`.
- **Mathematical Accuracy & Geometric Sanity**:
  - Tested 3D-to-2D corner projection, rotation matrix math, and yaw offset transformations in dedicated unit tests (`test_pose_solver`).

---

## 📦 Package Structure

```text
apriltag_localizer/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/apriltag_localizer/
│   └── chessboard_pose_estimator.hpp # Header declaring pose estimation pipeline and TF logic
├── src/
│   └── chessboard_pose_estimator.cpp # Direct image detection, SolvePnP calculation, and ROS node component
└── test/
    └── test_pose_solver.cpp          # GTest unit tests for SolvePnP math and corner projections
```

*(Note: System configuration file `chessboard_tags.yaml` is managed in `lekiwi_bringup/config/localization/`).*

---

## 📡 Topics & TF Frames

### Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `~/image_raw` | `sensor_msgs/msg/Image` | Camera video stream (e.g. `/cameras/stereo_left/image_raw`). |
| `~/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsic camera matrix and distortion coefficients. |
| `~/camera_mode` | `lekiwi_interfaces/msg/CameraMode` | Robot perception mode (gated on `CHESS_THINKING`). |

### Published Topics
| Topic | Type | Description |
|---|---|---|
| `/chessboard/robot_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Estimated 6-DoF robot pose in `map` frame with covariance. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Real-time performance, FPS, latency, and tag detection status. |

### Services
| Service | Type | Description |
|---|---|---|
| `/chessboard/lock_anchor` | `std_srvs/srv/Trigger` | Locks `map` -> `odom` TF and pauses continuous tag detection. |
| `/chessboard/reset_anchor` | `std_srvs/srv/Trigger` | Resets anchor lock and allows tag scanning again. |

### Published TF Frames
- Static TF: `map` -> `chessboard_frame`
- Dynamic TF (when anchored): `map` -> `odom`

---

## 🚀 Usage

### 1. Build and Run Tests
```bash
colcon build --symlink-install --packages-select apriltag_localizer --cmake-args -GNinja
colcon test --packages-select apriltag_localizer --event-handlers console_direct+
```

### 2. Launching in Bringup
The estimator component is composed directly into `lekiwi_perception_container` via `cameras.launch.py`:
```bash
ros2 launch lekiwi_bringup cameras.launch.py
```

