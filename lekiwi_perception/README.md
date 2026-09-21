# `lekiwi_perception`

High-performance C++ ROS 2 package managing camera acquisition via GStreamer, zero-copy valve gating, Hailo-8/8L NPU hardware-accelerated deep learning inference, AprilTag chessboard localization, and perception visualization.

---

## 🏗️ Components

This package registers four C++ plugins using `rclcpp_components`:

### 1. `CameraStreamerComponent`
Lifecycle-managed camera driver using GStreamer 1.0.
- Supports CSI cameras via `libcamerasrc` (Raspberry Pi 5 RP1 CSI) and USB cameras via `v4l2src`.
- Features dynamic GStreamer `valve` gating based on the `/camera_mode` topic: when the camera is not in the configured `active_modes`, frames are dropped immediately at the source with 0 CPU overhead.
- Publishes ROS 2 `sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`, and `sensor_msgs/msg/CameraInfo`.

### 2. `HailoChessInferenceComponent`
Lifecycle-managed neural network inference engine running on the Hailo-8 / Hailo-8L M.2 NPU HAT.
- Executes compiled Hailo Executable Format (`.hef`) models (`yolo11n.hef`, `yolov8n-seg.hef`) for piece and board detection.
- Uses `chess_vision_mapper` to compute homography and perspective transformation matrices, projecting detected 2D piece bounding boxes onto an $8 \times 8$ chessboard matrix.
- Generates Forsyth-Edwards Notation (FEN) strings published to `/chess/fen` and piece detections to `/chess/detections_2d`.

### 3. `ChessOverlayComponent`
Headless perception overlay renderer drawing bounding boxes, AprilTag corners, and 81 grid points onto camera frames.
- Subscribes to raw camera image, piece detections, tag centers, and grid points.
- Implements lazy evaluation (skips drawing, cloning, and JPEG encoding when no subscribers are present).
- Implements TTL staleness gating (drops stale detections/grid markers to prevent visual ghosting).
- Publishes compressed annotated stream to `/chess/overlay_image/compressed`.

### 4. `ChessboardPoseEstimator`
Lifecycle-managed component estimating camera/robot pose with respect to the chessboard using corner AprilTags.
- Solves PnP / orthogonal Procrustes from 4 corner tags (IDs 0, 1, 2, 3: A1, H1, H8, A8).
- Publishes tag detections, normalized corner centers, and robot pose.
- Optionally broadcasts TF transforms between chessboard and camera frames.

---

## 📦 Package Structure

```text
lekiwi_perception/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── calibration/                 # Calibration streaming configs for ROS camera calibration tool
│       ├── stereo_left_conf.yaml
│       ├── stereo_right_conf.yaml
│       ├── usb_side_conf.yaml
│       └── usb_wrist_conf.yaml
├── include/
├── resources/                       # Pre-compiled HEF neural network models
│   └── models/
├── src/
│   ├── apriltag/
│   └── hailo/
└── test/
```

## 📡 Topics & Services

### Published Topics
| Topic | Type | Description |
|---|---|---|
| `<namespace>/image_raw` | `sensor_msgs/msg/Image` | Raw or color-converted camera video frame. |
| `<namespace>/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Compressed camera stream. |
| `<namespace>/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsic calibration parameters. |
| `/chess/fen` | `std_msgs/msg/String` | Real-time chess board state in FEN format. |
| `/chess/detections_2d` | `vision_msgs/msg/Detection2DArray` | 2D bounding boxes and class IDs for detected pieces. |
| `/chess/tag_centers` | `geometry_msgs/msg/PolygonStamped` | AprilTag corner positions normalized in image coordinates. |
| `/chess/grid_points` | `geometry_msgs/msg/PolygonStamped` | 81 projected chessboard grid intersections. |
| `/chess/overlay_image/compressed` | `sensor_msgs/msg/CompressedImage` | Low-latency annotated visual stream (bounding boxes, tags, grid). |

### Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `/camera_mode` | `lekiwi_interfaces/msg/CameraMode` | Latched system camera mode used for dynamic valve gating. |
| `/cameras/stereo_left/image_raw` | `sensor_msgs/msg/Image` | Input frame for chess piece and board inference. |
| `/chess/detections_2d` | `vision_msgs/msg/Detection2DArray` | Input piece bounding boxes for overlay. |
| `/chess/tag_centers` | `geometry_msgs/msg/PolygonStamped` | Tag corner points for overlay. |
| `/chess/grid_points` | `geometry_msgs/msg/PolygonStamped` | Grid intersection points for overlay. |

### Services
| Service | Type | Description |
|---|---|---|
| `/hailo_chess_inference/set_camera_mode` | `lekiwi_interfaces/srv/SetCamMode` | Directly set mode for the inference component. |

---

## ⚙️ Key Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_topic` | `string` | `"/cameras/stereo_left/image_raw"` | Input image topic for overlay rendering. |
| `overlay_topic` | `string` | `"/chess/overlay_image/compressed"` | Output compressed overlay topic. |
| `jpeg_quality` | `int` | `80` | JPEG compression quality (1-100). |
| `stale_timeout_sec` | `double` | `0.5` | TTL timeout in seconds to discard stale detection/grid markers. |
| `debug` | `bool` | `false` | Enable FPS overlay badge. |

---

## 🧪 Testing

Run C++ GoogleTest unit and integration tests:

```bash
colcon test --packages-select lekiwi_perception --event-handlers console_direct+
colcon test-result --verbose
```
