# `lekiwi_perception`

High-performance C++ ROS 2 package managing camera acquisition via GStreamer, zero-copy valve gating, Hailo-8/8L NPU hardware-accelerated deep learning inference, chessboard calibration, and FEN piece mapping.

---

## 🏗️ Components

This package registers three C++ plugins using `rclcpp_components`:

### 1. `CameraStreamerComponent`
Lifecycle-managed camera driver using GStreamer 1.0.
- Supports CSI cameras via `libcamerasrc` (Raspberry Pi 5 RP1 CSI) and USB cameras via `v4l2src`.
- Features dynamic GStreamer `valve` gating based on the `/camera_mode` topic: when the camera is not in the configured `active_modes`, frames are dropped immediately at the source with 0 CPU overhead.
- Publishes ROS 2 `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo`.

### 2. `HailoChessInferenceComponent`
Lifecycle-managed neural network inference engine running on the Hailo-8 / Hailo-8L M.2 NPU HAT.
- Executes compiled Hailo Executable Format (`.hef`) models (`yolo11n.hef`, `yolov8n-seg.hef`) for piece and board detection.
- Uses `chess_vision_mapper` to compute homography and perspective transformation matrices, projecting detected 2D piece bounding boxes onto an $8 \times 8$ chessboard matrix.
- Generates Forsyth-Edwards Notation (FEN) strings published to `/hailo_chess_inference/fen`.

### 3. `ChessVisualizerComponent`
Visual debugging component combining the debug camera image with digital 2D top-down board rendering and transparent piece sprite overlays.
- Emits composite visualization images to `/chess_visualizer/visual_image`.
- Optional standalone OpenCV Qt GUI window.

---

## 📡 Topics & Services

### Published Topics
| Topic | Type | Description |
|---|---|---|
| `<namespace>/image_raw` | `sensor_msgs/msg/Image` | Raw or color-converted camera video frame. |
| `<namespace>/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsic calibration parameters. |
| `/hailo_chess_inference/fen` | `std_msgs/msg/String` | Real-time chess board state in FEN format. |
| `/hailo_chess_inference/detections` | `vision_msgs/msg/Detection2DArray` | 2D bounding boxes and class IDs for detected pieces. |
| `/hailo_chess_inference/debug_image` | `sensor_msgs/msg/Image` | Debug frame with drawn bounding boxes and grid overlay. |
| `/hailo_chess_inference/status` | `lekiwi_interfaces/msg/HailoInferenceStatus` | Pipeline health, FPS, and error state. |
| `/chess_visualizer/visual_image` | `sensor_msgs/msg/Image` | Side-by-side composite visual debugging panel. |

### Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `/camera_mode` | `lekiwi_interfaces/msg/CameraMode` | Latched system camera mode used for dynamic valve gating. |
| `/cameras/stereo_left/image_raw` | `sensor_msgs/msg/Image` | Input frame for chess piece and board inference. |

### Services
| Service | Type | Description |
|---|---|---|
| `/hailo_chess_inference/set_camera_mode` | `lekiwi_interfaces/srv/SetCamMode` | Directly set mode for the inference component. |

---

## ⚙️ Key Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gscam_config` | `string` | `""` | GStreamer pipeline string (must contain `valve name=gate`). |
| `camera_name` | `string` | `"camera"` | Frame and calibration identifier. |
| `frame_id` | `string` | `"camera"` | TF2 coordinate frame name. |
| `camera_info_url` | `string` | `""` | Path to camera calibration YAML file. |
| `active_modes` | `int[]` | `[]` | List of mode integers where this camera stream is enabled. |
| `publish_debug_image` | `bool` | `true` | Publish annotated debug frames. |
| `vdevice_group_id` | `string` | `"lekiwi_chess"` | HailoRT virtual device group identifier. |
| `model_width` / `model_height` | `int` | `640` | Input resolution for the Hailo HEF neural network model. |

---

## 🧪 Testing

Run C++ GoogleTest unit and integration tests:

```bash
colcon test --packages-select lekiwi_perception --event-handlers console_direct+
colcon test-result --verbose
```

