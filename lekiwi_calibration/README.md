# LeKiwi Calibration Suite (`lekiwi_calibration`)

Package ROS 2 chuẩn (`ament_python`) hợp nhất toàn bộ công cụ hiệu chuẩn (calibration tooling) ngoại tuyến cho robot **LeKiwi**:
1. **Chessboard AprilTag Calibration**: Tối ưu toạ độ 2D và yaw của 4 tag góc bàn cờ (A1, H1, H8, A8) bằng thuật toán Multi-View Planar Bundle Adjustment.
2. **Hand-Eye Calibration**: Cân chỉnh ngoại thông số Camera-to-Robot Extrinsics (Eye-to-Hand & Eye-in-Hand) với giao diện OpenCV native GUI (0ms lag) và hỗ trợ tự động nhả torque động cơ cánh tay.
3. **Omni Base Kinematic Calibration**: Tối ưu thông số bán kính bánh xe (`wheel_radius`), bán kính robot (`robot_radius`) và phân tích sai số UMBmark cho đế chuyển động đa hướng 3 bánh omni.

Toàn bộ kết quả cân chuẩn được tự động lưu về thư mục cấu hình tập trung (Single Source of Truth - SSOT): `lekiwi_bringup/config/calibration/`.

---

## 🚀 Hướng Dẫn Sử Dụng Nhanh

### 1. Cân chỉnh Bàn cờ AprilTag (`calibrate_chessboard`)
Khởi động node thu thập mẫu và giải Bundle Adjustment:
```bash
ros2 launch lekiwi_calibration calibrate_chessboard.launch.py
```
Hoặc chạy trực tiếp CLI:
```bash
ros2 run lekiwi_calibration calibrate_chessboard
```
**Phím tắt giao diện GUI:**
- `[SPACE]`: Chụp mẫu thủ công tại góc nhìn hiện tại (yêu cầu thấy $\ge 2$ tag).
- `[A]`: Bật/Tắt chế độ tự động chụp (`AUTO`).
- `[C]`: Kích hoạt tiến trình tối ưu phi tuyến Bundle Adjustment (chạy ngầm không đơ GUI).
- `[S]`: Xuất kết quả đã tối ưu ra `lekiwi_bringup/config/calibration/chessboard_tags.yaml`.
- `[R]`: Xóa toàn bộ các mẫu chụp để lấy lại từ đầu.
- `[Q]` hoặc `ESC`: Thoát ứng dụng.

---

### 2. Cân chỉnh Hand-Eye (`calibrate_handeye`)
Sinh target ChArUco để in ra giấy (nếu chưa có):
```bash
# In bảng đơn tiêu chuẩn
ros2 run lekiwi_calibration generate_charuco --mode single --output ~/charuco_target

# Hoặc in bảng ghép A4 nhiều tỉ lệ
ros2 run lekiwi_calibration generate_charuco --mode multi --output ~/charuco_multi_a4
```

Khởi động giao diện cân chỉnh Hand-Eye:
```bash
# Eye-to-Hand (Camera Stereo gắn cố định trên thân robot quan sát cánh tay)
ros2 launch lekiwi_calibration calibrate_handeye.launch.py

# Eye-in-Hand (Camera gắn trên cổ tay robot di chuyển theo gripper)
ros2 launch lekiwi_calibration calibrate_handeye.launch.py \
    params_file:=/path/to/custom_handeye_params.yaml
```

**Phím tắt giao diện GUI:**
- `[SPACE]`: Thu thập một cặp biến đổi $(T_{\text{robot}}, T_{\text{tracking}})$. Cần $\ge 5 - 10$ mẫu ở nhiều tư thế khác nhau.
- `[C]`: Giải nghiệm Hand-Eye (so sánh đồng thời Tsai-Lenz, Park, Horaud, Andreff, Daniilidis để chọn sai số dư nhỏ nhất).
- `[S]`: Lưu cấu hình TF ra `lekiwi_bringup/config/calibration/handeye_stereo_left.yaml`.
- `[T]`: Bật/Tắt torque cánh tay robot thủ công.
- `[R]`: Reset toàn bộ mẫu đã thu thập.
- `[Q]` hoặc `ESC`: Thoát chương trình.

---

### 3. Cân chỉnh Hệ Truyền Động Omni (`calibrate_omni_base`)
Đảm bảo robot bringup và controller đang hoạt động, sau đó kích hoạt quy trình calib:

#### Chế độ Quay tại chỗ (Spin Test - Tối ưu `robot_radius` qua IMU Ground Truth):
```bash
ros2 launch lekiwi_calibration calibrate_omni_base.launch.py \
    calib_mode:=spin \
    rot_cnt:=5 \
    angular_vel:=0.5
```

#### Chế độ Chạy thẳng (Rollout Test - Tối ưu `wheel_radius`):
```bash
ros2 launch lekiwi_calibration calibrate_omni_base.launch.py \
    calib_mode:=rollout \
    test_dist:=1.0 \
    linear_vel:=0.15 \
    actual_measured_dist:=1.025
```

#### Chế độ Đường chạy vuông UMBmark (Square Test - Đo hệ số sai số Type A & Type B):
```bash
ros2 launch lekiwi_calibration calibrate_omni_base.launch.py \
    calib_mode:=square \
    test_dist:=1.0
```

---

## 📁 Cấu Trúc Package
```text
lekiwi_calibration/
├── config/
│   ├── chessboard_calib_params.yaml
│   ├── handeye_params.yaml
│   └── omni_base_calib_params.yaml
├── launch/
│   ├── calibrate_chessboard.launch.py
│   ├── calibrate_handeye.launch.py
│   └── calibrate_omni_base.launch.py
├── lekiwi_calibration/
│   ├── chessboard/
│   │   ├── calibrator_node.py
│   │   ├── solver.py
│   │   └── visualizer.py
│   ├── handeye/
│   │   ├── charuco_detector.py
│   │   ├── generate_charuco.py
│   │   ├── handeye_calibration_node.py
│   │   └── handeye_solver.py
│   └── omni/
│       ├── kinematics_calib.py
│       └── omni_base_calibrator_node.py
└── test/
    ├── test_chessboard_solver.py
    ├── test_handeye_solver.py
    └── test_omni_kinematics.py
```

