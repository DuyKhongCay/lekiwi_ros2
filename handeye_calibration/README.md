# LeKiwi Hand-Eye Calibration Package (`handeye_calibration`)

Gói phần mềm ROS 2 thực hiện **Hand-Eye Calibration** (cho cả 2 dạng **Eye-to-Hand** và **Eye-in-Hand**) dành riêng cho robot **LeKiwi**.

---

## 🌟 Điểm nổi bật & Khác biệt

1. **Native OpenCV GUI (0ms Lag)**:
   - Thay thế hoàn toàn giao diện Web Viser (vốn dễ bị nghẽn WebSocket, giật lag hình ảnh).
   - Hiển thị trực tiếp hình ảnh stream từ camera, nhận diện ChArUco corners và trục toạ độ 3D thời gian thực.
2. **Quy trình Manual / Lead-Through tự động**:
   - Khi khởi động, node tự động gọi service `controller_manager/switch_controller` để chuyển các arm controller sang `INACTIVE`.
   - Tự động gọi `/set_torque_enabled` (`lekiwi_interfaces/srv/SetTorqueEnabled`) để nhả torque động cơ, cho phép người dùng cầm tay dắt cánh tay robot lấy mẫu một cách nhẹ nhàng.
3. **Độc lập và Gọn nhẹ**:
   - Loại bỏ phụ thuộc vào `so101_kinematics_msgs` hay các motion planner phức tạp.
   - Launch file và config YAML độc lập, không ảnh hưởng cấu hình bringup chính của robot.
4. **Hợp nhất Script tạo ChArUco Board**:
   - Tích hợp `gen_charuco_handeye.py` và `gen_multi_charuco_a4.py` thành 1 công cụ CLI duy nhất `generate_charuco.py` (hỗ trợ xuất PNG & PDF chuẩn in A4/A3).

---

## 🛠️ Hướng dẫn sử dụng

### 1. Tạo và in bảng ChArUco Calibration Target

Tạo 1 bảng đơn lẻ chuẩn A4:
```bash
ros2 run handeye_calibration generate_charuco --mode single --squares_x 3 --squares_y 4 --square_mm 6.0 --marker_mm 4.5 --output ~/charuco_target
```

Tạo 1 trang A4 chứa nhiều bảng kích thước khác nhau (kèm đường viền cắt tiện lợi):
```bash
ros2 run handeye_calibration generate_charuco --mode multi --output ~/charuco_multi_a4
```

---

### 2. Khởi động Robot và Cụm Camera

Chạy robot bringup và camera stream (chẳng hạn camera stereo trái):
```bash
ros2 launch lekiwi_bringup robot.launch.py
```

---

### 3. Chạy Node Calibration

Khởi động giao diện Calibration (Eye-to-Hand cho camera stereo gắn trên thân robot):
```bash
ros2 launch handeye_calibration handeye_calibration.launch.py \
    image_topic:=/cameras/stereo_left/image_raw \
    camera_info_topic:=/cameras/stereo_left/camera_info \
    is_eye_in_hand:=false
```

*(Nếu calib cho camera cổ tay `cameras/usb_wrist`, đổi `is_eye_in_hand:=true` và truyền topic tương ứng)*.

---

### 4. Thao tác trên Giao diện GUI

| Phím tắt | Thao tác | Mô tả |
| :---: | :--- | :--- |
| **`[SPACE]`** | **Take Sample** | Thu thập cặp biến đổi TF: $(T_{\text{robot}}, T_{\text{tracking}})$. Cần ít nhất 5–10 mẫu ở các góc khác nhau. |
| **`[C]`** | **Compute** | Tính toán ma trận biến đổi Hand-Eye qua các thuật toán (Tsai-Lenz, Park, Daniilidis,...), tự chọn kết quả sai số nhỏ nhất. |
| **`[S]`** | **Save** | Lưu kết quả ra file cấu hình `~/.ros/lekiwi_handeye_calibration.yaml`. |
| **`[T]`** | **Toggle Torque** | Bật / Tắt torque động cơ cánh tay thủ công khi cần. |
| **`[R]`** | **Reset** | Xóa toàn bộ các mẫu đã thu thập để lấy lại từ đầu. |
| **`[Q]`** / `ESC` | **Quit** | Đóng giao diện và thoát chương trình. |
