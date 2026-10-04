# chess_rosbag_to_lerobot

Chuyển đổi dữ liệu Rosbag2 MCAP của robot đánh cờ LeKiwi (tạo bởi `chess_episode_recorder`) sang định dạng **LeRobot Dataset v3.0** (`parquet` + `mp4`) phục vụ huấn luyện Physical-AI (ACT, Diffusion Policy, SmolVLA, OpenVLA).

## Tính năng nổi bật

1. **Anti-Future-Leakage Synchronizer**: Lấy nhịp tham chiếu từ camera cổ tay (`usb_wrist`), đồng bộ hóa các quan sát phụ (`usb_side`, `stereo_right`, `/joint_states`) tại thời điểm $t_{sample} \le t_{ref}$.
2. **SSOT Chess Spatial Conditioning**: Kế thừa chuẩn hình học từ `lekiwi_chess_master` và `lekiwi_motion/chessboard_mapper.hpp`, tự động tính toán tọa độ Cartesian 3D của ô gắp (pick pose) và ô đặt (place pose) đưa vào observation space.
3. **Multi-Camera Processing**: Giải mã trực tiếp luồng ảnh nén JPEG từ 3 camera (`384x384`) không cần cv_bridge.
4. **Hugging Face Hub Direct Push**: Hỗ trợ gắn token và push trực tiếp lên Hugging Face Hub (`huggingface.co/datasets/<repo-id>`).

## Hướng dẫn sử dụng

### 1. Cài đặt / Build package
```bash
cd ~/docker_ws/lekiwi_ros2
colcon build --packages-select chess_rosbag_to_lerobot
source install/setup.bash
```

### 2. Chuyển đổi cục bộ (Local Conversion)
```bash
chess_convert \
  --input-dir /media/usb_storage/lekiwi_episodes/chess_teleop_v1 \
  --config ~/docker_ws/lekiwi_ros2/chess_rosbag_to_lerobot/config/lekiwi_chess.yaml \
  --repo-id local/lekiwi_chess_v1 \
  --overwrite
```

### 3. Xem trực quan dữ liệu đã convert
```bash
lerobot-dataset-viz --repo-id local/lekiwi_chess_v1 --episode-index 0
```
