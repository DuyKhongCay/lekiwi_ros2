# LeKiwi ROS 2: Nền Tảng Robot Di Động Thao Tác & Trí Tuệ Nhân Tạo Vật Lý (Physical-AI)

<div align="center">

[![English](https://img.shields.io/badge/Language-English-blue.svg)](README.md)
[![Tiếng Việt](https://img.shields.io/badge/Ngôn_ngữ-Tiếng_Việt-red.svg)](README.vi.md)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy%20%7C%20Humble-22314E.svg?logo=ros)](https://docs.ros.org/)
[![Hardware](https://img.shields.io/badge/Hardware-Raspberry%20Pi%205-C51A4A.svg?logo=raspberrypi)](https://www.raspberrypi.com/)
[![HailoRT](https://img.shields.io/badge/NPU-Hailo--8%2F8L%2026TOPS-00A651.svg)](https://hailo.ai/)
[![Physical-AI](https://img.shields.io/badge/Physical--AI-LeRobot%20v3.0-FF6F00.svg)](https://github.com/huggingface/lerobot)
[![Middleware](https://img.shields.io/badge/Middleware-Zenoh%20RMW-black.svg)](https://zenoh.io/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

**[English](README.md)** | **[Tiếng Việt](README.vi.md)**

<p align="center">
  <em>Nền tảng robot di động thao tác để bàn mã nguồn mở tích hợp đế di chuyển đa hướng holonomic, cánh tay 6 bậc tự do (6-DoF), thị giác máy tính tăng tốc bằng NPU biên, chơi cờ vua tự động và học bắt chước hành vi Physical-AI hiện đại nhất.</em>
</p>

</div>

---

> [!IMPORTANT]
> ### 📚 Cổng Tài Liệu Kỹ Thuật Chuyên Sâu & GitHub Wiki
> Tệp README này đóng vai trò như trang giới thiệu tổng quan, tầm nhìn dự án và các bước làm quen nhanh. Để tra cứu toàn bộ sổ tay kỹ thuật chuyên sâu — bao gồm **phân tích chi tiết Kiến trúc hệ thống, danh mục 15 Packages hoạt động, chuẩn giao tiếp ROS 2 Action/Service, sơ đồ đấu dây phần cứng, quy trình hiệu chuẩn Hand-Eye và hướng dẫn triển khai Docker phân tán** — xin mời truy cập GitHub Wiki chính thức của dự án:
> 
> 👉 **[Khám phá GitHub Wiki Dự Án LeKiwi](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)**

---

## 🌟 Tầm Nhìn Dự Án & Điểm Nhấn Công Nghệ

**LeKiwi** là nền tảng robot di động thao tác (Mobile Manipulator) tích hợp toàn diện, được thiết kế nhằm kết nối robot vật lý thực tế với **Trí tuệ nhân tạo vật lý (Physical-AI: Imitation Learning & mô hình VLA)** và **Trí tuệ nhân tạo tại biên (Edge AI)**. Với cấu trúc gồm đế 3 bánh omni đa hướng và cánh tay nối tiếp 6-DoF, LeKiwi minh chứng khả năng tương tác thế giới thực phức tạp thông qua bài toán chơi cờ vua tự động với con người.

```
       [ Cặp Camera Stereo CSI ]            [ 9x Động Cơ Servo Feetech STS3215 ]
                   │                                         │
                   ▼                                         ▼
   ┌───────────────────────────────┐         ┌───────────────────────────────┐
   │      NPU HAT Hailo-8 M.2      │         │   Phần Cứng ros2_control      │
   │    Thị Giác Stereo & 26 TOPS  │         │  Bus Nối Tiếp Bất Đồng Bộ 1M  │
   └───────────────┬───────────────┘         └───────────────▲───────────────┘
                   │                                         │
                   ▼                                         │
   ┌─────────────────────────────────────────────────────────┴───────────────┐
   │           Hệ Thống Điều Phối & FSM Nhiệm Vụ Tự Hành LeKiwi              │
   │        Engine Cờ Vua Stockfish 16  ◄──►  Server LeRobot ZeroMQ          │
   └─────────────────────────────────────────────────────────────────────────┘
```

### Các Năng Lực Cốt Lõi
* **♟️ Trí Tuệ Trò Chơi Tự Hành:** Chơi cờ vua tự động từ đầu đến cuối dựa trên engine Stockfish 16, FSM điều phối nhiệm vụ động và thị giác máy tính nhận diện nước cờ đối thủ.
* **⚡ Thị Giác Biên Tăng Tốc Phần Cứng:** Xử lý luồng ảnh stereo dual CSI và mạng nơ-ron YOLO đạt trên 30 FPS theo thời gian thực nhờ card NPU Hailo-8 (26 TOPS) trên Raspberry Pi 5.
* **🤖 Điều Khiển Thao Tác Chuẩn Xác:** Cánh tay 6-DoF được truyền động bởi 9 servo thông minh Feetech STS3215 thông qua lớp trừu tượng phần cứng `ros2_control` bất đồng bộ trên bus UART 1 Mbps.
* **🦾 Học Bắt Chước Hành Vi Physical-AI:** Cầu nối streaming nhị phân tốc độ cao kết nối các mô hình chính sách LeRobot (ACT, Diffusion Policy, SmolVLA) với ROS 2 qua giao thức ZeroMQ.
* **🌐 Mạng Phân Tán Triệt Tiêu Multicast:** Triển khai gọn gàng trên Raspberry Pi 5 bằng Docker và middleware Zenoh (`rmw_zenoh_cpp`), cho phép kết nối trạm PC giám sát RViz2 từ xa qua Wi-Fi mà không lo nghẽn mạng multicast.

---

## 📸 Hình Ảnh Hoạt Động Thực Tế (Showcase)

Dưới đây là các kịch bản hoạt động tiêu biểu của robot LeKiwi. *(Hình ảnh thực tế, video demo và ảnh động GIF đang liên tục được ghi hình và bổ sung vào các khung bên dưới)*.

### 1. 🤖 Robot Thật & Khung Gầm 3 Bánh Omni
Robot sở hữu đế holonomic 3 bánh omni linh hoạt có khả năng tịnh tiến đa hướng và xoay tròn tại chỗ, tích hợp bộ não máy tính Raspberry Pi 5 và hệ thống quản lý nguồn pin.

| Toàn Cảnh Robot Thực Tế | Cấu Trúc Khung Gầm & Cánh Tay |
| :---: | :---: |
| ![Tổng quan robot LeKiwi](docs/images/showcase/lekiwi_robot_overview.png)<br>*(Vị trí ảnh: Toàn cảnh robot LeKiwi sẵn sàng trên sàn thử nghiệm)* | ![Chi tiết khung gầm và khớp tay](docs/images/showcase/lekiwi_chassis_arm.png)<br>*(Vị trí ảnh: Chi tiết cơ khí đế 3 bánh omni và cụm 6 khớp tay)* |

---

### 2. ♟️ Đại Kiện Tướng Cờ Vua Tự Hành & Gắp Nhả 6-DoF
LeKiwi tự động phát hiện ô cờ, nhận diện nước đi của người chơi, tính toán nước cờ tối ưu bằng Stockfish 16, và thực hiện quỹ đạo gắp nhả quân cờ với cơ chế kiểm tra vùng vươn tới (reachability).

| Thị Giác Bàn Cờ & FEN | Thao Tác Gắp Nhả Quân Cờ |
| :---: | :---: |
| ![Nhận diện bàn cờ](docs/images/showcase/chess_vision_evaluation.png)<br>*(Vị trí ảnh: Nhận diện góc bàn cờ và chuỗi trạng thái FEN qua YOLO)* | ![Gắp nhả quân cờ](docs/images/showcase/chess_piece_manipulation.png)<br>*(Vị trí ảnh: Kẹp tay robot gắp và di chuyển quân cờ chuẩn xác)* |

---

### 3. 👁️ Thị Giác Máy Tính & NPU Hailo-8 Tăng Tốc
Được trang bị 2 camera CSI IMX và card NPU Hailo-8 26 TOPS, LeKiwi xử lý tính toán ảnh stereo, bình sai bàn cờ và nhận diện đối tượng với mức tải CPU Pi 5 cực thấp.

| Luồng Xử Lý Camera Stereo | Nhận Diện Bằng NPU Hailo |
| :---: | :---: |
| ![Luồng camera stereo](docs/images/showcase/stereo_camera_stream.png)<br>*(Vị trí ảnh: Luồng ảnh stereo hai mắt được nắn thẳng quang học)* | ![Bounding box từ Hailo](docs/images/showcase/hailo_yolo_detection.png)<br>*(Vị trí ảnh: Khung nhận diện vật thể YOLO thời gian thực ở 30 FPS)* |

---

### 4. 🦾 Điều Khiển Từ Xa & Thu Thập Dữ Liệu Physical-AI
Thu thập các episode mẫu (demonstrations) thông qua cánh tay Leader Zhongli / uArm hoặc tay cầm Gamepad, ghi nhận rosbag MCAP không nén và chuyển đổi mượt mà sang định dạng Hugging Face LeRobot.

| Cánh Tay Điều Khiển Teleop | Triển Khai Chính Sách AI (Policy) |
| :---: | :---: |
| ![Dạy robot bằng cánh tay leader](docs/images/showcase/teleop_leader_arm.png)<br>*(Vị trí ảnh: Người dạy thao tác cho robot bằng cánh tay đối ứng)* | ![Chính sách AI tự động chạy](docs/images/showcase/lerobot_inference_rollout.png)<br>*(Vị trí ảnh: Robot tự động lặp lại động tác học được từ mô hình AI)* |

---

## 🚀 Cổng Hướng Dẫn Cài Đặt & Tài Liệu Kỹ Thuật

> [!TIP]
> ### 🛠️ Sẵn Sàng Build & Chạy Thử Nghiệm LeKiwi?
> Toàn bộ hướng dẫn từng bước chi tiết, cài đặt môi trường, chế độ mô phỏng và triển khai mạng phân tán hai máy đều được tổng hợp đầy đủ tại **GitHub Wiki của dự án**:
> 
> * 🐳 **[Triển Khai Container Docker Trên Raspberry Pi 5](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Chạy stack đóng gói sẵn kèm driver Hailo-8 NPU và Zenoh.
> * 💻 **[Build Mã Nguồn & Chạy Mô Phỏng Trên Máy Tính](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Thiết lập workspace `colcon` trên Ubuntu 24.04/22.04 mà không cần phần cứng thật.
> * ♟️ **[Hướng Dẫn Vận Hành Trận Đấu Cờ Vua Tự Hành](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Khởi động FSM điều phối nhiệm vụ và đường ống thị giác Stockfish.
> * 🦾 **[Thu Thập Dữ Liệu Physical-AI & Huấn Luyện LeRobot](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Ghi lại episode MCAP và chạy suy luận chính sách bắt chước hành vi.
> * 🌐 **[Giám Sát Từ Xa Qua PC Với Zenoh & RViz2](docs/how-to-guides/deployment/distributed-pi5-docker-and-pc-zenoh-setup.md)** — Kết nối máy tính trạm qua Wi-Fi để xem trực quan mô hình 3D và camera stereo.
> 
> 👉 **[Khám Phá Toàn Bộ Hướng Dẫn Cài Đặt Trên GitHub Wiki](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)**

---

## 🗺️ Lộ Trình Phát Triển & Các Cột Mốc (Roadmap)

Quá trình phát triển robot LeKiwi được chia thành các giai đoạn chiến lược từ trừu tượng hóa phần cứng cơ sở tới trí tuệ nhân tạo vật lý tự hành hoàn chỉnh:

- [x] **Giai đoạn 1: Trừu Tượng Hóa Phần Cứng & Khung Gầm Đa Hướng**
  - [x] Động học và đo đạc hành trình (odometry) cho đế holonomic 3 bánh omni.
  - [x] Tích hợp cảm biến IMU 9-DoF ICM-20948 qua giao tiếp I2C phần cứng.
  - [x] Driver bus serial bất đồng bộ cho chuỗi servo Feetech STS3215 ở tốc độ 1 Mbps với `ros2_control`.
  - [x] Thiết lập hệ thống udev rules cố định cổng thiết bị phần cứng.

- [x] **Giai đoạn 2: Thị Giác Biên & Tăng Tốc Phần Cứng NPU**
  - [x] Pipeline camera stereo 2 mắt CSI điều khiển linh hoạt qua GStreamer.
  - [x] Tích hợp card NPU Hailo-8 M.2 HAT+ (26 TOPS) trên Raspberry Pi 5.
  - [x] Nhận diện ô bàn cờ và quân cờ thời gian thực bằng mạng YOLO ở tốc độ 30+ FPS.
  - [x] Bộ công cụ hiệu chuẩn Hand-Eye và bình sai góc bàn cờ AprilTag.

- [x] **Giai đoạn 3: Tự Hành Đánh Cờ Vua & Điều Phối Nhiệm Vụ**
  - [x] Tích hợp engine cờ vua Stockfish 16 qua ROS 2 action server (`ComputeBestMove`).
  - [x] Máy trạng thái FSM điều phối nhiệm vụ cấp cao và chuyển đổi chế độ camera động.
  - [x] Hoạch định quỹ đạo chuyển động Cartesian 6-DoF kèm kiểm tra vùng vươn tới (reachability).
  - [x] Khép kín chu trình thi đấu cờ vua tự động giữa người và robot.

- [/] **Giai đoạn 4: Trí Tuệ Nhân Tạo Vật Lý (Physical-AI LeRobot)**
  - [x] Cầu nối streaming ZeroMQ thông lượng cao giữa mô hình LeRobot và ROS 2.
  - [x] Ghi nhận episode mẫu định dạng MCAP qua gamepad và cánh tay leader (`chess_episode_recorder`).
  - [x] Pipeline chuyển đổi dữ liệu không suy hao sang chuẩn Hugging Face LeRobot v3.0.
  - [🟡] Thử nghiệm và đánh giá toàn diện các chính sách ACT, Diffusion Policy và SmolVLA trên robot thật.

- [ ] **Giai đoạn 5: Tự Hành Toàn Không Gian (Nav2) & Tương Tác VLA Đa Nhiệm**
  - [ ] Tích hợp Nav2 với SLAM Toolbox và odometry thị giác/lidar cho di chuyển tự do trong phòng.
  - [ ] Điều phối nhiệm vụ bằng giọng nói và mô hình thị giác-ngôn ngữ-hành động (VLA).
  - [ ] Đồng bộ chính sách liên tục giữa trạm huấn luyện đám mây và máy tính biên robot.

---

## 🤝 Các Dự Án Tham Khảo & Lời Tri Ân (Acknowledgements)

Hệ sinh thái LeKiwi được xây dựng dựa trên sự kế thừa và học hỏi từ các dự án mã nguồn mở xuất sắc trong lĩnh vực Robotics, Trí tuệ nhân tạo vật lý và Điện toán biên. Chúng tôi xin chân thành cảm ơn các dự án tham khảo tiêu biểu sau:

| Nhóm Lĩnh Vực | Dự Án Tham Khảo | Vai Trò & Đóng Góp Trong Hệ Thống LeKiwi |
| :--- | :--- | :--- |
| **1. Cơ Khí, Động Cơ & Khung Gầm** | [`DuyKhongCay/lekiwi-robot`](https://github.com/DuyKhongCay/lekiwi-robot) | Bản vẽ thiết kế cơ khí CAD, file in 3D STL, danh mục linh kiện BOM và hướng dẫn lắp ráp vật lý cho robot. |
| | [`SIGRobotics-UIUC/LeKiwi`](https://github.com/SIGRobotics-UIUC/LeKiwi) | Thiết kế cơ khí nền tảng và ý tưởng gốc của robot di động đa hướng LeKiwi. |
| | [`legalaspro/feetech_ros2_driver`](https://github.com/legalaspro/feetech_ros2_driver) | Driver giao tiếp serial bất đồng bộ cho chuỗi servo thông minh Feetech STS3215. |
| | [`NTkot/icm20948_i2c`](https://github.com/NTkot/icm20948_i2c) | Thư viện tầng thấp giao tiếp và trích xuất dữ liệu cảm biến IMU 9 trục ICM-20948 qua I2C. |
| | [`MINT-SJTU/LeRobot-Anything-U-Arm`](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm) | Giao thức giao tiếp và giải thuật ánh xạ khớp cho cánh tay điều khiển từ xa Leader U-Arm / Zhongli. |
| **2. Physical-AI & Học Bắt Chước** | [`huggingface/lerobot`](https://github.com/huggingface/lerobot) | Nền tảng Physical-AI cốt lõi để huấn luyện các chính sách mô phỏng hành vi (ACT, Diffusion, SmolVLA). |
| | [`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai) | Mô hình kiến trúc cầu nối ZeroMQ hiệu năng cao giữa ROS 2 và LeRobot policy server. |
| | [`fiveages-sim/lerobot_ros2`](https://github.com/fiveages-sim/lerobot_ros2) | Quy trình tích hợp LeRobot trong môi trường mô phỏng ROS 2 và thu thập teleop. |
| **3. Thị Giác Biên & NPU Tăng Tốc** | [`hailo-ai/hailo-apps`](https://github.com/hailo-ai/hailo-apps) | Kiến trúc đường ống tăng tốc phần cứng GStreamer và mô hình suy luận trên NPU Hailo-8 M.2. |
| | [`ros-drivers/gscam`](https://github.com/ros-drivers/gscam) | Driver chuẩn cầu nối giữa luồng camera GStreamer và ROS 2 Image topics. |
| **4. Thuật Toán Cờ Vua & Trí Tuệ** | [`Disservin/chess-library`](https://github.com/Disservin/chess-library) | Thư viện C++ header-only hiệu năng cao biểu diễn bàn cờ và sinh nước đi hợp lệ cho Stockfish. |
| | [`wojciechc1/AI_Chess`](https://github.com/wojciechc1/AI_Chess) | Giải pháp kết hợp thị giác máy tính nhận diện bàn cờ và trí tuệ nhân tạo cờ vua. |
| **5. Chuẩn Kỹ Thuật & Công Cụ AI** | [`dbwls99706/ros2-engineering-skills`](https://github.com/dbwls99706/ros2-engineering-skills) | Bộ quy chuẩn thực hành kỹ thuật phần mềm, cấu hình executor và vòng đời lifecycle ROS 2. |
| | [`github/awesome-copilot`](https://github.com/github/awesome-copilot) | Hệ sinh thái AI Agents, skills và các hướng dẫn lập trình tăng tốc cho dự án. |

> [!NOTE]
> **Tri Ân Cộng Đồng Mở Rộng:** Bên cạnh các repository nêu trên, LeKiwi còn kế thừa vô vàn bài báo khoa học, thảo luận kỹ thuật từ cộng đồng ROS 2, các checkpoint mô hình mở từ Hugging Face và các driver kernel Linux. Xin gửi lời cảm ơn sâu sắc nhất tới cộng đồng nghiên cứu Robotics và Physical-AI mã nguồn mở toàn cầu!

---

> [!NOTE]
> ### 🤖 Thông Báo Về Tài Liệu Do AI Hỗ Trợ Biên Soạn
> Bộ tài liệu dự án này được hỗ trợ biên soạn và cấu trúc hóa bởi các AI engineering agents chuyên sâu. Mặc dù mọi phần nội dung đều được đối soát chéo với mã nguồn thực tế, tài liệu tự động đôi khi vẫn có thể chứa sai lệch nhỏ hoặc thông số chưa kịp cập nhật. Nếu bạn phát hiện bất kỳ lỗi kỹ thuật hoặc liên kết hỏng nào, vui lòng [mở một GitHub Issue](https://github.com/DuyKhongCay/lekiwi_ros2/issues) để giúp các maintainers và contributors hoàn thiện dự án!

---

## 📜 Giấy Phép & Thông Tin Tác Giả

* **Giấy phép (License):** Phát hành theo điều khoản [Giấy phép Apache-2.0](LICENSE).
* **Tác giả & Quản trị dự án:** **DuyKhongCay** ([@DuyKhongCay](https://github.com/DuyKhongCay))
* **Cổng GitHub Wiki Kỹ Thuật:** [https://github.com/DuyKhongCay/lekiwi_ros2/wiki](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)

