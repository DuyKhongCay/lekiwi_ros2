# lekiwi_ros2

## HailoRT TAPPAS 
```bash
python3 /home/duykhongcay/hailo_ws/hailo-apps/hailo_apps/python/pipeline_apps/hailo_chess_vision/chess_vision.py \
  --input /home/duykhongcay/lerobot_ws/video.mp4 \
  --hef-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg-2/real_hailo_model/yolov8n-seg.hef \
  --hef-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chess_detection_yolo11n-3/real_hailo_model/yolo11n.hef \
  --output-file /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/predict/result.mp4
  --width 3280 --height 2464 --rotation 180 -v
```
```bash
#Debug GST pipeline with hailonet

export GST_DEBUG_DUMP_DOT_DIR=/home/duykhongcay/docker_ws

cd /home/duykhongcay/docker_ws
for f in *.dot; do dot -Tsvg "$f" -o "${f%.dot}.svg"; done
```
```bash
gst-inspect-1.0 sharktracers

GST_SHARK_CTF_DISABLE= \
  GST_DEBUG="GST_TRACER:7" \
  GST_TRACERS="proctime;interlatency;framerate;scheduletime;cpuusage;queuelevel" \
  GST_SHARK_LOCATION=/home/duykhongcay/docker_ws/lekiwi_ros2/log/trace \
  ros2 launch hailo_perception_node hailo_chess_standalone.launch.py

  ./scripts/graphics/gstshark-plot /home/duykhongcay/docker_ws/lekiwi_ros2/log/trace --savefig pdf
```
## Hailo DFC
### Sử dụng CLI của `yolo` (Ultralytics CLI gốc) convert yolo model sang onnx format
Nếu bạn không muốn chạy qua script Python mà muốn dùng trực tiếp lệnh CLI chuẩn của Ultralytics:
```bash
export CUDA_VISIBLE_DEVICES="0"
export TF_FORCE_GPU_ALLOW_GROWTH="true"
export TF_RAM_ALLOCATOR_BYTES_LIMIT="1610612736"
export TF_GPU_ALLOCATOR="cuda_malloc_async"
export TF_CPP_MIN_LOG_LEVEL="2"
export SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])")
export LD_LIBRARY_PATH=$SITE_PACKAGES/nvidia/cudnn/lib:$SITE_PACKAGES/nvidia/cublas/lib:$LD_LIBRARY_PATH
```

```bash
yolo mode=export \
  model=/home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg/weights/best.pt \
  format=hailo \
  name=hailo8 \
  imgsz=640 \
  simplify=True \
  quantize=int8 \
  device=0 \
  fraction=1.0 \
  data=/home/duykhongcay/hailo_ws/chess_pieces_detection/datasets/chessboard-segmentation/calib.yaml \
  project=/home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg
```

```bash
yolo mode=export \
  model=/home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg/weights/best.pt \
  format=onnx \
  imgsz=640 \
  simplify=True \
  device=cpu \
  opset=12
```

```bash
hailo parser onnx \
  --net-name shared-input \
  --har-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/join_input_yolov8n_native_model.har \
  --end-node-names \
    "/model.22/cv2.0/cv2.0.2/Conv" \
    "/model.22/cv3.0/cv3.0.2/Conv" \
    "/model.22/cv4.0/cv4.0.2/Conv" \
    "/model.22/cv2.1/cv2.1.2/Conv" \
    "/model.22/cv3.1/cv3.1.2/Conv" \
    "/model.22/cv4.1/cv4.1.2/Conv" \
    "/model.22/cv2.2/cv2.2.2/Conv" \
    "/model.22/cv3.2/cv3.2.2/Conv" \
    "/model.22/cv4.2/cv4.2.2/Conv" \
    "/model.22/proto/cv3/act/Mul"  \
  --hw-arch hailo8 \
  '/home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg/weights/best.onnx'



```
---

### Hướng dẫn chạy tạo báo cáo Profile đầy đủ:

Bạn có thể chạy lệnh sau từ terminal (hoặc bên trong container Hailo Docker):
```bash
# Trên host pc
hailo compiler /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chess_detection_yolo11n-2/hailo_chesspieces_detect/yolo11n_hailo8.har \
  --hw-arch hailo8 \
  --output-har-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chess_detection_yolo11n-2/hailo_chesspieces_detect \
  --output-dir /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chess_detection_yolo11n-2/hailo_chesspieces_detect
```

```bash
# Trên edge device
hailortcli run -m streaming \
    --power-mode performance \
    --measure-latency \
    --measure-overall-latency \
    --measure-temp \
    --show-progress \
    -t 10 \
    /usr/local/hailo/resources/models/hailo8/yolo11s_obb.hef \
    collect-runtime-data \
    --output-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/other_models/yolov11-obb_runtime_data.json \
    measure-stats \
    --elem-fps \
    --elem-queue-size \
    --output-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/other_models/yolov11-obb_pipeline_stats.csv

hailortcli run \
  /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chess_detection_yolo11n-3/real_hailo_model/yolo11n.hef \
  --dot /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chess_detection_yolo11n-3/real_hailo_model/yolo11n.dot

  hailortcli benchmark -t 10 --batch-size 8 /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/other_models/yolov11m.hef

# Nhiều model cùng lúc
hailortcli run2 -m full_async \
    --scheduling-algorithm round_robin \
    --measure-temp \
    -t 20 \
    -j /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/multimodel_runtime_data.json \
    set-net /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg/hailo_model/yolov8n-seg.hef \
    --batch-size 2 \
    set-net /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chess_detection_yolo11n-2/hailo_model-3/yolo11n.hef \
    --batch-size 2 

```

``` bash
# SNR Noise Analysis
hailo analyze-noise \
  /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/auto_join_inputs_quantized_model.har \
  --data-path /home/duykhongcay/hailo_ws/chess_pieces_detection/datasets/rbflow_chesspieces_dataset/calib_dataset.npy \
  --batch-size 2 --data-count 16


#Trên host PC

hailo profiler \
  /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg-2/real_hailo_model/yolov8n-seg_compiled_model.har \
  --out-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg-2/real_hailo_model/yolov8n-seg_runtime_profile.html \
  --runtime-data /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg-2/real_hailo_model/yolov8n-seg_runtime_data.json 

# Join model
hailo join \
    /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/join_input_yolo11_native_model.har \
    /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/join_input_yolov8n_native_model.har \
    --scope-name1 yolo11n-det \
    --scope-name2 yolov8n-seg \
    --join-action auto_join_inputs \
    --output-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/auto_join_inputs_hailo_model.har

# Compile model
hailo compiler --hw-arch hailo8 \
  --model-script /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/model_scripts.alls \
  --output-dir /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model \
  --output-har-path /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/auto_join_inputs_compiled_model.har \
  /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/hailo_joined_model/auto_join_inputs_quantized_model.har
```


Chính xác! Cần phân biệt rõ hai khái niệm này trong Hailo Dataflow Compiler (DFC):

---

### 1. Thuật toán "Layer Noise Analysis" NỘI BỘ bên trong `runner.optimize()`
* **Bản chất:** Là một thuật toán con (Optimization Algorithm) nằm trong đường ống (pipeline) tự động của `runner.optimize()`.
* **Mục đích:** Giúp DFC tự động đánh giá độ nhạy nhiễu của các lớp để phục vụ cho các bước tối ưu hóa tiếp theo (như Fine-tune encoding, Mixed Precision, QFT).
* **Đặc điểm:** Tự động chạy với tập dữ liệu calibration mặc định của `optimize()`, không có các tham số đầu ra riêng như `batch_size` hay `data_count`.

---

### 2. API độc lập `runner.analyze_noise(...)` (API gốc của `ClientRunner`)
* **Bản chất:** Là phương thức API riêng biệt được gọi trực tiếp trên đối tượng `runner` (`runner.analyze_noise(...)`).
* **Mục đích:** Đo đạc chi tiết tỉ số tín hiệu trên nhiễu **Logits SNR (dB)**, phân phối kích hoạt (Activation distribution) và đồ thị phân tán (Scatter Plot) trên từng layer.
* **Tham số & Kết quả:** Nhận tham số độc lập `batch_size=2, data_count=16` và đóng gói toàn bộ bảng thống kê phân tích nhiễu vào file `.har` để xuất báo cáo trực quan trên **Hailo Profiler HTML**.

---

Trong [`scripts/export_hailo_model.py`](file:///home/duykhongcay/hailo_ws/chess_pieces_detection/scripts/export_hailo_model.py#L238-L242), chúng ta đã gọi chuẩn xác **API gốc `runner.analyze_noise`**:

```python
        # 1. Chạy tiến trình lượng hóa chung
        runner.optimize(calib_dataset_dict)
        
        # 2. Gọi API gốc analyze_noise độc lập với tham số batch_size=2, data_count=16
        print("[+] Running Layer Noise Analysis (batch_size=2, data_count=16)...")
        runner.analyze_noise(calib_dataset_dict, batch_size=2, data_count=16)
        
        # 3. Lưu toàn bộ kết quả phân tích nhiễu vào file HAR
        runner.save_har(str(quantized_har_path))
```

## Dưới đây là giải thích chi tiết từng chế độ (subcommand) của công cụ **Hailo CLI** (`hailo`) theo quy trình làm việc thực tế với **Hailo Dataflow Compiler (DFC)**:

---

### 1. Luồng chuyển đổi & Biên dịch chính (Core Compilation Pipeline)

Đây là 3 câu lệnh cốt lõi được sử dụng theo thứ tự để đưa một mô hình Deep Learning thông thường lên chip Hailo:

1. **`parser`** *(Translate network to Hailo network)*
   - **Chức năng:** Dịch (Parse) mô hình gốc từ các framework như ONNX, TensorFlow, TFLite sang định dạng đồ thị mạng chuẩn của Hailo (`.har` - Hailo Archive).
   - **Đầu ra:** File `.har` ở trạng thái `translated`.

2. **`optimize`** *(Optimize model)*
   - **Chức năng:** Tối ưu hóa mô hình, thực hiện **lượng hóa (Quantization)** từ FP32 xuống INT8 bằng tập dữ liệu calibration (`calib_dataset`). Bước này giúp mô hình chạy cực nhanh trên chip Hailo mà giữ nguyên độ chính xác.
   - **Đầu ra:** File `.har` ở trạng thái `quantized`.

3. **`compiler`** *(Compile Hailo model to HEF binary files)*
   - **Chức năng:** Biên dịch mô hình đã được tối ưu thành file nhị phân **HEF** *(Hailo Executable Format)*. Đây là file cuối cùng để nạp vào phần cứng chip Hailo NPU.
   - **Đầu ra:** File `.hef`.

---

### 2. Phân tích & Đánh giá hiệu năng (Profiling & Visualization)

Các công cụ giúp bạn kiểm tra băng thông, độ trễ, FPS và lỗi lượng hóa:

4. **`profiler`** *(Hailo models Profiler)*
   - **Chức năng:** Tạo báo cáo HTML chi tiết về hiệu năng mô hình (băng thông, thời gian xử lý từng layer, FPS dự kiến, tài nguyên tiêu thụ). Có thể chạy dạng ước tính (Compiler Estimation) hoặc kết hợp với dữ liệu thực tế từ chip (`--runtime-data`).

5. **`runtime-profiler`** *(Hailo Runtime Profiler)*
   - **Chức năng:** Chạy mô hình trực tiếp trên thiết bị phần cứng thật (hoặc qua HailoRT) để thu thập thông số thời gian chạy thực tế (sinh ra file `runtime_data.json`).

6. **`analyze-noise`** *(Analyze network quantization noise)*
   - **Chức năng:** Phân tích nhiễu (noise) sinh ra do quá trình lượng hóa INT8. So sánh sai số đầu ra giữa mô hình gốc FP32 và mô hình đã lượng hóa INT8 để phát hiện các layer bị sụt giảm độ chính xác.

7. **`visualizer`** *(HAR visualization tool)*
   - **Chức năng:** Hiển thị trực quan đồ thị mạng nơ-ron Hailo (`.har`) dạng đồ họa tương tác trên giao diện web/browser.

8. **`har`** *(Query and extract information from Hailo Archive file)*
   - **Chức năng:** Truy vấn thông tin nội bộ, trích xuất cấu trúc, danh sách lớp hoặc kiểm tra trạng thái của file `.har`.

---

### 3. Công cụ nâng cao & Tiện ích bổ sung (Advanced & Utilities)

9. **`join`** *(Join two Hailo models to a single model)*
   - **Chức năng:** Ghép 2 mô hình Hailo riêng biệt lại thành 1 mô hình duy nhất để chạy song song hoặc nối tiếp trên chip Hailo.

10. **`har-onnx-rt`** *(Generates ONNX-Runtime model including pre/post processing)*
    - **Chức năng:** Tạo mô hình chạy bằng ONNX-Runtime tích hợp sẵn các bước tiền xử lý (pre-processing) và hậu xử lý (post-processing).

11. **`dfc-studio`** *(Start DFC Studio)*
    - **Chức năng:** Khởi chạy giao diện đồ họa DFC Studio GUI giúp thao tác chuyển đổi mô hình qua giao diện web trực quan thay vì gõ lệnh CLI.

12. **`params-csv`** *(Convert translated params to csv)*
    - **Chức năng:** Xuất các thông số, trọng số (weights/biases) của mô hình sau khi dịch ra dạng CSV để dễ dàng kiểm tra, phân tích thủ công.

13. **`tutorial`** *(Runs the tutorials in jupyter notebook)*
    - **Chức năng:** Mở danh sách các bài hướng dẫn (Tutorial Jupyter Notebooks) có sẵn của Hailo để người mới học dễ theo dõi.

14. **`help`** *(Show the list of commands)*
    - **Chức năng:** Trợ giúp hiển thị danh sách các lệnh và cú pháp sử dụng.

## Câu lệnh **`hailortcli measure-power`** là công cụ đo đạc công suất/dòng điện/điện áp độc lập của HailoRT (dành cho các bo mạch phần cứng Hailo EVB hoặc bo mạch có tích hợp IC đo nguồn).

Dưới đây là giải thích chi tiết ý nghĩa từng thông số của câu lệnh này:

---

### 1. Các thông số thời gian & lấy mẫu (Timing Options)

* **`--duration UINT`** *(Bắt buộc nếu dùng đo đạc)*:
  * *Ý nghĩa:* Thời gian thực hiện đo đạc (tính bằng giây).
  * *Ví dụ:* `--duration 10` sẽ lấy mẫu công suất liên tục trong 10 giây.

* **`--sampling-period UINT`** *(Mặc định: 1100 us)*:
  * *Ý nghĩa:* Chu kỳ lấy mẫu (tính bằng micro-giây - $\mu s$). Tần suất chip cảm biến đọc dữ liệu từ đường điện áp/dòng điện.
  * *Các mức hỗ trợ:* `140`, `204`, `332`, `588`, `1100`, `2116`, `4156`, `8244` ($\mu s$).

* **`--averaging-factor UINT`** *(Mặc định: 256)*:
  * *Ý nghĩa:* Hệ số gom mẫu tính trung bình. Cảm biến sẽ gom N mẫu lại để tính trung bình trước khi trả kết quả ra phần mềm.
  * *Các mức hỗ trợ:* `1`, `4`, `16`, `64`, `128`, `256`, `512`, `1024`. Giúp làm mịn biểu đồ, loại bỏ các đỉnh nhiễu điện áp tức thời.

---

### 2. Thông số đường nguồn đo đạc (`--dvm`)

* **`--dvm ENUM`** *(Digital Voltmeter Subsystem)*: Chọn đường cấp nguồn vật lý cụ thể trên chip mà bạn muốn đo:
  * **`AUTO`** *(Mặc định)*: Tự động tổng hợp các đường nguồn chính (`VDD_CORE` + `MIPI_AVDD` + `AVDD_H`) trên bo mạch EVB PCIe để ước tính tổng công suất chip tiêu thụ.
  * **`VDD_CORE`**: Đo riêng đường nguồn cấp cho **nhân tính toán NPU Core**.
  * **`VDD_IO`**: Đo riêng đường nguồn giao tiếp đầu vào/đầu ra I/O (PCIe/Host interface).
  * **`MIPI_AVDD` / `MIPI_AVDD_H`**: Đo đường nguồn cấp cho khối giao tiếp camera MIPI.
  * **`USB_AVDD_IO`**: Đo đường nguồn cấp cho khối giao tiếp USB.
  * **`OVERCURRENT_PROTECTION`**: Đường bảo vệ quá dòng (Thường dùng làm mặc định trên các bo mạch M.2/mPCIe hỗ trợ đo nguồn).

---

### 3. Loại chỉ số muốn đo (`--type`)

* **`--type ENUM`**: Chọn loại đơn vị đo vật lý xuất ra:
  * **`AUTO`** *(Mặc định)*: Tự chọn theo cấu hình DVM.
  * **`POWER`**: Đo **Công suất** tiêu thụ (đơn vị: **Watt - W** hoặc **mW**).
  * **`CURRENT`**: Đo **Dòng điện** tiêu thụ (đơn vị: **Ampere - A** hoặc **mA**).
  * **`BUS_VOLTAGE`**: Đo **Điện áp đường bus** (đơn vị: **Volt - V**).
  * **`SHUNT_VOLTAGE`**: Đo **Điện áp rơi trên điện trở mẫu Shunt** (đơn vị: **mV**).

---

### 4. Nhóm thông số thiết bị [Device Options]

* **`-s, --device-id TEXT`**: Chỉ định ID của chip Hailo cần đo (dùng `*` nếu muốn đo lần lượt từng chip trên hệ thống cắm nhiều card Hailo).
* **`--bdf TEXT`**: Chỉ định chip qua địa chỉ PCIe Bus/Device/Function.
* **`--ip TEXT`**: Địa chỉ IP của chip (nếu kết nối qua Ethernet).

---

### 💡 Ví dụ câu lệnh hay dùng (Dành cho bo mạch EVB hỗ trợ cảm biến):

```bash
# Measure total power consumption for 5 seconds
hailortcli measure-power --duration 5 --type POWER --dvm AUTO

# Measure current draw on the NPU Core rail for 10 seconds
hailortcli measure-power --duration 10 --type CURRENT --dvm VDD_CORE
```

## Subcommand **`set-net`** trong `hailortcli run2` cho phép bạn **cấu hình độc lập các tham số riêng cho từng mô hình (HEF)** khi chạy đa mô hình trên cùng 1 NPU.

Dưới đây là giải thích chi tiết ý nghĩa từng tham số của `set-net`:

---

### 1. Tham số cơ bản (Positionals & General Options)

* **`hef`** *(Truyền đường dẫn file)*: Đường dẫn tới file `.hef` của mô hình bạn muốn thiết lập.
* **`--name TEXT`**: Đặt tên định danh (Alias) cho nhóm mạng (Network Group Name). Giúp bạn phân biệt nếu nạp 2 file `.hef` giống nhau hoặc cần gọi tên nhóm mạng trong bộ lập lịch.

---

### 2. Nhóm tham số Lập lịch [Option Group: Network Group Parameters]
Nhóm này điều chỉnh cách **Model Scheduler** ưu tiên và phân chia thời gian chạy trên NPU cho riêng mô hình này:

* **`--batch-size UINT`** *(Mặc định: 0 - Tự động)*:
  * *Ý nghĩa:* Kích thước Batch Size áp dụng riêng cho model này.
  * *Trường hợp dùng:* Ví dụ Model 1 bạn muốn chạy Batch 1 để lấy Latency thấp, nhưng Model 2 bạn muốn ép Batch 4 để tối ưu Throughput tổng thể.

* **`--scheduler-priority UINT`** *(Mặc định: 16)*:
  * *Ý nghĩa:* **Độ ưu tiên** của mô hình khi xếp hàng chờ NPU xử lý. Số càng nhỏ đại diện cho mức độ ưu tiên càng cao.
  * *Trường hợp dùng:* Trong hệ thống xe tự hành, Model phát hiện vật cản nguy hiểm cần đặt ưu tiên cao (`--scheduler-priority 1`), còn Model nhận diện biển báo có thể đặt ưu tiên thấp hơn (`--scheduler-priority 16`).

* **`--scheduler-threshold UINT`** *(Mặc định: 0)*:
  * *Ý nghĩa:* Ngưỡng số lượng frame/batch tối thiểu nằm trong hàng chờ (queue) thì Model Scheduler mới tiến hành chuyển mạch NPU (Context Switch) cho model này chạy.
  * *Trường hợp dùng:* Giúp tránh việc NPU phải tốn công chuyển đổi context quá nhiều lần khi chỉ có 1-2 frame lẻ tẻ gửi tới.

* **`--scheduler-timeout UINT`** *(Mặc định: 0 ms)*:
  * *Ý nghĩa:* Thời gian chờ tối đa (tính bằng miligiây) trước khi **ép buộc** NPU phải kích hoạt model này chạy, kể cả khi hàng chờ chưa gom đủ số frame theo `scheduler-threshold`.
  * *Trường hợp dùng:* Đảm bảo model không bị hoãn quá lâu (Starvation) khi tốc độ ảnh gửi tới bị chậm.

---

### 3. Nhóm tham số Luồng chạy [Option Group: Run Parameters]

* **`--framerate UINT`** *(Mặc định: 0 - Chạy tối đa tốc độ)*:
  * *Ý nghĩa:* Giới hạn tốc độ khung hình (FPS) đầu vào cấp cho mô hình này.
  * *Trường hợp dùng:* Đặt `--framerate 30` để hãm tốc độ xử lý khớp với tốc độ 30 FPS của Camera thực tế, tránh việc NPU bị ngốn 100% tài nguyên dư thừa.

---

### 4. Các Subcommand con của `set-net`

* **`set-vstream`**: Dùng để cấu hình chi tiết cho từng luồng ảo ảo đầu vào/đầu ra (Virtual Stream), ví dụ: đổi kiểu dữ liệu `uint8`/`float32` hoặc thay đổi layout cho riêng 1 stream của model.
* **`set-stream`**: Dùng để cấu hình tầng thấp cho các kênh giao tiếp phần cứng vật lý (Physical HW streams).

---

### 💡 Ví dụ áp dụng thực tế:

Mô phỏng chạy 2 model: **YOLOv8-Seg** (Ưu tiên cao nhất, Batch 1) và **YOLO11n** (Giới hạn tốc độ camera 30 FPS, ưu tiên bình thường):

```bash
# Configure custom batch size, priority, and framerate for each model independently
hailortcli run2 \
    --scheduling-algorithm round_robin \
    -m full_async \
    --measure-temp \
    -t 15 \
    set-net /path/to/yolov8n-seg.hef --scheduler-priority 1 --batch-size 1 \
    set-net /path/to/yolo11n.hef --scheduler-priority 16 --framerate 30
```


## Subcommand **`measure-stats`** (Pipeline Statistics Measurements) dùng để **đo đạc và phân tích hiệu năng của TỪNG THÀNH PHẦN (Element) riêng lẻ** trong chuỗi luồng xử lý dữ liệu (Inference Pipeline) của HailoRT.

---

### 1. Tại sao lại cần đo từng Element trong Pipeline?

Khi một mô hình HEF chạy, luồng dữ liệu không đi thẳng 1 bước duy nhất mà đi qua một chuỗi các khâu (Pipeline Elements) nối tiếp nhau:
1. **Khâu đầu vào Host:** Đọc/Tạo buffer dữ liệu ảnh trên CPU/RAM.
2. **Khâu nạp PCIe:** Truyền dữ liệu từ RAM xuống NPU qua giao tiếp PCIe.
3. **Khâu thực thi NPU:** Nhân NPU Core thực hiện tính toán suy luận.
4. **Khâu rút kết quả PCIe:** Truyền tensor kết quả từ NPU ngược về RAM.
5. **Khâu đầu ra Host:** Nhận kết quả và chuyển tới ứng dụng người dùng.

Nếu ứng dụng chạy bị chậm (FPS thấp), `measure-stats` sẽ giúp bạn biết **chính xác khâu nào trong 5 khâu trên đang bị thắt cổ chai (bottleneck)**.

---

### 2. Hai chỉ số chính mà `measure-stats` đo đạc:

#### 1. `--elem-fps` (Tốc độ xử lý của từng khâu)
* **Đo cái gì:** Đo tốc độ (FPS) độc lập tại từng khâu trong chuỗi Pipeline.
* **Tác dụng:** Giúp bạn nhận diện khâu nào chạy chậm nhất. Ví dụ: Nếu khâu NPU đạt 100 FPS nhưng khâu truyền dữ liệu từ Host chỉ đạt 30 FPS, bạn biết ngay nguyên nhân tụt FPS là do CPU/giao tiếp PCIe chứ không phải do chip Hailo.

#### 2. `--elem-queue-size` (Độ dài hàng chờ / Bộ đệm của từng khâu)
* **Đo cái gì:** Đo số lượng frame/buffer đang bị dồn ứ (tồn đọng) trong hàng chờ (queue) giữa các khâu.
* **Tác dụng:** 
  * Nếu hàng chờ của một khâu bị **đầy (Full Queue)** $\rightarrow$ Khâu phía sau xử lý không kịp, làm dồn ứ dữ liệu từ khâu phía trước (Backpressure).
  * Nếu hàng chờ luôn **rỗng (Empty Queue)** $\rightarrow$ Khâu phía trước không cấp đủ dữ liệu cho khâu phía sau làm việc (Data Starvation).

#### 3. `--output-path` (Xuất báo cáo CSV)
* Mặc định lưu ra file `pipeline_stats.csv`. Bạn có thể mở file này bằng Excel/Python để vẽ biểu đồ sự biến động của FPS và Queue size theo thời gian.

---

### 3. Ví dụ câu lệnh chuẩn cách dùng `measure-stats`

Để dùng `measure-stats`, bạn cần truyền ít nhất một trong 2 cờ `--elem-fps` hoặc `--elem-queue-size` phía sau subcommand:

```bash
# Measure individual pipeline element FPS and queue size for bottleneck analysis
hailortcli run /home/duykhongcay/hailo_ws/chess_pieces_detection/runs/chessboard_yolo8n-seg/hailo_model/yolov8n-seg.hef \
    -t 10 \
    measure-stats \
    --elem-fps \
    --elem-queue-size \
    --output-path /home/duykhongcay/hailo_ws/pipeline_stats.csv
```

## Sự phân chia thành 2 nhóm **Device** và **VDevice** xuất phát từ kiến trúc quản lý phần cứng của **HailoRT (Hailo Runtime Framework)**, chia thành tầng Phần cứng vật lý (Low-level) và tầng Trừu tượng hóa phần mềm (High-level).

---

### 1. Sự khác nhau cốt lõi giữa Device và VDevice

* **Device (Physical Device - Thiết bị vật lý):**
  * Đại diện cho **con chip Hailo phần cứng thật 100%** đang cắm trên khe PCIe/mPCIe/USB (ví dụ 1 module Hailo-8 M.2 trên Raspberry Pi 5).
  * Tương tác ở tầng thấp (low-level). Khi tương tác trực tiếp với `Device`, ứng dụng của bạn sẽ độc chiếm con chip đó.

* **VDevice (Virtual Device - Thiết bị ảo hóa):**
  * Là **tầng phần mềm ảo hóa (Abstraction Layer)** nằm đè lên trên 1 hoặc nhiều chip phần cứng thật.
  * `VDevice` cung cấp các tính năng thông minh mà tầng vật lý không làm được:
    1. **Model Scheduler (Lập lịch đa mô hình):** Cho phép nạp nhiều model (.hef) vào cùng 1 chip và tự động chia sẻ thời gian chạy (Round-robin / Time-sharing).
    2. **Multi-Process Sharing (Chia sẻ đa tiến trình):** Cho phép nhiều ứng dụng Python/C++ độc lập chạy cùng lúc trên OS mà không bị tranh chấp phần cứng (thông qua dịch vụ `hailort-service`).
    3. **Multi-Device Load Balancing (Gộp nhiều chip):** Nếu bạn cắm 2 hay 4 card Hailo-8, VDevice có thể gom cả 4 card thành 1 "VDevice ảo duy nhất" để tự phân tải mà bạn không cần tự viết code chia luồng.

---

### 2. Ý nghĩa chi tiết từng thông số

#### A. Nhóm `Device Options` (Thao tác trực tiếp với Phần cứng)
Dùng khi bạn muốn chỉ định đích danh một phần cứng vật lý cụ thể để test:

* **`-s, --device-id TEXT`**: 
  * *Ý nghĩa:* ID định danh của thiết bị vật lý (ví dụ: `0001:04:00.0` lấy từ lệnh `hailortcli scan`).
  * *Trường hợp dùng:* Khi máy cắm 2 card Hailo và bạn muốn ép câu lệnh chỉ chạy trên card số 1.
* **`--bdf TEXT:BDF`**: 
  * *Ý nghĩa:* Định danh thiết bị theo địa chỉ khe cắm PCIe Bus/Device/Function (tương tự như trong lệnh `lspci` của Linux).
* **`--ip TEXT:IPV4`**: 
  * *Ý nghĩa:* Địa chỉ IP của thiết bị (Dùng trong trường hợp chip Hailo kết nối với máy tính qua mạng Ethernet chứ không qua khe cắm PCIe).

---

#### B. Nhóm `VDevice Options` (Thao tác qua Tầng Ảo hóa)
Dùng khi bạn làm việc ở tầng ứng dụng thực tế (Production, ROS2, Đa tiến trình, Đa mô hình):

* **`--device-count UINT`**: 
  * *Ý nghĩa:* Số lượng chip vật lý mà VDevice ảo này sẽ đại diện/gom lại.
  * *Mặc định:* `1`. 
  * *Trường hợp dùng:* Nếu hệ thống cắm 4 card Hailo và bạn đặt `--device-count 4`, HailoRT sẽ tạo 1 VDevice duy nhất và tự động chia đều các frame ảnh đầu vào cho 4 chip cùng xử lý để nhân 4 FPS.
* **`--multi-process-service`**: 
  * *Ý nghĩa:* Bật chế độ cho phép nhiều App độc lập truy cập NPU cùng lúc.
  * *Trường hợp dùng:* Bạn có App A (nhận diện khuôn mặt) và App B (nhận diện biển số xe) chạy ở 2 Terminal/Process khác nhau. Bật cờ này giúp cả 2 App gửi ảnh vào chip Hailo mà không bị lỗi "Device busy".
* **`--group-id TEXT`**: 
  * *Ý nghĩa:* Đặt tên/ID cho nhóm VDevice dùng chung.
  * *Trường hợp dùng:* Giúp các App khác nhau biết đường kết nối đúng vào nhóm VDevice ảo đã được tạo sẵn từ trước.

---

### Tóm tắt nhanh

| Tiêu chí | `Device Options` (Vật lý) | `VDevice Options` (Ảo hóa) |
| :--- | :--- | :--- |
| **Cấp độ quản lý** | Low-level (Trực tiếp chip phần cứng) | High-level (Qua bộ ảo hóa / Scheduler) |
| **Chạy nhiều Model cùng lúc**| ❌ Không hỗ trợ |  Hỗ trợ (Model Scheduler) |
| **Chia sẻ nhiều Tiến trình (App)**| ❌ Độc chiếm chip (Block app khác) |  Cho phép nhiều App cùng gửi data vào NPU |
| **Số lượng chip phần cứng** | Đúng 1 chip duy nhất | Có thể gom 1 hoặc N chip thành 1 VDevice |
| **Nên dùng khi nào?** | Khi benchmark 1 model, test driver | Khi làm ứng dụng thực tế (ROS 2, Multi-Camera, Multi-App) |

### Accuracy Analysis
Though ​most models work well with our default optimization, some suffer from high quantization noise that induces substantial accuracy degradation. As an example, we choose the MobileNet-v3-Large-Minimalistic neural network model that, due to its structural characteristics, results in a high degradation of 6% for Top-1 accuracy on the ImageNet-1K validation dataset.

To analyze the source of degradation, the Hailo `analyze_noise` API will be used. The analysis tool uses a given dataset to measure the noise level in each layer and allows to pinpoint problematic layers that should be handled. The analysis tool uses the entire dataset by default, use the `data_count` argument to limit the number of images.  
It is recommended to use at least 64 images, preferably not from the same calibration set, however, to keep the tool’s processing time to a reasonable level, it is also recommended not to use more than 100-200 images.

The following is equivalent to running the CLI command:
```bash
`hailo analyze-noise quantized_model_har_path 
    --data-path data_path 
    --batch-size 2 --data-count 16`
```

The output is saved inside the HAR, to be visualized later on by the Profiler.

## Dưới đây là hướng dẫn chi tiết cách đọc và hiểu log ở bước **Validating layers feasibility** (Kiểm tra tính khả thi của các layer) trong quá trình biên dịch/export model bằng Hailo Dataflow Compiler (DFC):

---

### 1. Ý nghĩa tổng quan của bước này
Trong kiến trúc của Hailo NPU, mô hình học máy (như YOLOv8n-seg) sẽ được biên dịch để chạy tối ưu trên các tài nguyên phần cứng (NPU clusters, SRAM, ALU,...). 

Trước khi thực sự phân bổ phần cứng (**Resource Allocation / Mapping**), Hailo DFC sẽ thực hiện bước **Validating layers feasibility** để:
* Kiểm tra từng layer trong mô hình (ví dụ: Conv, Add, Transpose, Concatenate, Sigmoid,...) xem có **tương thích với phần cứng Hailo** hay không.
* Xác nhận thông số của layer (Kernel size, Stride, Padding, Channels, Input/Output shape) có vượt quá giới hạn phần cứng cho phép hay không.

---

### 2. Chi tiết các thành phần trong Log của bạn

```text
Validating yolov8n-seg_context_1 layer by layer (100%)

 +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  + 
 +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  + 
 +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  + 
 +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  +  + 

● Finished                                                                 

[info] Layers feasibility validated successfully
```

#### Giải thích chi tiết:
1. **`Validating yolov8n-seg_context_1 layer by layer (100%)`**:
   * Hailo đang duyệt qua từng layer thuộc sub-graph/context 1 của mô hình `yolov8n-seg`.
2. **Ký hiệu dấu `+`**:
   * Mỗi dấu **`+`** đại diện cho **1 layer đã vượt qua kiểm tra tính khả thi thành công** (Hardware compatible).
   * Ma trận các dấu `+` biểu thị tất cả các layer được kiểm tra đều hợp lệ.
3. **`[info] Layers feasibility validated successfully`**:
   * **Kết quả**: Tất cả các layer trong mô hình đều đạt tiêu chuẩn phần cứng NPU của Hailo và sẵn sàng để chuyển sang bước phân bổ tài nguyên (**Resource Allocation / Mapping**).

---

### 3. Các trường hợp lỗi thường gặp ở bước này (Nếu có)

Nếu mô hình chứa các toán tử không tương thích hoặc tham số quá lớn, bạn sẽ thấy log thay đổi:
* **Ký hiệu `-` hoặc `x` / Warning**: Layer đó không thể chạy hoàn toàn hoặc không tối ưu trên NPU.
* **Thông báo Lỗi / Fallback**: 
  * `Unsupported layer type`: NPU chưa hỗ trợ toán tử này (thường xảy ra với các custom activation, custom NMS hoặc toán tử 3D/Transformer đặc thù).
  * `Dimension limit exceeded`: Kích thước tensor/input quá lớn so với dung lượng bộ nhớ đệm (SRAM) của phần cứng.
  * Khi đó, DFC sẽ đề xuất đưa layer đó về chạy trên CPU (Pre/Post-processing) hoặc yêu cầu bạn sửa lại file cấu hình `.alls` / mô hình ban đầu.

---

### 4. Phần log tiếp theo trong màn hình của bạn

Ngay sau khi **Layers feasibility validated successfully**, quá trình tự động chuyển sang bước tiếp theo:

```text
[info] Running resources allocation (mapping) flow, time per context: 59m 59s
Context:0/1 Iteration 4: Trying parallel mapping...  
          cluster_0  cluster_1  cluster_2  cluster_3  ...
 worker0  V          V          V          V          ...
```

* **Ý nghĩa**: Hailo đang xếp lịch (schedule) và gán (map) các layer vừa được xác nhận ở trên vào các khối tính toán thực tế (`cluster_0` tới `cluster_7`) trên chip NPU.
* **Ký hiệu `V`**: Đánh dấu worker đang phân bổ thành công các khối tính toán song song.


### 5. Ý nghĩa tổng quan của bảng Utilization
Sau khi hoàn thành phân bổ layer vào phần cứng, Hailo DFC đưa ra bảng thống kê hiệu suất sử dụng các tài nguyên phần cứng trên NPU (chip Hailo có 8 cụm xử lý độc lập từ `cluster_0` đến `cluster_7`).

Mô hình **YOLOv8n-seg** của bạn được chia thành **2 Context** (`context_0` và `context_1` - tương ứng với 2 sub-graph chạy nối tiếp nhau trên NPU).

---

### 6 Chi tiết các thông số trong bảng

| Tên cột | Ý nghĩa kỹ thuật | Đánh giá qua chỉ số của bạn |
| :--- | :--- | :--- |
| **Cluster** | Các cụm tính toán phần cứng của Hailo NPU (từ `cluster_0` tới `cluster_7`). | Chip sử dụng cả 8 cụm tính toán. |
| **Control Utilization** | Mức độ khai thác tài nguyên điều khiển & luồng dữ liệu (Interconnect, Data routing, Control units). | Có một số cluster đạt `100%` (như `cluster_4`, `cluster_6` ở context 0 và `cluster_2`, `cluster_4` ở context 1). Điều này nghĩa là đường truyền dữ liệu giữa các layer ở các cụm này đã được tận dụng tối đa. |
| **Compute Utilization** | Mức độ tận dụng các nhân tính toán toán học (MACs / ALUs - xử lý phép nhân ma trận, Convolution). | Context 0 trung bình **34%**, Context 1 trung bình **56.4%** (một số cluster như `cluster_4` đạt tới **98.4%**). Cho thấy khả năng tính toán của NPU được khai thác rất tốt. |
| **Memory Utilization** | Mức độ sử dụng bộ nhớ đệm SRAM nội bộ trong chip (dùng lưu weights và feature maps trung gian). | Trung bình **35.3%** (context 0) và **42.6%** (context 1). Bộ nhớ SRAM còn thừa nhiều không lo bị tràn bộ nhớ. |

---

### 7. Ý nghĩa dòng `Successful Mapping (allocation time: 8m 34s)`

* **`Successful Mapping`**: Hailo Compiler đã giải xong bài toán phân bổ tài nguyên phần cứng (Resource Allocation problem) thành công! Mô hình hoàn toàn vừa vặn và chạy tối ưu trên NPU mà không bị quá tải hay thiếu tài nguyên.
* **`allocation time: 8m 34s`**: Thuật toán tối ưu hóa của Hailo đã mất 8 phút 34 giây để tìm ra phương án đi dây (routing) và xếp lịch (scheduling) tốt nhất cho mô hình này.

---

### 10. Đánh giá chung về mô hình YOLOv8n-seg của bạn
1. **Tài nguyên phần cứng dư dả**: Mức sử dụng bộ nhớ (Memory ~35-42%) và tính toán (Compute ~34-56%) rất an toàn.
2. **Sẵn sàng tạo file `.hef`**: Ngay sau bước này, quá trình export sẽ đóng gói thành công file `.hef` (Hailo Executable Format) để bạn có thể nạp vào thiết bị chạy inference thực tế.