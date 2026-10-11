# How-To: Deploy ZeroMQ Policy Server

This guide shows how to deploy the high-performance ZeroMQ policy inference server on a workstation and connect the LeKiwi robot client over a local network.

---

## 🌐 Network & Deployment Topology

The physical deployment topology connects the GPU training/inference workstation with the Raspberry Pi 5 onboard computer via a low-latency 5GHz wireless access point:

```mermaid
flowchart LR
    subgraph HostStation ["Host Workstation / Cloud Instance"]
        direction TB
        subgraph HostOS ["Ubuntu 22.04 / 24.04 (Host)"]
            NvidiaDriver["NVIDIA Driver 550+ / CUDA 12.x"]
            DockerHost["Docker Container: lerobot-server"]
            subgraph ServerProc ["Container Runtime"]
                PolicyServer["lerobot-policy-server\n(PyTorch, LeRobot, ZMQ REP:8090)"]
            end
        end
    end

    subgraph NetworkLink ["Local Area Network (LAN)"]
        WiFi["Wi-Fi 6 (5GHz) / Dedicated AP\nLatency: 2-8ms, Jitter: < 5ms\nPayload: Compressed JPEG + JSON/msgpack"]
    end

    subgraph RobotHardware ["LeKiwi Mobile Manipulator (Raspberry Pi 5)"]
        direction TB
        subgraph Pi5Host ["Raspberry Pi OS / Ubuntu 24.04 (arm64)"]
            HailoDriver["hailo_pci kernel module (PCIe Gen2/3 x1)"]
            SerialPort["/dev/lekiwi_serial (1,000,000 baud)"]
            
            subgraph Pi5Container ["Docker Container: lekiwi-ros2-hailort"]
                ROS2Middleware["ROS 2 Jazzy Base (RMW Zenoh / FastDDS)"]
                
                subgraph NodesRunning ["Active ROS 2 Nodes"]
                    NodeInference["lekiwi_inference\n(async_ros2_client_node OR hailo_act_node)"]
                    NodeControl["controller_manager\n(arm_forward_controller)"]
                    NodeHW["lekiwi_ftservo_hardware\n(Async Serial Worker)"]
                    NodeVision["lekiwi_perception\n(GStreamer + HailoRT)"]
                end
            end
        end

        subgraph PhysicalPeripherals ["Peripherals & Actuators"]
            NPU["Hailo-8 M.2 Module\n(26 TOPS INT8, M.2 HAT+)"]
            Servos["6x Feetech STS3215 Servos\n(Arm Joints & Gripper)"]
            Cams["USB/CSI Cameras\n(Wrist Cam & Wide Cam)"]
        end
    end

    PolicyServer ---|"TCP Socket :8090"| WiFi
    WiFi ---|"ZMQ REQ"| NodeInference

    HailoDriver ---|"PCIe Link"| NPU
    SerialPort ---|"UART 1Mbps"| Servos
    Cams --> NodeVision
    NodeInference --> NodeControl
    NodeControl --> NodeHW
    NodeHW --> SerialPort
    NodeInference -.->|"HailoRT API (Phase 3)"| HailoDriver
```

---

## 1. Launch the Server on GPU Host

On your GPU workstation (NVIDIA RTX with CUDA):

```bash
cd ~/lekiwi_ws/src/lekiwi_ros2/lerobot_policy_server
python3 -m lerobot_policy_server \
    --host "0.0.0.0" \
    --port 8090 \
    --fps 30 \
    --device cuda
```

### Protocol Specifications:
- **Transport**: ZeroMQ REQ / REP (`tcp://0.0.0.0:8090`).
- **Serialization**: `msgpack` with binary ndarray buffers.
- **Payload**: Dual JPEG images (head + wrist) + 6-DoF joint state array → Returns 50-step predicted action chunk.

---

## 2. Launch Client on LeKiwi Robot

On the Raspberry Pi 5 onboard the robot:

```bash
source ~/lekiwi_ws/install/setup.bash
ros2 launch lekiwi_inference async_inference.launch.py \
    server_address:="192.168.1.50:8090"
```

---

## 3. Latency & Watchdog Diagnostics

Monitor round-trip inference latency:

```bash
ros2 topic echo /lekiwi_inference/diagnostics
```

**Key Metrics to Monitor:**
- `network_rtt_ms`: Target < 15 ms over 5GHz Wi-Fi.
- `gpu_inference_ms`: Target < 12 ms on RTX 4080/4090.
- `action_queue_depth`: Nominal: 25–45 actions queued.
