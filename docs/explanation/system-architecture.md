# System Architecture

This article describes the holistic system architecture of the LeKiwi autonomous mobile manipulation platform.

---

## 🌐 System Context (C4 Level 1)

The system context diagram illustrates LeKiwi operating within its physical workcell, interacting with physical manipulation objects, human operators, and remote compute infrastructure:

```mermaid
flowchart TD
    subgraph Environment ["Physical & Operational Environment"]
        TargetObj["Manipulation Targets\n(Chess pieces, chess board)"]
        SceneLight["Workcell Environment & Lighting"]
    end

    subgraph Robot ["LeKiwi Robot Platform (Mobile Manipulator)"]
        Sensors["Cameras (Stereo / Wrist / Side) & Servos (STS3215)"]
        Actuators["6-DoF Arm Forward Controller & Omni Base"]
        EdgeCompute["Raspberry Pi 5 + Hailo-8 M.2 NPU (26 TOPS)"]
    end

    subgraph HostStation ["Remote Host Workstation / Cloud"]
        GPUWorkstation["Workstation (NVIDIA RTX GPU / CUDA)"]
        PolicyRegistry["HuggingFace Hub / Local Policy Weights"]
    end

    subgraph Operators ["Developers & Mission Control"]
        Orchestrator["LeKiwi FSM Orchestrator"]
        TeleopStation["Teleop / Gamepad Station"]
    end

    Sensors -->|"Photons & Feedback"| TargetObj
    EdgeCompute -->|"Joint Position Commands 50Hz"| Actuators
    Actuators -->|"Physical Interaction"| TargetObj

    EdgeCompute -->|"ZMQ REQ-REP (Phase 1)"| GPUWorkstation
    GPUWorkstation -->|"Action Chunk Stream"| EdgeCompute
    GPUWorkstation -->|"Load Checkpoint"| PolicyRegistry

    Orchestrator -->|"Mode Triggers"| EdgeCompute
    TeleopStation -->|"Safety E-Stop / Override"| EdgeCompute
```

---

## 🧩 Component Decomposition (C4 Level 2)

LeKiwi's software architecture enforces clean boundaries between **Perception**, **Decision**, **Planning**, and **Hardware Abstraction**:

```mermaid
flowchart TD
    subgraph SENSORS["Perception & Sensors"]
        CAM["Dual CSI Cameras (IMX219 / IMX708)"] -->|"GStreamer / v4l2"| PERCEPT["lekiwi_perception\n(Camera Hub & Lifecycle)"]
        PERCEPT -->|"HailoRT / NPU"| YOLO["YOLO Chessboard & Piece Detection"]
        IMU_DEV["ICM-20948 (I2C /dev/i2c-1)"] --> IMU_HW["lekiwi_icm20948_hardware\n(ros2_control SensorInterface)"]
    end

    subgraph DECISION["Decision & Orchestration"]
        YOLO -->|"FEN String / State"| CHESS["lekiwi_chess_master\n(Stockfish 16 Engine Node)"]
        CHESS -->|"ComputeBestMove Action"| ORCH["lekiwi_orchestrator\n(Mission FSM & Readiness Gate)"]
        ORCH -->|"/perception/set_context"| PERCEPT
        ORCH -->|"CheckMoveFeasibility Srv"| MOTION["lekiwi_motion\n(Workspace IK & Feasibility Checker)"]
    end

    subgraph PLANNING["Planning & Policy Serving"]
        ORCH -->|"ExecuteChessMove Action"| RECORDER["chess_episode_recorder\n(Action Server & MCAP)"]
        INFER["lekiwi_inference\n(Async / Edge Client Node)"] <-->|"ZeroMQ REQ/REP"| POL_SRV["lerobot_policy_server\n(SmolVLA / ACT / Diffusion Policy)"]
    end

    subgraph CONTROL["Execution & Hardware Control"]
        INFER -->|"arm_forward_controller (50Hz)"| CM["ros2_control Controller Manager\n(Joint Trajectory & Forward Control)"]
        MOTION -->|"arm_trajectory_controller"| CM
        CM --> FT_HW["lekiwi_ftservo_hardware\n(SystemInterface - 1 Mbps Serial Bus)"]
        FT_HW -->|"Async Serial I/O /dev/lekiwi_serial"| SERVOS["9x Feetech STS3215 Servos\n(6-DoF Arm + 3-Wheel Omni Base)"]
        IMU_HW -->|"/imu/data_raw"| CM
    end
```

---

## 🔍 Inference Subsystem Components (C4 Level 3)

Zooming into the `lekiwi_inference` module demonstrates the dual-phase implementation isolating safety, transport, and hardware execution:

```mermaid
flowchart TB
    subgraph LeKiwiInferencePackage ["Package: lekiwi_inference"]
        direction TB
        
        subgraph ModeAsync ["Phase 1: Async Inference Engine"]
            AsyncNode["async_ros2_client_node\n(ROS 2 Jazzy Lifecycle / Node)"]
            ClientCore["AsyncInferenceClient\n(Action Queue + Aggregator)"]
            TransportZMQ["ZmqTransportClient\n(msgpack binary wire protocol)"]
            Watchdog["Safety Watchdog & Timeout Arbiter"]
        end

        subgraph ModeEdge ["Phase 3: Edge NPU Inference Engine"]
            EdgeNode["hailo_act_inference_node\n(Zero-latency local executor)"]
            HailoRTDriver["HailoRT Runtime Engine\n(VDevice / ConfiguredNetworkGroup)"]
            QuantHEF["act_policy.hef\n(INT8 Quantized ACT Model)"]
            LocalSmoother["Temporal Action Chunk Smoother"]
        end

        ConfigManager["Inference Config Manager\n(YAML Params & Dynamic Reconfigure)"]
    end

    subgraph RemoteServer ["Package: lerobot_policy_server (GPU Workstation)"]
        ServerCore["ZMQ REP Policy Server"]
        PyTorchEngine["PyTorch LeRobot Engine\n(ACT / SmolVLA / Diffusion)"]
    end

    subgraph HardwareAndROS ["Robot System (Pi 5 Hardware & ros2_control)"]
        Cameras["lekiwi_perception\n(/cameras/usb_wrist, /cameras/usb_side)"]
        Joints["lekiwi_ftservo_hardware\n(/joint_states)"]
        Controller["arm_forward_controller\n(/arm_forward_controller/commands 50Hz)"]
        FSM["lekiwi_orchestrator\n(Mission FSM / State Transitions)"]
    end

    %% Wiring Phase 1
    Cameras -->|"sensor_msgs/CompressedImage"| AsyncNode
    Joints -->|"sensor_msgs/JointState"| AsyncNode
    AsyncNode --> ClientCore
    ClientCore --> TransportZMQ
    ClientCore --> Watchdog
    Watchdog -->|"Emergency Hold / Ramp-down"| Controller
    ClientCore -->|"Float64MultiArray 50Hz"| Controller

    TransportZMQ -->|"TCP ZMQ Request"| ServerCore
    ServerCore -->|"Action Reply"| TransportZMQ
    ServerCore --> PyTorchEngine

    %% Wiring Phase 3
    Cameras -.->|"Zero-Copy / Shared Memory cv_bridge"| EdgeNode
    Joints -.->|"JointState"| EdgeNode
    EdgeNode --> HailoRTDriver
    HailoRTDriver --> QuantHEF
    HailoRTDriver --> LocalSmoother
    LocalSmoother -.->|"Float64MultiArray 50Hz"| Controller

    FSM -->|"Start/Stop/Reset Actions"| AsyncNode
    FSM -.->|"Switch Mode"| EdgeNode
```

---

## 🔄 Inter-Process Communication Patterns

1. **Deterministic Real-Time Motor Loop (50 Hz)**:
   - Synchronized by `ros2_control` update cycle.
   - Decoupled from serial blocking delays via dedicated asynchronous worker threads in `lekiwi_ftservo_hardware`.
2. **High-Throughput Visual AI Pipeline**:
   - Camera images are captured via V4L2/GStreamer and processed through zero-copy HailoRT buffers for object detection.
   - For remote policy serving, frames are JPEG-compressed to preserve wireless bandwidth.
3. **Discrete Decision & Orchestration**:
   - Long-running engine analysis and trajectory executions use **ROS 2 Actions**, providing non-blocking execution, cancelability, and periodic progress feedback.

