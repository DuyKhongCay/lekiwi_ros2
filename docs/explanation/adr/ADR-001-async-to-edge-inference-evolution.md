# ADR-001: Two-Stage Evolution from Remote Async Inference to On-Board Hailo-8 Edge Inference

## Status
Accepted

## Context
The LeKiwi physical AI robot requires running deep imitation learning policies (specifically ACT - Action Chunking with Transformers) to perform complex manipulation tasks such as chess piece picking, placing, and sorting.

We evaluated two deployment paths:
1. Running PyTorch ACT directly on the on-board compute (Raspberry Pi 5 CPU).
2. Remote GPU inference over ZeroMQ vs. on-board Hailo-8 M.2 NPU (26 TOPS INT8).

Benchmark findings:
- Raspberry Pi 5 quad-core Cortex-A76 running PyTorch ACT with dual ResNet18 visual backbones achieves only 3–5 Hz inference with 100% CPU saturation and severe thermal throttling.
- `so101_inference/lerobot_inference_node.py` uses direct PyTorch CPU/CUDA execution, which is unviable on-board Raspberry Pi 5 without hardware acceleration.
- Attempting to quantize ACT to Hailo-8 immediately introduces high debugging friction: model export (ONNX with dynamic vs. static shapes), cross-attention layer mapping in Hailo Dataflow Compiler (DFC), and INT8 quantization calibration.

```mermaid
pie title "Resource & Compute Latency Breakdown (Phase 1 vs Phase 3)"
    "Camera Ingest & Encoding" : 12
    "Wi-Fi Transmission (Phase 1 Only)" : 30
    "GPU Model Compute (Phase 1)" : 28
    "Hailo-8 INT8 NPU Compute (Phase 3)" : 15
    "Action Queue Interpolation & ROS 2 Publish" : 15
```

## Decision
We adopt a disciplined, phased development lifecycle:

```mermaid
flowchart LR
    subgraph Step1 ["Phase 1: Async Remote Inference (MVP)"]
        direction TB
        P1_Srv["Host GPU Policy Server\n(FP16 PyTorch ACT)"]
        P1_Net["ZMQ over 5GHz Wi-Fi\n(JPEG Images + Actions)"]
        P1_Rob["LeKiwi Async Client\n(Action Queue & Temporal Smoothing)"]
        P1_Srv --- P1_Net --- P1_Rob
        P1_Note["Goal: Rapid policy validation,\nzero quantization barrier,\neasy training-to-deploy loop."]
    end

    subgraph Step2 ["Phase 2: Hardware-in-the-Loop & Quantization Pipeline"]
        direction TB
        P2_HIL["Real-robot episode collection & telemetry verification"]
        P2_DFC["Model Export & Hailo DFC Calibration\n(ONNX -> HAR -> HEF INT8)"]
        P2_Bench["Accuracy vs Latency Benchmarking (CosSim >= 0.98)"]
        P2_HIL --> P2_DFC --> P2_Bench
        P2_Note["Goal: Validate INT8 accuracy,\nresolve transformer op constraints,\nprepare standalone HEF."]
    end

    subgraph Step3 ["Phase 3: Production Edge NPU Inference"]
        direction TB
        P3_NPU["On-board Hailo-8 M.2 NPU\n(26 TOPS, <2.5W, PCIe)"]
        P3_Node["hailo_act_inference_node\n(Zero-latency local executor)"]
        P3_Perf["Deterministic 50Hz Execution\n(Sub-15ms, 100% offline, zero drop)"]
        P3_NPU --- P3_Node --- P3_Perf
        P3_Note["Goal: True robot autonomy,\nimmunity to Wi-Fi drops,\nproduction deployment."]
    end

    Step1 ==>|System Proven & Deployed| Step2
    Step2 ==>|HEF Compiled & Verified| Step3
```

1. **Phase 1: Async Remote Policy Inference (`async_inference`)**:
   - Host GPU / Cloud workstation runs `lerobot_policy_server` (ZeroMQ binary protocol with msgpack + raw ndarray buffers).
   - LeKiwi robot runs `lekiwi_inference/async_ros2_client_node`.
   - Communication over 5GHz Wi-Fi using `sensor_msgs/CompressedImage` to conserve wireless bandwidth.
   - Deterministic 50Hz action publisher decoupled from network round-trip via a thread-safe sliding action queue with temporal moving-average aggregation (`weighted_average`).
   - Hardened with a Safety Watchdog that ramps down or holds position if network latency exceeds `max_age_s`.

2. **Phase 2: Validation, Telemetry & Dataset Grounding**:
   - Validate policy execution on physical LeKiwi robot with `arm_forward_controller` (`/arm_forward_controller/commands`).
   - Record evaluation episodes and calibration datasets for NPU quantization.

3. **Phase 3: Edge NPU Inference (`edge_inference`) on Hailo-8**:
   - Quantize the validated ACT policy to INT8 using Hailo DFC.
   - Build `hailo_act_inference_node` in `lekiwi_inference` using HailoRT (Python / C++ API).
   - Ingest intra-process camera frames directly from `lekiwi_perception`, achieving zero network dependency, deterministic < 15ms total latency, and fully untethered autonomous operation.

## Consequences
- **Positive**:
  - Immediate turnaround: LeRobot models trained on HuggingFace can be deployed and validated on the physical robot in Phase 1 within minutes without waiting for quantization compilation.
  - Fail-safe design: The action queue decouples the 50Hz motor loop from inference jitters.
  - Smooth migration: The ROS 2 control topic, joint conventions, and safety watchdog remain identical between Phase 1 and Phase 3.
- **Negative / Constraints**:
  - Phase 1 requires a dedicated 5GHz Wi-Fi router close to the workcell to prevent dropped frames.
  - Phase 3 requires careful ONNX export and INT8 calibration to avoid catastrophic accuracy loss on the Transformer regression head.

