# Architecture & Explanation: Deep Insights

The Explanation section provides **understanding-oriented** discussions. It answers "why" the system is designed this way, explores theoretical foundations, compares architectural tradeoffs, and documents significant architectural decisions.

---

## 🏛️ Explanatory Articles

- [**System Architecture & Subsystems**](system-architecture.md): Complete C4 architectural representation, component decomposition, and inter-process communication topology.
- [**Physical-AI Inference Strategy**](physical-ai-inference-strategy.md): The two-stage evolution from remote GPU ZeroMQ policy serving to onboard Hailo-8 M.2 NPU INT8 acceleration.
- [**Hardware Control Loop & STS3215 Bus**](hardware-control-loop.md): How `lekiwi_ftservo_hardware` isolates real-time 50Hz `ros2_control` execution from non-blocking serial bus I/O.
- [**Decision FSM & Stockfish Engine**](decision-and-fsm.md): Lifecycle state transitions, chess move evaluation, and verification gating before physical execution.
- [**Architecture Decision Records (ADR Index)**](adr/index.md): Catalog of immutable records documenting critical system engineering decisions.

