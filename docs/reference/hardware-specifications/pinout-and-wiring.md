# Hardware Specifications & Wiring Pinout

This reference document outlines the physical wiring, bus topologies, and electrical parameters of the LeKiwi robot platform.

---

## ⚡ Power Distribution Architecture

| Subsystem | Input Voltage | Max Current | Source |
| :--- | :--- | :--- | :--- |
| **Compute Core (Pi 5 + Hailo-8)** | 5.1V DC | 5.0A | Dedicated Buck Converter (USB-C PD) |
| **Servo Bus (9x STS3215)** | 7.4V (2S LiPo / 8.4V Peak) | 12.0A Surge | Direct High-Current Battery Rail |
| **Cameras & Sensors** | 3.3V / 5.0V | 1.5A | Raspberry Pi 5 onboard regulators |

---

## 🚌 Bus & Port Mappings

```mermaid
flowchart TD
    PI5["Raspberry Pi 5\n(Ubuntu 24.04 ARM64)"]
    
    PI5 -->|"PCIe Gen 2/3 x1 HAT+"| HAILO["Hailo-8 M.2 Key M\n(26 TOPS INT8 NPU)"]
    PI5 -->|"I2C /dev/i2c-1 (Pins 3, 5)"| IMU["ICM-20948 9-DoF IMU\n(Address: 0x68)"]
    PI5 -->|"CSI-2 Port 0 & 1"| DUAL_CAM["Dual CSI Cameras\n(IMX219 / IMX708)"]
    PI5 -->|"USB 3.0 (/dev/lekiwi_serial)"| FT_BRIDGE["USB-to-Half-Duplex-UART\n(1,000,000 baud)"]
    
    FT_BRIDGE --> SERVO_BUS["Feetech STS3215 Daisy Chain\n(IDs: 1 to 9)"]
```

### Servo Bus ID Allocation:
- **IDs 1–6**: 6-DoF Arm articulation (Base pan, Shoulder, Elbow, Wrist pitch, Wrist roll, Gripper clamp).
- **IDs 7–9**: 3-Wheel Omnidirectional drive base (Continuous rotation velocity control mode).

