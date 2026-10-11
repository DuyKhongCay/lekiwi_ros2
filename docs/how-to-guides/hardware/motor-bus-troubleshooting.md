# How-To: Motor Bus Fault Recovery & Troubleshooting

This guide explains how to diagnose, recover from, and prevent communication and hardware faults on the 9-servo Feetech STS3215 serial bus.

---

## Common Failure Symptoms

| Symptom | Probable Cause | Diagnostic Command |
| :--- | :--- | :--- |
| **`Timeout waiting for response from ID X`** | Loose daisy-chain wire or baudrate mismatch | Check `ros2 control list_hardware_interfaces` |
| **Robot arm loses stiffness / collapses** | Voltage drop below 6.5V or thermal protection triggered | Inspect `voltage` and `temperature` on `/diagnostics` |
| **`Device /dev/lekiwi_serial busy or not found`** | Serial port locked by prior crashed node or missing udev rule | `lsof /dev/lekiwi_serial` |

---

## 1. Inspect Hardware Component State & Reset

The Feetech bus worker in `lekiwi_ftservo_hardware` runs an asynchronous background I/O thread (`feetech_bus_worker.cpp`) that automatically attempts silent reconnection on sporadic packet drops.

To verify hardware interface status:

```bash
ros2 control list_hardware_components
```

**Expected Output:**
```text
Hardware Component 1
    name: lekiwi_ftservo_hardware
    type: system
    plugin: lekiwi_ftservo_hardware/LekiwiFeetechHardware
    state: id=3 label=active
```

If the hardware interface entered an error state, re-activate it through the lifecycle controller manager:

```bash
ros2 control set_hardware_component_state lekiwi_ftservo_hardware inactive
ros2 control set_hardware_component_state lekiwi_ftservo_hardware active
```

---

## 2. Emergency Torque Release & Manual Positioning

To move the arm manually (e.g., for lead-through teaching, calibration, or safely parking the arm):

```bash
# Disable holding torque across all joints (target: 0 = TARGET_ALL)
ros2 service call /set_torque_enabled lekiwi_interfaces/srv/SetTorqueEnabled "{target: 0, enabled: false, toggle: false}"

# Re-enable holding torque when positioning is complete
ros2 service call /set_torque_enabled lekiwi_interfaces/srv/SetTorqueEnabled "{target: 0, enabled: true, toggle: false}"
```

> [!TIP]
> To disable torque on the arm servos only while keeping the mobile base holding position, pass `target: 1` (`TARGET_ARM`).

---

## 3. Bus Health & Diagnostics Monitoring

Launch the diagnostic monitor node to stream real-time voltage, temperatures, and communication error rates:

```bash
ros2 launch lekiwi_bringup diagnostics.launch.py
```

Echo the diagnostic stream to verify all 9 servos are reporting normal health:

```bash
ros2 topic echo /diagnostics
```

Look for key health fields:
- `Bus Status`: `OK`
- `Supply Voltage`: `7.2V - 8.2V` (Nominal 2S LiPo)
- `Motor Temperatures`: `< 55°C`
