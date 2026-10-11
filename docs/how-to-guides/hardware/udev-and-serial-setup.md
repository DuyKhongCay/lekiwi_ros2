# How-To: Udev Rules & Serial Bus Setup

This guide details how to install and verify persistent Linux udev device permissions for the LeKiwi serial bus and peripherals.

---

## The Problem
When connecting USB-to-UART serial adapters and cameras on Linux, the kernel dynamically assigns device names such as `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `/dev/video0`. In a multi-device system with servos, teleoperation arms, cameras, and IMU interfaces, these assignments vary across reboots and require specific user group permissions (`dialout`, `video`, `i2c`).

---

## 1. Install Udev Rule Files

Copy the provided rule files from the LeKiwi workspace to `/etc/udev/rules.d/`:

```bash
cd ~/lekiwi_ws/src/lekiwi_ros2
sudo cp lekiwi_bringup/udev/99-lekiwi.rules /etc/udev/rules.d/
sudo cp teleop_zhongli_servo_hw/udev/99-teleop-lekiwi.rules /etc/udev/rules.d/
```

Add your user to the hardware access groups:

```bash
sudo usermod -aG dialout,video,i2c $USER
```

---

## 2. Rule Definitions Explained

The installed rules match persistent hardware attributes (serial numbers and vendor/product IDs):

### A. Main Robot Peripherals (`lekiwi_bringup/udev/99-lekiwi.rules`):
```udev
# USB serial adapter for Feetech STS3215 bus (matched by factory serial number)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="5B14033437", SYMLINK+="lekiwi_serial", MODE="0660", GROUP="dialout"

# UVC Cameras (Wrist & Side)
SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{idVendor}=="4c4a", ATTRS{idProduct}=="4a55", SYMLINK+="usb_wrist", MODE="0660", GROUP="video"
SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{idVendor}=="1bcf", ATTRS{idProduct}=="2cd1", SYMLINK+="usb_side", MODE="0660", GROUP="video"

# Gamepad (Xbox 360 wireless dongle)
SUBSYSTEM=="input", KERNEL=="js[0-9]*", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", ATTRS{serial}=="5F6E9C98", SYMLINK+="input/gamepad", SYMLINK+="gamepad", MODE="0660", GROUP="input"
SUBSYSTEM=="input", KERNEL=="event[0-9]*", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", ATTRS{serial}=="5F6E9C98", SYMLINK+="input/gamepad_event", MODE="0660", GROUP="input"

# ICM-20948 I2C interface permissions
KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"
KERNEL=="i2c-1", SYMLINK+="i2c-imu", GROUP="i2c", MODE="0660"
```

### B. Teleoperation Leader Arm (`teleop_zhongli_servo_hw/udev/99-teleop-lekiwi.rules`):
```udev
# Prolific PL2303 USB-to-Serial adapter for uArm Leader
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="ttyUSB_PROLIFIC", SYMLINK+="uarm_leader", MODE="0666"
```

---

## 3. Reload and Trigger Udev Rules

Apply the new rules immediately without restarting the host:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 4. Verification

Verify that the symlinks point to the active tty device nodes:

```bash
ls -l /dev/lekiwi_serial /dev/uarm_leader /dev/i2c-imu
```

**Expected Output:**
```text
lrwxrwxrwx 1 root root 7 Oct 11 08:30 /dev/lekiwi_serial -> ttyUSB0
lrwxrwxrwx 1 root root 7 Oct 11 08:30 /dev/uarm_leader -> ttyUSB1
lrwxrwxrwx 1 root root 5 Oct 11 08:30 /dev/i2c-imu -> i2c-1
```

Confirm read and write permissions:

```bash
test -r /dev/lekiwi_serial && test -w /dev/lekiwi_serial && echo "SUCCESS: /dev/lekiwi_serial is accessible"
```
