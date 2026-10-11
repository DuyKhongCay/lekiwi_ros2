# How-To: Stereo Camera & Hand-Eye Calibration

This guide describes how to run intrinsic chessboard calibration and hand-eye calibration using the `lekiwi_calibration` package to register camera frames to the robot's base coordinate frame.

---

## 1. Chessboard Calibration Suite

LeKiwi includes an automated OpenCV/ChArUco chessboard calibrator node to compute camera matrices and chessboard corner poses:

```bash
source ~/lekiwi_ws/install/setup.bash
ros2 launch lekiwi_calibration calibrate_chessboard.launch.py
```

The node captures synchronized frames and outputs calibration parameters directly to the configuration folder.

---

## 2. Hand-Eye Calibration (Eye-to-Base)

LeKiwi mounts its stereo camera on the head chassis overlooking the arm workspace (Eye-on-Base configuration).

### Procedure:
1. Fasten a ChArUco / AprilTag pattern rigidly to the robot gripper tip (`gripperframe`).
2. Run the automated hand-eye calibration suite:

```bash
ros2 launch lekiwi_calibration calibrate_handeye.launch.py
```

3. Move the arm across sample positions covering rotation and translation. At each point, the calibrator samples:
   - $T_{\text{base}}^{\text{gripper}}$ (from `/tf` via forward kinematics).
   - $T_{\text{camera}}^{\text{tag}}$ (from visual detection).

4. The node solves the $AX = YB$ hand-eye transformation using Park-Martin/Tsai-Lenz and writes the resulting calibration matrix to:
   `$(ros2 pkg prefix lekiwi_description)/share/lekiwi_description/config/calibration/handeye_calib_stereo_left.yaml`

---

## 3. Verification

Restart the robot state publisher and verify the transform between `baseframe` and `camera_stereo_left_optical_frame`:

```bash
ros2 run tf2_ros tf2_echo baseframe camera_stereo_left_optical_frame
```
