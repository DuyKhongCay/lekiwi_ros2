/**
 * @file pose_solver.hpp
 * @brief Core geometry and multi-tag PnP solver for AprilTag chessboard localization.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef APRILTAG_LOCALIZER__POSE_SOLVER_HPP_
#define APRILTAG_LOCALIZER__POSE_SOLVER_HPP_

#include <map>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace apriltag_localizer
{

  /**
   * @brief Configuration for a single AprilTag mounted on the chessboard.
   */
  struct TagConfig
  {
    int id{0};
    std::string name;
    cv::Point3d center{0.0, 0.0, 0.0};
    double yaw{0.0};
    std::vector<cv::Point3d> corners_board;
  };

  /**
   * @brief Pure algorithmic helper functions for pose estimation and geometric calculations.
   */
  class PoseSolver
  {
  public:
    /**
     * @brief Computes 4 3D corner coordinates of a square tag in chessboard coordinates.
     * Corner 0: Bottom-Right [ h, -h, 0]
     * Corner 1: Bottom-Left  [-h, -h, 0]
     * Corner 2: Top-Left     [-h,  h, 0]
     * Corner 3: Top-Right    [ h,  h, 0]
     *
     * @param[in] center 3D center in chessboard_frame.
     * @param[in] size Edge length in meters (e.g. 0.02).
     * @param[in] yaw_rad In-plane rotation around Z axis in radians. Default is 0.0.
     * @return 4 3D points ordered matching ArUco.
     */
    static std::vector<cv::Point3d> compute_tag_corners(
        const cv::Point3d &center, double size, double yaw_rad = 0.0);

    /**
     * @brief Estimates chessboard pose T_cam^board using all detected board tags jointly (Global Multi-Tag PnP).
     * Requires at least 2 detected board tags. Returns false if tag count < 2 or estimation fails.
     *
     * @param[in] marker_corners Detected 2D marker corners from ArucoDetector.
     * @param[in] marker_ids Detected marker IDs.
     * @param[in] tag_configs Map of configured tags with their fixed 3D layout on the board.
     * @param[in] camera_mat 3x3 intrinsic camera matrix.
     * @param[in] dist_coeffs Camera distortion coefficients vector.
     * @param[out] rvec 3x1 Rodrigues rotation vector of the board in camera optical frame.
     * @param[out] tvec 3x1 translation vector of the board in camera optical frame.
     * @param[out] used_tags_cnt Number of valid board tags used (must be >= 2).
     * @return True if estimation succeeds with >= 2 tags, false otherwise.
     */
    static bool estimate_board_pose(
        const std::vector<std::vector<cv::Point2f>> &marker_corners,
        const std::vector<int> &marker_ids,
        const std::map<int, TagConfig> &tag_configs,
        const cv::Mat &camera_mat,
        const cv::Mat &dist_coeffs,
        cv::Mat &rvec,
        cv::Mat &tvec,
        int &used_tags_cnt);
  };

} // namespace apriltag_localizer

#endif // APRILTAG_LOCALIZER__POSE_SOLVER_HPP_
