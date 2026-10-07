// Copyright 2026 LeKiwi Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file system_readiness_node.hpp
 * @brief ROS 2 node monitoring navigation and manipulation readiness states.
 * @details Aggregates local/global odometry, joint states, and TF chain freshness
 *          to determine whether the platform is safe for navigation or precision grasping.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__SYSTEM_READINESS_NODE_HPP_
#define LEKIWI_MOTION__SYSTEM_READINESS_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lekiwi_motion/supervision/system_readiness_state.hpp"

namespace lekiwi_motion
{

    /**
     * @class SystemReadinessNode
     * @brief Node evaluating system readiness for navigation and grasping tasks.
     * @details Subscribes to odometry streams and joint states, inspects TF transforms,
     *          runs domain readiness evaluation, publishes boolean status topics, and
     *          serves trigger queries.
     */
    class SystemReadinessNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructs the readiness evaluation node and registers parameters.
         * @param[in] options Node options passed to base rclcpp::Node.
         */
        explicit SystemReadinessNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default virtual destructor.
         */
        ~SystemReadinessNode() override = default;

    private:
        /**
         * @brief Periodically samples sensor snapshots and updates readiness status.
         */
        void evaluate_readiness();

        /**
         * @brief Callback caching incoming local filtered odometry.
         * @param[in] msg Pointer to nav_msgs::msg::Odometry message.
         */
        void on_local_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

        /**
         * @brief Callback caching incoming global fused odometry.
         * @param[in] msg Pointer to nav_msgs::msg::Odometry message.
         */
        void on_global_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

        /**
         * @brief Callback caching robot arm joint states.
         * @param[in] msg Pointer to sensor_msgs::msg::JointState message.
         */
        void on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg);

        /**
         * @brief Checks if a coordinate transform is available and fresh within max age.
         * @param[in] target Target coordinate frame name.
         * @param[in] source Source coordinate frame name.
         * @param[in] now_sec Current ROS clock timestamp in seconds.
         * @param[in] max_age_sec Maximum permissible transform age in seconds.
         * @param[in] is_static Flag indicating if the transform is static.
         * @return True if transform exists and meets age criteria.
         */
        bool is_transform_fresh(
            const std::string &target,
            const std::string &source,
            double now_sec,
            double max_age_sec,
            bool is_static = false) const;

        /**
         * @brief Fills diagnostic status wrapper with readiness and drift metrics.
         * @param[out] stat Diagnostic status wrapper to populate.
         */
        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

        /**
         * @brief Service callback handling navigation readiness query.
         * @param[in] req Trigger service request.
         * @param[out] res Trigger service response populated with nav readiness flag.
         */
        void handle_nav_query(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

        /**
         * @brief Service callback handling grasp precision readiness query.
         * @param[in] req Trigger service request.
         * @param[out] res Trigger service response populated with grasp readiness flag.
         */
        void handle_grasp_query(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

        /// TF2 buffer and listener for transform chain verification.
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        /// Diagnostic updater instance for ROS 2 diagnostic aggregation.
        diagnostic_updater::Updater diagnostic_updater_;

        /// Subscriptions to robot sensor and state streams.
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        /// Heartbeat publishers for nav and grasp readiness flags.
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_ready_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_ready_pub_;

        /// Query services for synchronous readiness interrogation.
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nav_readiness_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grasp_readiness_srv_;

        /// Periodic timer driving evaluation loop.
        rclcpp::TimerBase::SharedPtr eval_timer_;

        /// Pure domain logic evaluator instance and latest report cache.
        SystemReadinessEvaluator evaluator_;
        SystemReadinessReport last_report_;

        /// Cached sensor snapshots.
        LocalOdomSnapshot local_odom_snapshot_;
        GlobalOdomSnapshot global_odom_snapshot_;
        JointsSnapshot joints_snapshot_;

        /// Configuration parameters and coordinate frames.
        std::string map_frame_{"map"};
        std::string odom_frame_{"odom"};
        std::string base_frame_{"base_footprint"};
        std::string tip_frame_{"gripperframe"};
        std::string board_frame_{"chessboard_frame"};
        std::string local_odom_topic_{"/odometry/local"};
        std::string global_odom_topic_{"/odometry/global"};
        std::string joint_states_topic_{"/joint_states"};
        double check_frequency_hz_{10.0};
        double max_transform_age_sec_{0.30};
        double nav_odom_max_age_sec_{0.50};

        /// Cached publication states and monotonic clock trackers.
        bool last_published_nav_ready_{false};
        bool last_published_grasp_ready_{false};
        double last_eval_time_sec_{0.0};
    };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__SYSTEM_READINESS_NODE_HPP_
