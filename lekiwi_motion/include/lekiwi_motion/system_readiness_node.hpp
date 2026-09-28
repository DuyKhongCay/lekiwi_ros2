// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_MOTION__SYSTEM_READINESS_NODE_HPP_
#define LEKIWI_MOTION__SYSTEM_READINESS_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lekiwi_motion/system_readiness_state.hpp"

namespace lekiwi_motion
{

    class SystemReadinessNode : public rclcpp::Node
    {
    public:
        explicit SystemReadinessNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~SystemReadinessNode() override = default;

    private:
        void evaluate_readiness();

        void on_local_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void on_global_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg);

        bool is_transform_fresh(
            const std::string &target,
            const std::string &source,
            double now_sec,
            double max_age_sec,
            bool is_static = false) const;

        void dispatch_nav2_startup();
        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

        void handle_nav_query(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res);
        void handle_grasp_query(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

        // TF2 Buffer & Listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Diagnostics
        diagnostic_updater::Updater diagnostic_updater_;

        // Subscriptions
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_ready_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_ready_pub_;

        // Services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nav_readiness_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grasp_readiness_srv_;

        // Nav2 Autostart Client
        rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr nav2_client_;

        // Periodic Timer
        rclcpp::TimerBase::SharedPtr eval_timer_;

        // Pure domain logic evaluator
        SystemReadinessEvaluator evaluator_;
        SystemReadinessReport last_report_;

        // Local caching of inputs
        LocalOdomSnapshot local_odom_snapshot_;
        GlobalOdomSnapshot global_odom_snapshot_;
        JointsSnapshot joints_snapshot_;

        // Parameters
        std::string map_frame_{"map"};
        std::string odom_frame_{"odom"};
        std::string base_frame_{"base_footprint"};
        std::string ee_frame_{"gripperframe"};
        std::string board_frame_{"chessboard_frame"};
        double check_frequency_hz_{10.0};
        double max_transform_age_sec_{0.30};
        double nav_odom_max_age_sec_{0.50};
        bool autostart_nav2_{true};
        std::string nav2_lifecycle_service_{"/lifecycle_manager_navigation/manage_nodes"};
        double nav2_service_timeout_sec_{5.0};

        // Nav2 Autostart State Tracking
        std::atomic<bool> nav2_started_{false};
        std::atomic<bool> nav2_dispatch_in_progress_{false};
        rclcpp::Time nav2_dispatch_time_{0, 0, RCL_ROS_TIME};

        // Cached publication states
        bool last_published_nav_ready_{false};
        bool last_published_grasp_ready_{false};
        double last_eval_time_sec_{0.0};
    };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__SYSTEM_READINESS_NODE_HPP_
