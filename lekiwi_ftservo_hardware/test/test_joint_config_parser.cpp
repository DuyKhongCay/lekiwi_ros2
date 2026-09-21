/**
 * @file test_joint_config_parser.cpp
 * @brief Unit tests for JointConfigParser (L1 verification).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "lekiwi_ftservo_hardware/joint_config_parser.hpp"

namespace lekiwi_ftservo_hardware
{

  static hardware_interface::InterfaceInfo make_interface(const std::string &name)
  {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    return info;
  }

  TEST(JointConfigParserTest, RejectsEmptyJoints)
  {
    hardware_interface::HardwareInfo info;
    JointTopology topology;
    std::string error;

    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_FALSE(error.empty());
  }

  TEST(JointConfigParserTest, RejectsMissingOrInvalidId)
  {
    hardware_interface::HardwareInfo info;
    hardware_interface::ComponentInfo joint;
    joint.name = "joint1";
    joint.command_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));

    // 1. Missing ID parameter
    info.joints = {joint};
    JointTopology topology;
    std::string error;
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));

    // 2. Non-integer ID
    joint.parameters["id"] = "abc";
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));

    // 3. Out of range ID (0 or 254)
    joint.parameters["id"] = "0";
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));

    joint.parameters["id"] = "254";
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
  }

  TEST(JointConfigParserTest, RejectsDuplicateIds)
  {
    hardware_interface::HardwareInfo info;
    hardware_interface::ComponentInfo joint1;
    joint1.name = "joint1";
    joint1.parameters["id"] = "5";
    joint1.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    joint1.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));
    joint1.command_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));

    hardware_interface::ComponentInfo joint2;
    joint2.name = "joint2";
    joint2.parameters["id"] = "5";
    joint2.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    joint2.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));
    joint2.command_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));

    info.joints = {joint1, joint2};
    JointTopology topology;
    std::string error;
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("Duplicate"), std::string::npos);
  }

  TEST(JointConfigParserTest, ValidatesStateInterfaces)
  {
    hardware_interface::HardwareInfo info;
    hardware_interface::ComponentInfo joint;
    joint.name = "joint1";
    joint.parameters["id"] = "1";
    joint.command_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));

    // 1. Missing both state interfaces
    info.joints = {joint};
    JointTopology topology;
    std::string error;
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("missing required state interface"), std::string::npos);

    // 2. Only position state interface (missing velocity)
    joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("missing required state interface 'velocity'"), std::string::npos);

    // 3. Only velocity state interface (missing position)
    joint.state_interfaces = {make_interface(hardware_interface::HW_IF_VELOCITY)};
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("missing required state interface 'position'"), std::string::npos);
  }

  TEST(JointConfigParserTest, ValidatesCommandInterfaces)
  {
    hardware_interface::HardwareInfo info;
    hardware_interface::ComponentInfo joint;
    joint.name = "joint1";
    joint.parameters["id"] = "1";
    joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));

    // 1. Missing primary command interface
    info.joints = {joint};
    JointTopology topology;
    std::string error;
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));

    // 2. Unsupported command interface
    joint.command_interfaces.push_back(make_interface("effort"));
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));

    // 3. Conflicting primary interfaces (both position and velocity)
    joint.command_interfaces = {
        make_interface(hardware_interface::HW_IF_POSITION),
        make_interface(hardware_interface::HW_IF_VELOCITY)};
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
  }

  TEST(JointConfigParserTest, RejectsInvalidOptionalParameters)
  {
    hardware_interface::HardwareInfo info;
    hardware_interface::ComponentInfo joint;
    joint.name = "joint1";
    joint.parameters["id"] = "1";
    joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));
    joint.command_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));

    // 1. Out-of-range acceleration
    joint.parameters["acceleration"] = "255";
    info.joints = {joint};
    JointTopology topology;
    std::string error;
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("Out-of-range acceleration"), std::string::npos);

    // 2. Unparseable acceleration
    joint.parameters["acceleration"] = "fast";
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("Invalid acceleration"), std::string::npos);
    joint.parameters.erase("acceleration");

    // 3. Non-positive velocity scale
    joint.parameters["velocity_radians_per_second_per_tick"] = "-0.01";
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("must be positive"), std::string::npos);
    joint.parameters.erase("velocity_radians_per_second_per_tick");

    // 4. Invalid velocity direction
    joint.parameters["velocity_direction"] = "0";
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("velocity_direction must be 1 or -1"), std::string::npos);
    joint.parameters.erase("velocity_direction");

    // 5. Non-positive max velocity
    joint.parameters["max_velocity_radians_per_second"] = "-1.0";
    info.joints = {joint};
    EXPECT_FALSE(JointConfigParser::parse(info, topology, &error));
    EXPECT_NE(error.find("max_velocity_radians_per_second must be positive"), std::string::npos);
  }

  TEST(JointConfigParserTest, SuccessfullyParsesHybridTopology)
  {
    hardware_interface::HardwareInfo info;

    // Joint 1: Position joint (Arm) with acceleration
    hardware_interface::ComponentInfo arm_joint;
    arm_joint.name = "arm_shoulder_pan";
    arm_joint.parameters["id"] = "1";
    arm_joint.parameters["acceleration"] = "50";
    arm_joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    arm_joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));
    arm_joint.command_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    arm_joint.command_interfaces.push_back(make_interface("torque_enable"));

    // Joint 2: Velocity joint (Wheel) with scale, direction, max_vel
    hardware_interface::ComponentInfo wheel_joint;
    wheel_joint.name = "base_left_wheel";
    wheel_joint.parameters["id"] = "7";
    wheel_joint.parameters["acceleration"] = "0";
    wheel_joint.parameters["velocity_radians_per_second_per_tick"] = "0.002";
    wheel_joint.parameters["velocity_direction"] = "-1";
    wheel_joint.parameters["max_velocity_radians_per_second"] = "4.5";
    wheel_joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_POSITION));
    wheel_joint.state_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));
    wheel_joint.command_interfaces.push_back(make_interface(hardware_interface::HW_IF_VELOCITY));
    wheel_joint.command_interfaces.push_back(make_interface("torque_enable"));

    info.joints = {arm_joint, wheel_joint};
    JointTopology topology;
    std::string error;

    ASSERT_TRUE(JointConfigParser::parse(info, topology, &error)) << error;
    EXPECT_EQ(topology.size(), 2U);
    EXPECT_EQ(topology.all_ids, (std::vector<uint8_t>{1, 7}));
    EXPECT_EQ(topology.position_joint_ids, (std::vector<uint8_t>{1}));
    EXPECT_EQ(topology.velocity_joint_ids, (std::vector<uint8_t>{7}));
    EXPECT_EQ(topology.position_joint_indices, (std::vector<size_t>{0}));
    EXPECT_EQ(topology.velocity_joint_indices, (std::vector<size_t>{1}));

    // Verify Arm Joint metadata
    EXPECT_EQ(topology.joints[0].name, "arm_shoulder_pan");
    EXPECT_EQ(topology.joints[0].id, 1);
    EXPECT_FALSE(topology.joints[0].velocity_command);
    EXPECT_EQ(topology.joints[0].acceleration, 50);
    EXPECT_TRUE(topology.joints[0].has_torque_enable_command);
    EXPECT_EQ(topology.joints[0].command_interface_name, "arm_shoulder_pan/position");
    EXPECT_EQ(topology.joints[0].torque_enable_command_name, "arm_shoulder_pan/torque_enable");

    // Verify Wheel Joint metadata
    EXPECT_EQ(topology.joints[1].name, "base_left_wheel");
    EXPECT_EQ(topology.joints[1].id, 7);
    EXPECT_TRUE(topology.joints[1].velocity_command);
    EXPECT_DOUBLE_EQ(topology.joints[1].velocity_radians_per_second_per_tick, 0.002);
    EXPECT_EQ(topology.joints[1].velocity_direction, -1);
    EXPECT_DOUBLE_EQ(topology.joints[1].max_velocity_radians_per_second, 4.5);
    EXPECT_TRUE(topology.joints[1].has_torque_enable_command);
    EXPECT_EQ(topology.joints[1].command_interface_name, "base_left_wheel/velocity");
  }

} // namespace lekiwi_ftservo_hardware
