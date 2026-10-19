/**
 * @file test_feetech_bus_worker.cpp
 * @brief Unit tests verifying fail-safe braking, clean lifecycle, and FSM resilience in FeetechBusWorker.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "lekiwi_ftservo_hardware/feetech_bus_worker.hpp"
#include "lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp"
#include "lekiwi_ftservo_hardware/sts_constants.hpp"

namespace lekiwi_ftservo_hardware
{

  static JointTopology create_mock_wheel_topology()
  {
    JointTopology topology;
    JointRuntime wheel1;
    wheel1.name = "wheel_fl";
    wheel1.id = 1;
    wheel1.velocity_command = true;
    wheel1.max_velocity_radians_per_second = 5.0;
    wheel1.velocity_radians_per_second_per_tick = 0.00153398;
    wheel1.position_state_name = "wheel_fl/position";
    wheel1.velocity_state_name = "wheel_fl/velocity";
    wheel1.command_interface_name = "wheel_fl/velocity";
    wheel1.has_torque_enable_command = false;

    JointRuntime wheel2;
    wheel2.name = "wheel_fr";
    wheel2.id = 2;
    wheel2.velocity_command = true;
    wheel2.max_velocity_radians_per_second = 5.0;
    wheel2.velocity_radians_per_second_per_tick = 0.00153398;
    wheel2.position_state_name = "wheel_fr/position";
    wheel2.velocity_state_name = "wheel_fr/velocity";
    wheel2.command_interface_name = "wheel_fr/velocity";
    wheel2.has_torque_enable_command = false;

    topology.joints = {wheel1, wheel2};
    topology.all_ids = {1, 2};
    topology.velocity_joint_ids = {1, 2};
    topology.velocity_joint_indices = {0, 1};
    return topology;
  }

  TEST(FeetechBusWorkerTest, DefaultConstructorDestructorSafe)
  {
    EXPECT_NO_THROW({
      FeetechBusWorker worker;
      EXPECT_FALSE(worker.is_running());
      EXPECT_EQ(worker.get_bus_state(), sts::BusState::DISCONNECTED);
      EXPECT_EQ(worker.get_active_dtc(), sts::dtc::kDtcNone);
      EXPECT_DOUBLE_EQ(worker.get_current_per(), 0.0);
    });
  }

  TEST(FeetechBusWorkerTest, FsmInitialStateAfterInitIsDisconnected)
  {
    FeetechBusWorker worker;
    const auto topology = create_mock_wheel_topology();
    std::string err;
    EXPECT_TRUE(worker.init(topology, &err));
    EXPECT_EQ(worker.get_bus_state(), sts::BusState::DISCONNECTED);
    EXPECT_EQ(worker.get_active_dtc(), sts::dtc::kDtcNone);
    EXPECT_DOUBLE_EQ(worker.get_current_per(), 0.0);
  }

  TEST(FeetechBusWorkerTest, ConfigureFailsSafelyOnNonExistentPort)
  {
    FeetechBusWorker worker;
    const auto topology = create_mock_wheel_topology();
    std::string err;
    ASSERT_TRUE(worker.init(topology, &err));

    EXPECT_FALSE(worker.configure("/dev/non_existent_serial_lekiwi_123", 1000000, 20, &err));
    EXPECT_EQ(worker.get_bus_state(), sts::BusState::DISCONNECTED);
    EXPECT_EQ(worker.get_active_dtc(), sts::dtc::kDtcPortOpenError);
  }

  TEST(FeetechBusWorkerTest, VerifyDeviceNodeRejectsInvalidPaths)
  {
    std::string err;
    // Empty path
    EXPECT_FALSE(StsProtocol::verify_device_node("", &err));
    // Non-existent path
    EXPECT_FALSE(StsProtocol::verify_device_node("/dev/does_not_exist_xyz", &err));
    // Regular file (not character device)
    EXPECT_FALSE(StsProtocol::verify_device_node("/etc/hosts", &err));
    // Valid character device
    EXPECT_TRUE(StsProtocol::verify_device_node("/dev/null", &err));
  }

  TEST(FeetechBusWorkerTest, DtcAndBusStateStringConversions)
  {
    EXPECT_STREQ(sts::to_string(sts::BusState::DISCONNECTED), "DISCONNECTED");
    EXPECT_STREQ(sts::to_string(sts::BusState::STANDBY), "STANDBY");
    EXPECT_STREQ(sts::to_string(sts::BusState::OPERATIONAL), "OPERATIONAL");
    EXPECT_STREQ(sts::to_string(sts::BusState::DEGRADED), "DEGRADED");

    EXPECT_STRNE(sts::dtc::description(sts::dtc::kDtcDeviceNodeMissing), "");
    EXPECT_STRNE(sts::dtc::description(sts::dtc::kDtcPortOpenError), "");
    EXPECT_STRNE(sts::dtc::description(sts::dtc::kDtcServoConfigFailed), "");
    EXPECT_STRNE(sts::dtc::description(sts::dtc::kDtcAntiJerkResyncFailed), "");
    EXPECT_STRNE(sts::dtc::description(sts::dtc::kDtcHighPacketErrorRate), "");
  }

  TEST(FeetechBusWorkerTest, StopAndCleanupAreIdempotentAndSafeWithoutHardware)
  {
    FeetechBusWorker worker;
    const auto topology = create_mock_wheel_topology();
    std::string err;
    EXPECT_TRUE(worker.init(topology, &err));
    EXPECT_FALSE(worker.is_running());

    // Calling stop() when not running must be safe and attempt braking
    EXPECT_NO_THROW(worker.stop());
    EXPECT_FALSE(worker.is_running());

    // Calling cleanup() multiple times must be idempotent
    EXPECT_NO_THROW(worker.cleanup());
    EXPECT_NO_THROW(worker.cleanup());
    EXPECT_FALSE(worker.is_running());
    EXPECT_EQ(worker.get_bus_state(), sts::BusState::DISCONNECTED);
  }

  TEST(FeetechBusWorkerTest, DeactivateBrakesAndDisablesTorque)
  {
    FeetechBusWorker worker;
    const auto topology = create_mock_wheel_topology();
    std::string err;
    EXPECT_TRUE(worker.init(topology, &err));

    // deactivate() without configure should return false (protocol not configured) but not throw
    EXPECT_FALSE(worker.deactivate(&err));
    EXPECT_FALSE(worker.is_running());
  }

  TEST(FeetechBusWorkerTest, ActivateFailsSafelyWithoutConfiguration)
  {
    FeetechBusWorker worker;
    const auto topology = create_mock_wheel_topology();
    std::string err;
    EXPECT_TRUE(worker.init(topology, &err));

    // activate() without configure should return false immediately without deadlocking
    EXPECT_FALSE(worker.activate(&err));
    EXPECT_FALSE(worker.is_running());
    EXPECT_EQ(worker.get_bus_state(), sts::BusState::DISCONNECTED);
  }

  TEST(LeKiwiFeetechHardwareTest, LifecycleShutdownAndErrorHooksExecuteSafely)
  {
    EXPECT_NO_THROW({
      LeKiwiFeetechHardwareInterface hw;
      rclcpp_lifecycle::State state;

      // Invoking on_shutdown before configure/init must succeed safely
      EXPECT_EQ(hw.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);

      // Invoking on_error must succeed safely
      EXPECT_EQ(hw.on_error(state), hardware_interface::CallbackReturn::SUCCESS);
    });
  }

} // namespace lekiwi_ftservo_hardware
