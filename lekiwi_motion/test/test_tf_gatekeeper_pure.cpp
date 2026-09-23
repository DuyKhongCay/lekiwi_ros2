#include <gtest/gtest.h>
#include <limits>
#include "lekiwi_motion/tf_gatekeeper_node.hpp"

namespace policy = lekiwi_motion::policy;

TEST(ReadinessPolicy, RejectsMissingStaleAndFutureSamples)
{
  // Timestamp zero is not evidence that a dynamic sample is static.
  EXPECT_FALSE(policy::fresh(1.0, 0.0, 0.3));
  EXPECT_FALSE(policy::fresh(1.0, 0.5, 0.3));
  EXPECT_FALSE(policy::fresh(1.0, 1.1, 0.3));
  EXPECT_TRUE(policy::fresh(1.0, 0.9, 0.3));
}

TEST(ReadinessPolicy, RejectsInvalidVarianceBeforeSumming)
{
  // Opposite signs must not hide a negative variance behind a valid sum.
  EXPECT_FALSE(policy::converged(-0.001, 0.001, 0.0, 0.0012, 0.003));
  EXPECT_FALSE(policy::converged(std::numeric_limits<double>::quiet_NaN(), 0, 0, 1, 1));
  EXPECT_FALSE(policy::converged(0, 0, std::numeric_limits<double>::infinity(), 1, 1));
  EXPECT_TRUE(policy::converged(0.0005, 0.0005, 0.002, 0.0012, 0.003));
}

TEST(ReadinessPolicy, RequiresFinitePlanarStandstill)
{
  // Both translational and angular motion independently close the gate.
  EXPECT_TRUE(policy::stationary(0.01, 0, 0, 0.03, 0.08));
  EXPECT_FALSE(policy::stationary(0.03, 0.03, 0, 0.03, 0.08));
  EXPECT_FALSE(policy::stationary(0, 0, 0.09, 0.03, 0.08));
  EXPECT_FALSE(policy::stationary(std::numeric_limits<double>::quiet_NaN(), 0, 0, 1, 1));
}
