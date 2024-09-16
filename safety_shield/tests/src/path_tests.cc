#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <vector>

#include "path_fixture.h"
#include "safety_shield/motion.h"
#include "safety_shield/path.h"

namespace safety_shield {

TEST_F(PathTest, GetValuesTest) {
  EXPECT_DOUBLE_EQ(path_.getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(0), 1.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(1), 0.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(2), -1.0);
}

TEST_F(PathTest, ErrorHandlingTest) {
  EXPECT_THROW(path_.setPhases({-1.0, 0.0, 1.0}, {1.0, 0.0, -1.0}), std::invalid_argument);
  EXPECT_THROW(path_.setPhases({1.0, 0.0, 1.1}, {1.0, 0.0, -1.0}), std::invalid_argument);
  EXPECT_THROW(path_.increment(-0.0001), std::invalid_argument);
  path_.setPhases({1.0, 2.0, 4.0}, {-100.0, 0.0, 100.0});
  path_.increment(0.5);
  // Should not go below zero.
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
  path_.increment(0.5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
  path_.increment(0.5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
  path_.increment(0.5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
  path_.increment(2.0);
  // Acceleration is still negative.
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 1.0/2.0 * 100.0 * 2.0 * 2.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 2.0 * 100.0);
}

TEST_F(PathTest, IncrementTest) {
  // 0.5
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 0.02083333333, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.125);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.5);
  // 1.0
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 0.16666666666, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.5);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 1.0);
  // 1.5
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 0.54166666666, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 1.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 1.0);
  // 2.0
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 1.16666666666, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 1.5);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 1.0);
  // 2.5
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 2.02083333333, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 1.875);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.5);
  // 3.0
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 3.0, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 2.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);

  EXPECT_DOUBLE_EQ(path_.getJerk(0), 1.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(1), 0.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(2), -1.0);

  // 3.5
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 4.0, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 2.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
}

TEST_F(PathTest, GetFinalMotionTest) {
  double final_pos;
  double final_vel;
  double final_acc;
  path_.getFinalMotion(final_pos, final_vel, final_acc);
  EXPECT_DOUBLE_EQ(final_pos, 3.0);
  EXPECT_DOUBLE_EQ(final_vel, 2.0);
  EXPECT_DOUBLE_EQ(final_acc, 0.0);
  path_.increment(1.5);
  path_.getFinalMotion(final_pos, final_vel, final_acc);
  EXPECT_NEAR(final_pos, 3.0, 1e-5);
  EXPECT_DOUBLE_EQ(final_vel, 2.0);
  EXPECT_DOUBLE_EQ(final_acc, 0.0);
  path_.increment(2.0);
  path_.getFinalMotion(final_pos, final_vel, final_acc);
  EXPECT_NEAR(final_pos, 4.0, 1e-5);
  EXPECT_DOUBLE_EQ(final_vel, 2.0);
  EXPECT_DOUBLE_EQ(final_acc, 0.0);
}

TEST_F(PathTest, GetMaxVelocityTest) {
  EXPECT_DOUBLE_EQ(path_.getMaxVelocity(), 2.0);
}

TEST_F(PathStopTest, IncrementTest) {
  path_.increment(1.0);
  double pos = 0;
  double vel = 2;
  double acc = 0;
  double jerk = -1;
  pos = pos + vel + 0.5 * acc + 1.0 / 6.0 * jerk;
  vel = vel + acc + 0.5 * jerk;
  acc = acc + jerk;
  EXPECT_NEAR(path_.getPosition(), pos, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), vel);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), acc);
  path_.increment(1.0);
  jerk = 0;
  pos = pos + vel + 0.5 * acc + 1.0 / 6.0 * jerk;
  vel = vel + acc + 0.5 * jerk;
  acc = acc + jerk;
  EXPECT_NEAR(path_.getPosition(), pos, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), vel);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), acc);
  path_.increment(1.0);
  jerk = 1;
  pos = pos + vel + 0.5 * acc + 1.0 / 6.0 * jerk;
  vel = vel + acc + 0.5 * jerk;
  acc = acc + jerk;
  EXPECT_NEAR(path_.getPosition(), pos, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), vel);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), acc);
}

TEST_F(PathStopTest, GetFinalMotionTest) {
  double final_pos;
  double final_vel;
  double final_acc;
  path_.getFinalMotion(final_pos, final_vel, final_acc);
  EXPECT_DOUBLE_EQ(final_pos, 3.0);
  EXPECT_DOUBLE_EQ(final_vel, 0.0);
  EXPECT_DOUBLE_EQ(final_acc, 0.0);
  path_.increment(1.5);
  path_.getFinalMotion(final_pos, final_vel, final_acc);
  EXPECT_NEAR(final_pos, 3.0, 1e-5);
  EXPECT_DOUBLE_EQ(final_vel, 0.0);
  EXPECT_DOUBLE_EQ(final_acc, 0.0);
  path_.increment(2.0);
  path_.getFinalMotion(final_pos, final_vel, final_acc);
  EXPECT_NEAR(final_pos, 3.0, 1e-5);
  EXPECT_DOUBLE_EQ(final_vel, 0.0);
  EXPECT_DOUBLE_EQ(final_acc, 0.0);
}

TEST_F(PathStopTest, GetMotionUnderVelTest) {
  double pos_under_vel;
  double vel_under_vel;
  double acc_under_vel;
  double jerk_under_vel;
  double time;
  path_.getMotionUnderVel(2.0, time, pos_under_vel, vel_under_vel, acc_under_vel, jerk_under_vel);
  EXPECT_DOUBLE_EQ(time, 0.0);
  EXPECT_DOUBLE_EQ(pos_under_vel, 0.0);
  EXPECT_DOUBLE_EQ(vel_under_vel, 2.0);
  path_.getMotionUnderVel(1.5, time, pos_under_vel, vel_under_vel, acc_under_vel, jerk_under_vel);
  EXPECT_DOUBLE_EQ(time, 1.0);
  EXPECT_DOUBLE_EQ(vel_under_vel, 1.5);
  path_.getMotionUnderVel(-5.0, time, pos_under_vel, vel_under_vel, acc_under_vel, jerk_under_vel);
  EXPECT_DOUBLE_EQ(time, -1.0);  // Indicated that this never happens
  path_.increment(1.5);
  path_.getMotionUnderVel(0.0, time, pos_under_vel, vel_under_vel, acc_under_vel, jerk_under_vel);
  EXPECT_DOUBLE_EQ(time, 1.5);
  EXPECT_DOUBLE_EQ(vel_under_vel, 0.0);
}

TEST_F(PathStopTest, GetMaxVelocityTest) {
  EXPECT_DOUBLE_EQ(path_.getMaxVelocity(), 2.0);
  path_.increment(1.0);
  EXPECT_DOUBLE_EQ(path_.getMaxVelocity(), 1.5);
}

}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}