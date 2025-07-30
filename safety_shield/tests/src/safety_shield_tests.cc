#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>

#include "safety_shield/safety_shield.h"
#include "safety_shield_fixture.h"
#include "safety_shield/exceptions.h"
#include "spdlog/spdlog.h"

namespace safety_shield {

void throwRobotMovementException() {
  try {
    throw RobotMovementException();
  } catch (RobotMovementException e) {
    std::cout << e.what() << std::endl;
  }
}

void throwTrajectoryException(std::string msg) {
  try {
    throw TrajectoryException(msg);
  } catch (TrajectoryException e) {
    std::cout << e.what() << std::endl;
  }
}

TEST(SafetyShieldExceptionTest, ExpectionTest) {
  EXPECT_NO_THROW(throwRobotMovementException());
  EXPECT_NO_THROW(throwTrajectoryException("Test"));
}

TEST_F(SafetyShieldTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(SafetyShieldTest, CalculateMaxAccJerkTest) {
  std::vector<double> prev_speed = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> a_max_part = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  std::vector<double> j_max_part = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  // max_s_stop: 0.2
  // a_max_allowed: [10, 10, 10, 10, 10, 10]
  // j_max_allowed: [400, 400, 400, 400, 400, 400]
  double a_max_manoeuvre, j_max_manoeuvre;
  shield_->calculateMaxAccJerk(prev_speed, a_max_part, j_max_part, a_max_manoeuvre, j_max_manoeuvre);
  EXPECT_NEAR(a_max_manoeuvre, (10.0 - 2.0) / (1.0 + 2.0 * 0.2), 1e-5);
  EXPECT_NEAR(j_max_manoeuvre, (400.0 - 10.0 - 3 * 2.0 * a_max_manoeuvre) / (1.0 + 2.0 * 0.2), 1e-5);
}

TEST_F(SafetyShieldTest, PlanSafetyShieldTest) {
  double pos = 0;
  double vel = 0;
  double acc = 0;
  double ve = 1;
  double a_max = 10;
  double j_max = 100;
  double final_pos, final_vel, final_acc;
  bool success;
  safety_shield::Path path;
  for (int i = 0; i < 11; i++) {
    vel = double(i) / 10.0;
    for (int j = -40; j < 40; j++) {
      acc = double(j) / 5.0;
      double t_to_a_0 = ceil((abs(acc) / j_max) / shield_->getSampleTime()) * shield_->getSampleTime();
      if (vel + acc * t_to_a_0 / 2 < 0) {
        continue;
      }
      for (int k = 0; k < 101; k++) {
        ve = double(k) / 100.0;
        success = shield_->planSafetyShield(pos, vel, acc, ve, a_max, j_max, path);
        ASSERT_TRUE(success);
        path.getFinalMotion(final_pos, final_vel, final_acc);
        ASSERT_NEAR(final_vel, ve, 1e-6);
        ASSERT_NEAR(final_acc, 0.0, 1e-2);
      }
    }
  }
}

TEST_F(SafetyShieldTest, NextMotionTest) {
  // Dummy human measurement
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] = reach_lib::Point(1.5177, -0.1214,  0.3653);
  dummy_human_meas[1] = reach_lib::Point(1.3805, -0.1239,  0.3661);
  dummy_human_meas[2] = reach_lib::Point(1.4457, -0.1015,  0.5656);
  dummy_human_meas[3] = reach_lib::Point(1.55199995, -0.49652919,  0.35673341);
  dummy_human_meas[4] = reach_lib::Point(1.34220006, -0.50628106,  0.35305421);
  dummy_human_meas[5] = reach_lib::Point(1.45120127, -0.10260095,  0.70079994);
  dummy_human_meas[6] = reach_lib::Point(1.53840061, -0.89403554,  0.30875003);
  dummy_human_meas[7] = reach_lib::Point(1.35799936, -0.90420248,  0.30646656);
  dummy_human_meas[8] = reach_lib::Point(1.45260135, -0.12800337,  0.75369877);
  dummy_human_meas[9] = reach_lib::Point(1.56480009, -0.95111153,  0.42744498);
  dummy_human_meas[10] = reach_lib::Point(1.33259994, -0.95372288,  0.42934275);
  dummy_human_meas[11] = reach_lib::Point(1.4498005 , -0.08519133,  0.96759635);
  dummy_human_meas[12] = reach_lib::Point(1.53150086, -0.09399649,  0.87549717);
  dummy_human_meas[13] = reach_lib::Point(1.37090087, -0.08939671,  0.87259628);
  dummy_human_meas[14] = reach_lib::Point(1.45500055, -0.13649181,  1.03259597);
  dummy_human_meas[15] = reach_lib::Point(1.62240178, -0.08509644,  0.90599443);
  dummy_human_meas[16] = reach_lib::Point(1.27479982, -0.08029668,  0.90509315);
  dummy_human_meas[17] = reach_lib::Point(1.88200111, -0.05759683,  0.89318002);
  dummy_human_meas[18] = reach_lib::Point(1.02110049, -0.05889703,  0.89177984);
  dummy_human_meas[19] = reach_lib::Point(2.13130103, -0.05649683,  0.90218228);
  dummy_human_meas[20] = reach_lib::Point(0.76580055, -0.05329701,  0.89958196);
  dummy_human_meas[21] = reach_lib::Point(2.21530148, -0.04149671,  0.89398712);
  dummy_human_meas[22] = reach_lib::Point(0.68120021, -0.04299688,  0.8933868);

  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.912;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  reach_lib::AABB table = reach_lib::AABB({-1.0, -1.0, -0.1}, {1.0, 1.0, 0.0});
  std::vector<reach_lib::AABB> environment_elements = {table};
  ShieldType shield_type = ShieldType::SSM;

  double t = 0.0;
  safety_shield::Motion current_motion;
  safety_shield::Motion next_motion = shield_->getCurrentMotion();
  for (int ep = 0; ep < 2; ep++) {
    for (int i = 0; i < 100; i++) {  // i < 100; i<10000
      t += 0.001;
      shield_->humanMeasurement(dummy_human_meas, t);
      t += 0.003;
      current_motion = shield_->getCurrentMotion();
      EXPECT_TRUE(current_motion.hasSamePos(&next_motion));
      EXPECT_TRUE(current_motion.hasSameVel(&next_motion));
      EXPECT_TRUE(current_motion.hasSameAcc(&next_motion));
      if (i % 10 == 0) {  // % 2
        std::vector<double> qpos{0.2 * t, t, t,
                                 t,       t, std::min(t, 3.1)};  // qpos{0.2*t, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
        std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        shield_->newLongTermTrajectory(qpos, qvel);
        // spdlog::info("new LTT");
      }
      next_motion = shield_->step(t);
      // spdlog::info("finished step");
    }
    shield_->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, environment_elements, shield_type);
    next_motion = shield_->getCurrentMotion();
  }
}

}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}