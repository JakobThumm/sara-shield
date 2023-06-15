#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>

#include "robot_reach_fixture.h"
#include "safety_shield/robot_reach.h"
#include "spdlog/spdlog.h"

namespace safety_shield {

TEST_F(RobotReachTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(RobotReachTest, ForwardKinematicTest0) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  robot_reach_->forwardKinematic(0.0, 0, result);
  Eigen::Matrix4d expect;
  expect << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0.1, 0, 0, 0, 1;
  EXPECT_TRUE(result.isApprox(expect));
}

TEST_F(RobotReachTest, ForwardKinematicTest1) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  robot_reach_->forwardKinematic(M_PI / 2, 0, result);
  Eigen::Matrix4d expect;
  expect << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0;
  EXPECT_TRUE(result.isApprox(expect));
}

TEST_F(RobotReachTest, PointToVectorTest) {
  reach_lib::Point p;
  p.x = 1.0;
  p.y = 2.0;
  p.z = 3.0;
  Eigen::Vector4d result = robot_reach_->pointToVector(p);
  Eigen::Vector4d expect;
  expect << 1.0, 2.0, 3.0, 1.0;
  EXPECT_TRUE(result.isApprox(expect));
}

TEST_F(RobotReachTest, PointTo3dVectorTest) {
  reach_lib::Point p;
  p.x = 1.0;
  p.y = 2.0;
  p.z = 3.0;
  Eigen::Vector3d result = robot_reach_->pointTo3dVector(p);
  Eigen::Vector3d expect;
  expect << 1.0, 2.0, 3.0;
  EXPECT_TRUE(result.isApprox(expect));
}

TEST_F(RobotReachTest, VectorToPointTest) {
  Eigen::Vector4d vec;
  vec << 1.0, 2.0, 3.0, 1.0;
  reach_lib::Point result = robot_reach_->vectorToPoint(vec);
  reach_lib::Point expect(1.0, 2.0, 3.0);
  EXPECT_EQ(result.x, expect.x);
  EXPECT_EQ(result.y, expect.y);
  EXPECT_EQ(result.z, expect.z);
}

TEST_F(RobotReachTest, TransformCapsuleTest0) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  reach_lib::Capsule result = robot_reach_->transformCapsule(0, T);
  reach_lib::Capsule expect(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, -0.1, 0.0), 0.01);
  EXPECT_EQ(result.p1_.x, expect.p1_.x);
  EXPECT_EQ(result.p1_.y, expect.p1_.y);
  EXPECT_EQ(result.p1_.z, expect.p1_.z);
  EXPECT_EQ(result.p2_.x, expect.p2_.x);
  EXPECT_EQ(result.p2_.y, expect.p2_.y);
  EXPECT_EQ(result.p2_.z, expect.p2_.z);
  EXPECT_EQ(result.r_, expect.r_);
}

TEST_F(RobotReachTest, TransformCapsuleTest1) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(1, 1) = 0.0;
  T(2, 2) = 0.0;
  T(1, 2) = -1.0;
  T(2, 1) = 1.0;
  T(2, 3) = 0.2;
  reach_lib::Capsule result = robot_reach_->transformCapsule(0, T);
  reach_lib::Capsule expect(reach_lib::Point(0.0, 0.0, 0.2), reach_lib::Point(0.0, 0.0, 0.1), 0.01);
  EXPECT_EQ(result.p1_.x, expect.p1_.x);
  EXPECT_EQ(result.p1_.y, expect.p1_.y);
  EXPECT_EQ(result.p1_.z, expect.p1_.z);
  EXPECT_EQ(result.p2_.x, expect.p2_.x);
  EXPECT_EQ(result.p2_.y, expect.p2_.y);
  EXPECT_EQ(result.p2_.z, expect.p2_.z);
  EXPECT_EQ(result.r_, expect.r_);
}

TEST_F(RobotReachTest, ReachTest0) {
  Motion start_config(0, std::vector<double>{0.0}, 0);
  Motion goal_config(0.1, std::vector<double>{0.0}, 0.1);
  std::vector<double> alpha_i{1.0};
  double s_diff = 0.1;
  std::vector<reach_lib::Capsule> result = robot_reach_->reach(start_config, goal_config, s_diff, alpha_i);
  std::vector<reach_lib::Capsule> expect;
  double expexted_radius = 0.01 + 1.0/8.0 * s_diff * s_diff * alpha_i[0] + 0.03;
  reach_lib::Capsule capsule(
    reach_lib::Point(0.0, 0.0, 1.0 + 0.1),
    reach_lib::Point(0.0, 0.0, 1.0 + 0.1 - 0.1),
    expexted_radius
  );
  expect.push_back(capsule);
  for (int i = 0; i < 1; i++) {
    EXPECT_EQ(result[i].p1_.x, expect[i].p1_.x);
    EXPECT_EQ(result[i].p1_.y, expect[i].p1_.y);
    EXPECT_EQ(result[i].p1_.z, expect[i].p1_.z);
    EXPECT_EQ(result[i].p2_.x, expect[i].p2_.x);
    EXPECT_EQ(result[i].p2_.y, expect[i].p2_.y);
    EXPECT_EQ(result[i].p2_.z, expect[i].p2_.z);
    EXPECT_EQ(result[i].r_, expect[i].r_);
  }
}

TEST_F(RobotReachTest, ReachTest1) {
  Motion start_config(0, std::vector<double>{0.0}, 0);
  Motion goal_config(0.1, std::vector<double>{M_PI/8.0}, 0.1);
  std::vector<double> alpha_i{1.0};
  double s_diff = 0.1;
  std::vector<reach_lib::Capsule> result = robot_reach_->reach(start_config, goal_config, s_diff, alpha_i);
  std::vector<reach_lib::Capsule> expect;
  double expexted_radius = sqrt(pow(0.03826834, 2) + pow(1.1 - 0.09238795 - 1.0, 2))/2.0 + 0.01 + 1.0/8.0 * pow(s_diff, 2) * alpha_i[0] + 0.03;
  /* T = 
    [ 1, 0, 0, 0;
      0, 0, -1, 0; 
      0, 1, 0, 1.1; 
      0, 0, 0, 1, ] * 
    [  0.9238795, -0.3826834,  0.0000000, 0.0;
       0.3826834,  0.9238795,  0.0000000, 0.0;
       0.0000000,  0.0000000,  1.0000000, 0.0;
       0, 0, 0, 1] =
    [    0.9239   -0.3827         0         0
         0         0   -1.0000         0
         0.3827    0.9239         0    1.1000
         0         0         0    1.0000]
  */
  reach_lib::Capsule capsule(
    reach_lib::Point(0.0, 0.0, 1.1),
    reach_lib::Point(0.03826834/2.0, 0.0, ((1.1 - 0.09238795) + 1.0) / 2),
    expexted_radius
  );
  expect.push_back(capsule);
  for (int i = 0; i < 1; i++) {
    EXPECT_NEAR(result[i].p1_.x, expect[i].p1_.x, 1e-5);
    EXPECT_NEAR(result[i].p1_.y, expect[i].p1_.y, 1e-5);
    EXPECT_NEAR(result[i].p1_.z, expect[i].p1_.z, 1e-5);
    EXPECT_NEAR(result[i].p2_.x, expect[i].p2_.x, 1e-5);
    EXPECT_NEAR(result[i].p2_.y, expect[i].p2_.y, 1e-5);
    EXPECT_NEAR(result[i].p2_.z, expect[i].p2_.z, 1e-5);
    EXPECT_NEAR(result[i].r_, expect[i].r_, 1e-5);
  }
}

}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}