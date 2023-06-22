#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "robot_reach_fixture.h"
#include "safety_shield/robot_reach.h"
#include "spdlog/spdlog.h"

namespace safety_shield {

TEST_F(RobotReachTest, InitializationTest) {
  EXPECT_NEAR(0, 0.0, 1e-8);
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
  double expexted_radius = 0.01 + 1.0 / 8.0 * s_diff * s_diff * alpha_i[0] + 0.03;
  reach_lib::Capsule capsule(reach_lib::Point(0.0, 0.0, 1.0 + 0.1), reach_lib::Point(0.0, 0.0, 1.0 + 0.1 - 0.1),
                             expexted_radius);
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
  Motion goal_config(0.1, std::vector<double>{M_PI / 8.0}, 0.1);
  std::vector<double> alpha_i{1.0};
  double s_diff = 0.1;
  std::vector<reach_lib::Capsule> result = robot_reach_->reach(start_config, goal_config, s_diff, alpha_i);
  std::vector<reach_lib::Capsule> expect;
  double expexted_radius = sqrt(pow(0.03826834, 2) + pow(1.1 - 0.09238795 - 1.0, 2)) / 2.0 + 0.01 +
                           1.0 / 8.0 * pow(s_diff, 2) * alpha_i[0] + 0.03;
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
  reach_lib::Capsule capsule(reach_lib::Point(0.0, 0.0, 1.1),
                             reach_lib::Point(0.03826834 / 2.0, 0.0, ((1.1 - 0.09238795) + 1.0) / 2), expexted_radius);
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

// Test this function
/*
Eigen::Matrix<double, 6, Eigen::Dynamic> RobotReach::getJacobian(const int joint, const reach_lib::Point& point) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
  jacobian.setZero(6, joint + 1);
  Eigen::Vector3d p_e = pointTo3dVector(point);
  for (int i = 0; i < joint + 1; ++i) {
    Eigen::Vector3d z_i = z_vectors_[i + 1];
    Eigen::Vector3d p_i = pointTo3dVector(robot_capsules_for_velocity_[i].p1_);
    Eigen::Vector3d upper = z_i.cross(p_e - p_i);
    Eigen::Vector<double, 6> column;
    column << upper, z_i;
    jacobian.col(i) = column;
  }
  return jacobian;
}
*/

void compareJacobians(Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian,
                      Eigen::Matrix<double, 6, Eigen::Dynamic>& expect) {
  ASSERT_TRUE(jacobian.size() == expect.size());
  for (unsigned i = 0; i < jacobian.rows(); ++i) {
    for (unsigned j = 0; j < jacobian.cols(); ++j) {
      EXPECT_NEAR(jacobian(i, j), expect(i, j), 1e-8);
    }
  }
}

TEST_F(RobotReachTest, GetJacobianTest0) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{0.0});
  reach_lib::Point point(0.0, 0.0, 1.1);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(0, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect;
  expect.setZero(6, 1);
  expect << 0, 0, 0, 0, -1, 0;
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);
}

TEST_F(RobotReachTest, GetJacobianTest1) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{0.0});
  reach_lib::Point point(0.0, 0.0, 1.0);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(0, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect;
  expect.setZero(6, 1);
  expect << 0.1, 0, 0, 0, -1, 0;
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);
}

TEST_F(RobotReachTest, GetJacobianTest2) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{M_PI / 2.0});
  reach_lib::Point point(0.1, 0.0, 1.1);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(0, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect;
  expect.setZero(6, 1);
  expect << 0.0, 0, 0.1, 0, -1, 0;
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);
}

TEST_F(RobotReachTest, GetVelocityOfCapsuleTest0) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{0.0});
  std::vector<double> q_dot{1.0};
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->getVelocityOfCapsule(0, q_dot);
  RobotReach::SE3Vel vel1 = capsule_velocity.first;
  RobotReach::SE3Vel vel2 = capsule_velocity.second;
  EXPECT_NEAR(vel1.first(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.first(1), 0.0, 1e-8);
  EXPECT_NEAR(vel1.first(2), 0.0, 1e-8);
  EXPECT_NEAR(vel1.second(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.second(1), -1.0, 1e-8);
  EXPECT_NEAR(vel1.second(2), 0.0, 1e-8);
  EXPECT_NEAR(vel2.first(0), 0.1, 1e-8);
  EXPECT_NEAR(vel2.first(1), 0.0, 1e-8);
  EXPECT_NEAR(vel2.first(2), 0.0, 1e-8);
  EXPECT_NEAR(vel2.second(0), 0.0, 1e-8);
  EXPECT_NEAR(vel2.second(1), -1.0, 1e-8);
  EXPECT_NEAR(vel2.second(2), 0.0, 1e-8);
}

TEST_F(RobotReachTest, GetVelocityOfCapsuleTest1) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{M_PI / 2.0});
  std::vector<double> q_dot{1.0};
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->getVelocityOfCapsule(0, q_dot);
  RobotReach::SE3Vel vel1 = capsule_velocity.first;
  RobotReach::SE3Vel vel2 = capsule_velocity.second;
  EXPECT_NEAR(vel1.first(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.first(1), 0.0, 1e-8);
  EXPECT_NEAR(vel1.first(2), 0.0, 1e-8);
  EXPECT_NEAR(vel1.second(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.second(1), -1.0, 1e-8);
  EXPECT_NEAR(vel1.second(2), 0.0, 1e-8);
  EXPECT_NEAR(vel2.first(0), 0.0, 1e-8);
  EXPECT_NEAR(vel2.first(1), 0.0, 1e-8);
  EXPECT_NEAR(vel2.first(2), 0.1, 1e-8);
  EXPECT_NEAR(vel2.second(0), 0.0, 1e-8);
  EXPECT_NEAR(vel2.second(1), -1.0, 1e-8);
  EXPECT_NEAR(vel2.second(2), 0.0, 1e-8);
}

TEST_F(RobotReachTest, ApproximateVelOfCapsuleTest0) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{0.0});
  std::vector<double> q_dot{1.0};
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->getVelocityOfCapsule(0, q_dot);
  double v_approx =
      robot_reach_->approximateVelOfCapsule(0, capsule_velocity.second.first, capsule_velocity.second.second);
  EXPECT_NEAR(v_approx, 0.11, 1e-8);
}

TEST_F(RobotReachTest, ApproximateVelOfCapsuleTest1) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{M_PI / 2.0});
  std::vector<double> q_dot{1.0};
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->getVelocityOfCapsule(0, q_dot);
  double v_approx =
      robot_reach_->approximateVelOfCapsule(0, capsule_velocity.second.first, capsule_velocity.second.second);
  EXPECT_NEAR(v_approx, 0.11, 1e-8);
}

// jacobian of tcp from siciliano p. 114
// a1 = a2 = a3 = 1
Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_tcp_siciliano(std::vector<double> q) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
  jacobian.setZero(6, 3);
  double element_00 = -sin(q[0]) - sin(q[0] + q[1]) - sin(q[0] + q[1] + q[2]);
  double element_01 = cos(q[0]) + cos(q[0] + q[1]) + cos(q[0] + q[1] + q[2]);
  double element_10 = -sin(q[0] + q[1]) - sin(q[0] + q[1] + q[2]);
  double element_11 = cos(q[0] + q[1]) + cos(q[0] + q[1] + q[2]);
  double element_20 = -sin(q[0] + q[1] + q[2]);
  double element_21 = cos(q[0] + q[1] + q[2]);

  Eigen::Vector<double, 6> col_0;
  col_0 << element_00, element_01, 0, 0, 0, 1;
  jacobian.col(0) = col_0;

  Eigen::Vector<double, 6> col_1;
  col_1 << element_10, element_11, 0, 0, 0, 1;
  jacobian.col(1) = col_1;

  Eigen::Vector<double, 6> col_2;
  col_2 << element_20, element_21, 0, 0, 0, 1;
  jacobian.col(2) = col_2;

  return jacobian;
}

TEST_F(RobotReachTestVelocity, JacobianSicilianoTest0) {
  std::vector<double> q = {0.0, 0.0, 0.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  reach_lib::Point point = robot_reach_->getRobotCapsulesForVelocity()[2].p2_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(2, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect = jacobian_tcp_siciliano(q);

  // column 0
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);

  // column 1
  EXPECT_NEAR(jacobian(0, 1), expect(0, 1), 1e-8);
  EXPECT_NEAR(jacobian(1, 1), expect(1, 1), 1e-8);
  EXPECT_NEAR(jacobian(2, 1), expect(2, 1), 1e-8);
  EXPECT_NEAR(jacobian(3, 1), expect(3, 1), 1e-8);
  EXPECT_NEAR(jacobian(4, 1), expect(4, 1), 1e-8);
  EXPECT_NEAR(jacobian(5, 1), expect(5, 1), 1e-8);

  // column 2
  EXPECT_NEAR(jacobian(0, 2), expect(0, 2), 1e-8);
  EXPECT_NEAR(jacobian(1, 2), expect(1, 2), 1e-8);
  EXPECT_NEAR(jacobian(2, 2), expect(2, 2), 1e-8);
  EXPECT_NEAR(jacobian(3, 2), expect(3, 2), 1e-8);
  EXPECT_NEAR(jacobian(4, 2), expect(4, 2), 1e-8);
  EXPECT_NEAR(jacobian(5, 2), expect(5, 2), 1e-8);
}

TEST_F(RobotReachTestVelocity, JacobianSicilianoTest1) {
  std::vector<double> q = {1.0, 1.0, 1.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  reach_lib::Point point = robot_reach_->getRobotCapsulesForVelocity()[2].p2_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(2, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect = jacobian_tcp_siciliano(q);

  // column 0
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);

  // column 1
  EXPECT_NEAR(jacobian(0, 1), expect(0, 1), 1e-8);
  EXPECT_NEAR(jacobian(1, 1), expect(1, 1), 1e-8);
  EXPECT_NEAR(jacobian(2, 1), expect(2, 1), 1e-8);
  EXPECT_NEAR(jacobian(3, 1), expect(3, 1), 1e-8);
  EXPECT_NEAR(jacobian(4, 1), expect(4, 1), 1e-8);
  EXPECT_NEAR(jacobian(5, 1), expect(5, 1), 1e-8);

  // column 2
  EXPECT_NEAR(jacobian(0, 2), expect(0, 2), 1e-8);
  EXPECT_NEAR(jacobian(1, 2), expect(1, 2), 1e-8);
  EXPECT_NEAR(jacobian(2, 2), expect(2, 2), 1e-8);
  EXPECT_NEAR(jacobian(3, 2), expect(3, 2), 1e-8);
  EXPECT_NEAR(jacobian(4, 2), expect(4, 2), 1e-8);
  EXPECT_NEAR(jacobian(5, 2), expect(5, 2), 1e-8);
}

TEST_F(RobotReachTestVelocity, JacobianSicilianoTest2) {
  std::vector<double> q = {2.0, 2.0, 2.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  reach_lib::Point point = robot_reach_->getRobotCapsulesForVelocity()[2].p2_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(2, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect = jacobian_tcp_siciliano(q);

  // column 0
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);

  // column 1
  EXPECT_NEAR(jacobian(0, 1), expect(0, 1), 1e-8);
  EXPECT_NEAR(jacobian(1, 1), expect(1, 1), 1e-8);
  EXPECT_NEAR(jacobian(2, 1), expect(2, 1), 1e-8);
  EXPECT_NEAR(jacobian(3, 1), expect(3, 1), 1e-8);
  EXPECT_NEAR(jacobian(4, 1), expect(4, 1), 1e-8);
  EXPECT_NEAR(jacobian(5, 1), expect(5, 1), 1e-8);

  // column 2
  EXPECT_NEAR(jacobian(0, 2), expect(0, 2), 1e-8);
  EXPECT_NEAR(jacobian(1, 2), expect(1, 2), 1e-8);
  EXPECT_NEAR(jacobian(2, 2), expect(2, 2), 1e-8);
  EXPECT_NEAR(jacobian(3, 2), expect(3, 2), 1e-8);
  EXPECT_NEAR(jacobian(4, 2), expect(4, 2), 1e-8);
  EXPECT_NEAR(jacobian(5, 2), expect(5, 2), 1e-8);
}

TEST_F(RobotReachTestVelocity, JacobianSicilianoTest3) {
  std::vector<double> q = {1.0, 0.5, -1.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  reach_lib::Point point = robot_reach_->getRobotCapsulesForVelocity()[2].p2_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(2, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect = jacobian_tcp_siciliano(q);

  // column 0
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);

  // column 1
  EXPECT_NEAR(jacobian(0, 1), expect(0, 1), 1e-8);
  EXPECT_NEAR(jacobian(1, 1), expect(1, 1), 1e-8);
  EXPECT_NEAR(jacobian(2, 1), expect(2, 1), 1e-8);
  EXPECT_NEAR(jacobian(3, 1), expect(3, 1), 1e-8);
  EXPECT_NEAR(jacobian(4, 1), expect(4, 1), 1e-8);
  EXPECT_NEAR(jacobian(5, 1), expect(5, 1), 1e-8);

  // column 2
  EXPECT_NEAR(jacobian(0, 2), expect(0, 2), 1e-8);
  EXPECT_NEAR(jacobian(1, 2), expect(1, 2), 1e-8);
  EXPECT_NEAR(jacobian(2, 2), expect(2, 2), 1e-8);
  EXPECT_NEAR(jacobian(3, 2), expect(3, 2), 1e-8);
  EXPECT_NEAR(jacobian(4, 2), expect(4, 2), 1e-8);
  EXPECT_NEAR(jacobian(5, 2), expect(5, 2), 1e-8);
}

TEST_F(RobotReachTestVelocity, JacobianSicilianoTest4) {
  std::vector<double> q = {-0.5, 1.0, 1.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  reach_lib::Point point = robot_reach_->getRobotCapsulesForVelocity()[2].p2_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(2, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect = jacobian_tcp_siciliano(q);

  // column 0
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);

  // column 1
  EXPECT_NEAR(jacobian(0, 1), expect(0, 1), 1e-8);
  EXPECT_NEAR(jacobian(1, 1), expect(1, 1), 1e-8);
  EXPECT_NEAR(jacobian(2, 1), expect(2, 1), 1e-8);
  EXPECT_NEAR(jacobian(3, 1), expect(3, 1), 1e-8);
  EXPECT_NEAR(jacobian(4, 1), expect(4, 1), 1e-8);
  EXPECT_NEAR(jacobian(5, 1), expect(5, 1), 1e-8);

  // column 2
  EXPECT_NEAR(jacobian(0, 2), expect(0, 2), 1e-8);
  EXPECT_NEAR(jacobian(1, 2), expect(1, 2), 1e-8);
  EXPECT_NEAR(jacobian(2, 2), expect(2, 2), 1e-8);
  EXPECT_NEAR(jacobian(3, 2), expect(3, 2), 1e-8);
  EXPECT_NEAR(jacobian(4, 2), expect(4, 2), 1e-8);
  EXPECT_NEAR(jacobian(5, 2), expect(5, 2), 1e-8);
}

TEST_F(RobotReachTestVelocity, JacobianSicilianoTest5) {
  std::vector<double> q = {-1.0, -1.0, -1.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  reach_lib::Point point = robot_reach_->getRobotCapsulesForVelocity()[2].p2_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(2, point);
  Eigen::Matrix<double, 6, Eigen::Dynamic> expect = jacobian_tcp_siciliano(q);

  // column 0
  EXPECT_NEAR(jacobian(0, 0), expect(0, 0), 1e-8);
  EXPECT_NEAR(jacobian(1, 0), expect(1, 0), 1e-8);
  EXPECT_NEAR(jacobian(2, 0), expect(2, 0), 1e-8);
  EXPECT_NEAR(jacobian(3, 0), expect(3, 0), 1e-8);
  EXPECT_NEAR(jacobian(4, 0), expect(4, 0), 1e-8);
  EXPECT_NEAR(jacobian(5, 0), expect(5, 0), 1e-8);

  // column 1
  EXPECT_NEAR(jacobian(0, 1), expect(0, 1), 1e-8);
  EXPECT_NEAR(jacobian(1, 1), expect(1, 1), 1e-8);
  EXPECT_NEAR(jacobian(2, 1), expect(2, 1), 1e-8);
  EXPECT_NEAR(jacobian(3, 1), expect(3, 1), 1e-8);
  EXPECT_NEAR(jacobian(4, 1), expect(4, 1), 1e-8);
  EXPECT_NEAR(jacobian(5, 1), expect(5, 1), 1e-8);

  // column 2
  EXPECT_NEAR(jacobian(0, 2), expect(0, 2), 1e-8);
  EXPECT_NEAR(jacobian(1, 2), expect(1, 2), 1e-8);
  EXPECT_NEAR(jacobian(2, 2), expect(2, 2), 1e-8);
  EXPECT_NEAR(jacobian(3, 2), expect(3, 2), 1e-8);
  EXPECT_NEAR(jacobian(4, 2), expect(4, 2), 1e-8);
  EXPECT_NEAR(jacobian(5, 2), expect(5, 2), 1e-8);
}

}  // namespace safety_shield

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}