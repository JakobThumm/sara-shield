#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Dense>
#include <cmath>

#include "robot_reach_fixture.h"
#include "safety_shield/robot_reach.h"
#include "spdlog/spdlog.h"

namespace safety_shield {

TEST_F(RobotReachTest, InitializationTest) {
  EXPECT_NEAR(0, 0.0, 1e-8);
  robot_reach_->reset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

TEST_F(RobotReachTest, CrossMatrixTest) {
  Eigen::Vector3d vec(1.0, 2.0, 3.0);
  Eigen::Matrix3d result = robot_reach_->getCrossProductAsMatrix(vec);
  Eigen::Matrix3d expect;
  expect << 0, -3.0, 2.0, 3.0, 0, -1.0, -2.0, 1.0, 0;
  EXPECT_TRUE(result.isApprox(expect));
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(0, point);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(0, point);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(0, point);
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
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->calculateVelocityOfCapsule(0, q_dot);
  RobotReach::SE3Vel vel1 = capsule_velocity.v1;
  RobotReach::SE3Vel vel2 = capsule_velocity.v2;
  EXPECT_NEAR(vel1.v(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.v(1), 0.0, 1e-8);
  EXPECT_NEAR(vel1.v(2), 0.0, 1e-8);
  EXPECT_NEAR(vel1.w(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.w(1), -1.0, 1e-8);
  EXPECT_NEAR(vel1.w(2), 0.0, 1e-8);
  EXPECT_NEAR(vel2.v(0), 0.1, 1e-8);
  EXPECT_NEAR(vel2.v(1), 0.0, 1e-8);
  EXPECT_NEAR(vel2.v(2), 0.0, 1e-8);
  EXPECT_NEAR(vel2.w(0), 0.0, 1e-8);
  EXPECT_NEAR(vel2.w(1), -1.0, 1e-8);
  EXPECT_NEAR(vel2.w(2), 0.0, 1e-8);
}

TEST_F(RobotReachTest, GetVelocityOfCapsuleTest1) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{M_PI/2.0});
  std::vector<double> q_dot{1.0};
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->calculateVelocityOfCapsule(0, q_dot);
  RobotReach::SE3Vel vel1 = capsule_velocity.v1;
  RobotReach::SE3Vel vel2 = capsule_velocity.v2;
  EXPECT_NEAR(vel1.v(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.v(1), 0.0, 1e-8);
  EXPECT_NEAR(vel1.v(2), 0.0, 1e-8);
  EXPECT_NEAR(vel1.w(0), 0.0, 1e-8);
  EXPECT_NEAR(vel1.w(1), -1.0, 1e-8);
  EXPECT_NEAR(vel1.w(2), 0.0, 1e-8);
  EXPECT_NEAR(vel2.v(0), 0.0, 1e-8);
  EXPECT_NEAR(vel2.v(1), 0.0, 1e-8);
  EXPECT_NEAR(vel2.v(2), 0.1, 1e-8);
  EXPECT_NEAR(vel2.w(0), 0.0, 1e-8);
  EXPECT_NEAR(vel2.w(1), -1.0, 1e-8);
  EXPECT_NEAR(vel2.w(2), 0.0, 1e-8);
}

TEST_F(RobotReachTest, ApproximateVelOfCapsuleTest0) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{0.0});
  std::vector<double> q_dot{1.0};
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->calculateVelocityOfCapsule(0, q_dot);
  double v_approx = robot_reach_->approximateVelOfCapsule(0, capsule_velocity.v2.v, capsule_velocity.v2.w);
  EXPECT_NEAR(v_approx, 0.11, 1e-8);
}

TEST_F(RobotReachTest, ApproximateVelOfCapsuleTest1) {
  robot_reach_->calculateAllTransformationMatricesAndCapsules(std::vector<double>{M_PI/2.0});
  std::vector<double> q_dot{1.0};
  RobotReach::CapsuleVelocity capsule_velocity = robot_reach_->calculateVelocityOfCapsule(0, q_dot);
  double v_approx = robot_reach_->approximateVelOfCapsule(0, capsule_velocity.v2.v, capsule_velocity.v2.w);
  EXPECT_NEAR(v_approx, 0.11, 1e-8);
}

TEST_F(RobotReachTest, CalculateMaxVelErrorTest1) {
  double d_1 = 0.1;
  std::vector<double> dq_max = {1.0};
  std::vector<double> ddq_max = {10.0};
  std::vector<double> dddq_max = {200.0};
  double dt = 0.05;
  double epsilon_v = dt * dt / 8.0 * d_1 * (dddq_max[0] + ddq_max[0] * dq_max[0] + dq_max[0] * (ddq_max[0] + dq_max[0] * dq_max[0]));
  std::vector<double> velocity_errors = robot_reach_->calculateMaxVelErrors(dt, dq_max, ddq_max, dddq_max); 
  EXPECT_TRUE(velocity_errors.size() == 1);
  EXPECT_NEAR(velocity_errors[0], epsilon_v, 1e-8);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(2, point);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(2, point);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(2, point);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(2, point);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(2, point);
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->calculateJacobian(2, point);
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

TEST_F(RobotReachTestVelocity, maxVelocityOfMotionTest) {
  std::vector<double> q_1 = {0.0, 0.0, 0.0};
  std::vector<double> q_2 = {1.0, 0.0, 0.0};
  std::vector<double> dq_1 = {1.0, 0.0, 0.0};
  std::vector<double> dq_2 = {1.0, 1.0, 1.0};
  Motion m_1 = Motion(0.0, q_1, dq_1, 0.0);
  Motion m_2 = Motion(0.0, q_1, dq_2, 0.0);
  Motion m_3 = Motion(0.0, q_2, dq_1, 0.0);
  Motion m_4 = Motion(0.0, q_2, dq_2, 0.0);
  double max_vel_1 = robot_reach_->maxVelocityOfMotion(m_1);
  double max_vel_2 = robot_reach_->maxVelocityOfMotion(m_2);
  double max_vel_3 = robot_reach_->maxVelocityOfMotion(m_3);
  double max_vel_4 = robot_reach_->maxVelocityOfMotion(m_4);
  // length = 3 * 1 + 1 = 4. v = 1.0 * 4.0 = 4.0
  EXPECT_NEAR(max_vel_1, 4.0, 1e-8);
  // v = 1*1 + 2*1 + 3*2 = 9.0
  EXPECT_NEAR(max_vel_2, 9.0, 1e-8);
  EXPECT_NEAR(max_vel_3, 4.0, 1e-8);
  EXPECT_NEAR(max_vel_4, 9.0, 1e-8);
}

TEST_F(RobotReachTestVelocity, maxVelocityOfMotionExactTest) {
  robot_reach_->setVelocityMethod(safety_shield::RobotReach::VelocityMethod::EXACT);
  std::vector<double> q_1 = {0.0, 0.0, 0.0};
  std::vector<double> q_2 = {1.0, 0.0, 0.0};
  std::vector<double> dq_1 = {1.0, 0.0, 0.0};
  std::vector<double> dq_2 = {1.0, 1.0, 1.0};
  Motion m_1 = Motion(0.0, q_1, dq_1, 0.0);
  Motion m_2 = Motion(0.0, q_1, dq_2, 0.0);
  Motion m_3 = Motion(0.0, q_2, dq_1, 0.0);
  Motion m_4 = Motion(0.0, q_2, dq_2, 0.0);
  double max_vel_1 = robot_reach_->maxVelocityOfMotion(m_1);
  double max_vel_2 = robot_reach_->maxVelocityOfMotion(m_2);
  double max_vel_3 = robot_reach_->maxVelocityOfMotion(m_3);
  double max_vel_4 = robot_reach_->maxVelocityOfMotion(m_4);
  // length = 3 * 1 + 1 = 4. v = 1.0 * 4.0 = 4.0
  EXPECT_NEAR(max_vel_1, 4.0, 1e-8);
  // v = 1*1 + 2*1 + 3*2 = 9.0
  EXPECT_NEAR(max_vel_2, 9.0, 1e-8);
  EXPECT_NEAR(max_vel_3, 4.0, 1e-8);
  EXPECT_NEAR(max_vel_4, 9.0, 1e-8);
}

TEST_F(RobotReachTestVelocity, calculateAllCapsuleVelocitiesTest) {
  std::vector<double> q = {0.0, 0.0, 0.0};
  std::vector<double> dq = {1.0, 1.0, 1.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  std::vector<safety_shield::RobotReach::CapsuleVelocity> capsuleVels = robot_reach_->calculateAllCapsuleVelocities(dq);
  // Cap 1
  // v1 = [0, 0, 0], w1 = [0, 0, 1]
  EXPECT_NEAR(capsuleVels[0].v1.v[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v1.v[1], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v1.v[2], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v1.w[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v1.w[1], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v1.w[2], 1.0, 1e-8);
  // v2 = [0, 1, 0], w2 = [0, 0, 1]
  EXPECT_NEAR(capsuleVels[0].v2.v[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v2.v[1], 1.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v2.v[2], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v2.w[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v2.w[1], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[0].v2.w[2], 1.0, 1e-8);
  // Cap 2
  // v1 = [0, 1, 0], w1 = [0, 0, 2]
  EXPECT_NEAR(capsuleVels[1].v1.v[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v1.v[1], 1.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v1.v[2], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v1.w[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v1.w[1], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v1.w[2], 2.0, 1e-8);
  // v2 = [0, 3, 0], w2 = [0, 0, 2]
  EXPECT_NEAR(capsuleVels[1].v2.v[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v2.v[1], 3.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v2.v[2], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v2.w[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v2.w[1], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[1].v2.w[2], 2.0, 1e-8);
  // Cap 3
  // v1 = [0, 3, 0], w1 = [0, 0, 3]
  EXPECT_NEAR(capsuleVels[2].v1.v[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v1.v[1], 3.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v1.v[2], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v1.w[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v1.w[1], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v1.w[2], 3.0, 1e-8);
  // v2 = [0, 6, 0], w2 = [0, 0, 3]
  EXPECT_NEAR(capsuleVels[2].v2.v[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v2.v[1], 6.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v2.v[2], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v2.w[0], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v2.w[1], 0.0, 1e-8);
  EXPECT_NEAR(capsuleVels[2].v2.w[2], 3.0, 1e-8);
}

TEST_F(RobotReachTestInertiaMatrix, InertiaMatrixTest0) {
  std::vector<double> q = {M_PI_4, M_PI_4};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> inertia_matrix = robot_reach_->calculateAllInertiaMatrices();
  // First inertia matrix should only depend on the first joint velocity.
  EXPECT_NEAR(inertia_matrix[0](0, 1), 0.0, 1e-8);
  EXPECT_NEAR(inertia_matrix[0](1, 0), 0.0, 1e-8);
  EXPECT_NEAR(inertia_matrix[0](1, 1), 0.0, 1e-8);
  // Second inertia matrix can be taken from Siciliano.
  double a_1 = 1.0;
  double a_2 = 2.0;
  double l_1 = 0.5;
  double l_2 = 1.0;
  double I_1 = 0.03;
  double I_2 = 0.02;
  double m_1 = 3.0;
  double m_2 = 2.0;
  double B_11 = I_1 + m_1 * l_1 * l_1 + I_2 + m_2 * (l_2 * l_2 + a_1 * a_1 + 2 * a_1 * l_2 * cos(q[1]));
  double B_12 = I_2 + m_2 * (l_2 * l_2 + a_1 * l_2 * cos(q[1]));
  double B_22 = I_2 + m_2 * l_2 * l_2;
  double B_11_predicted = inertia_matrix[1](0, 0);
  double B_12_predicted = inertia_matrix[1](0, 1);
  double B_21_predicted = inertia_matrix[1](1, 0);
  double B_22_predicted = inertia_matrix[1](1, 1);
  EXPECT_NEAR(B_11, B_11_predicted, 1e-8);
  EXPECT_NEAR(B_12, B_12_predicted, 1e-8);
  EXPECT_NEAR(B_12, B_21_predicted, 1e-8);
  EXPECT_NEAR(B_22, B_22_predicted, 1e-8);
}

/*
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RobotReach::calculateLinkIntertiaMatrix(const int i, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& robot_inertia_matrix) const {
  if (robot_inertia_matrix.rows() != nb_joints_ || robot_inertia_matrix.cols() != nb_joints_) {
    throw std::invalid_argument("Robot inertia matrix has wrong dimensions.");
  }
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> E_i = Eigen::MatrixXd::Zero(nb_joints_, nb_joints_);
  E_i.block(0, 0, i + 1, i + 1) = Eigen::MatrixXd::Identity(i + 1, i + 1);
  return E_i * robot_inertia_matrix * E_i;
}
*/
TEST_F(RobotReachTestVelocity, LinkInertiaMatrixTest0) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> robot_inertia_matrix(3, 3);
  robot_inertia_matrix << 1.0, 2.0, 3.0,
                           2.0, 4.0, 5.0,
                           3.0, 5.0, 6.0;
  auto inertia_matrix_0 = robot_reach_->calculateLinkIntertiaMatrix(0, robot_inertia_matrix);
  auto inertia_matrix_1 = robot_reach_->calculateLinkIntertiaMatrix(1, robot_inertia_matrix);
  auto inertia_matrix_2 = robot_reach_->calculateLinkIntertiaMatrix(2, robot_inertia_matrix);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(1, 1), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(2, 1), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_0(2, 2), 0.0);
  // Second link
  EXPECT_DOUBLE_EQ(inertia_matrix_1(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(1, 0), 2.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(1, 1), 4.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(2, 1), 0.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_1(2, 2), 0.0);
  // Third link
  EXPECT_DOUBLE_EQ(inertia_matrix_2(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(0, 2), 3.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(1, 0), 2.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(1, 1), 4.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(1, 2), 5.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(2, 0), 3.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(2, 1), 5.0);
  EXPECT_DOUBLE_EQ(inertia_matrix_2(2, 2), 6.0);
}

TEST_F(RobotReachSchunkTest, MaxReflectedMassTest) {
  std::vector<double> q = {0.2, 0.4, -0.32, 0.86, 0.926, 1.3};  // Some value that is no singularity
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  std::vector<double> max_reflected_masses = robot_reach_->calculateAllMaxReflectedMasses();
  double sum_of_link_masses = 13.79;
  EXPECT_TRUE(max_reflected_masses[max_reflected_masses.size()-1] < sum_of_link_masses);
}

TEST_F(RobotReachSchunkTest, CalculateRobotLinkReflectedMassesPerTimeIntervalTest) {
  std::vector<double> q = {0.2, 0.4, -0.32, 0.86, 0.926, 1.3};  // Some value that is no singularity
  std::vector<double> dq = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Motion m(0.0, q, dq, 0.0);
  std::vector<Motion> motion_vec = {m};
  std::vector<std::vector<double>> reflected_masses = robot_reach_->calculateRobotLinkReflectedMassesPerTimeInterval(motion_vec);
  EXPECT_EQ(reflected_masses.size(), 1);
  EXPECT_EQ(reflected_masses[0].size(), 6);
  for (int i = 0; i < 6; i++) {
    EXPECT_TRUE(reflected_masses[0][i] >= 0);
    // TODO: Test the actual functionality!
  }
}

/*
TEST_F(RobotReachPandaTest, MaxReflectedMassTest) {
  std::vector<double> q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  robot_reach_->calculateAllTransformationMatricesAndCapsules(q);
  std::vector<double> max_reflected_masses = robot_reach_->calculateAllMaxReflectedMasses();
  double sum_of_link_masses = 16.691901;
  EXPECT_NEAR(max_reflected_masses[max_reflected_masses.size()-1], sum_of_link_masses, 1e-8);
}
*/

/// time interval method is equal to standard reach if reachability set interval size is equal or larger to s_diff (with single joint robot)
TEST_F(RobotReachTestTimeIntervals, EqualityTest0) {
  Motion start_config(0, std::vector<double>{0.0}, 0);
  Motion goal_config(0.1, std::vector<double>{M_PI / 8.0}, 0.1);
  std::vector<double> alpha_i{1.0};
  double s_diff = 0.1;
  std::vector<reach_lib::Capsule> standardCapsules = robot_reach_single_joint_->reach(start_config, goal_config, s_diff, alpha_i);
  std::vector<std::vector<reach_lib::Capsule>> timeIntervalCapsules = robot_reach_single_joint_->reachTimeIntervals({start_config, goal_config}, alpha_i);
  EXPECT_TRUE(timeIntervalCapsules.size() == 1);
  reach_lib::Capsule result = timeIntervalCapsules[0][0];
  reach_lib::Capsule expect = standardCapsules[0];
  EXPECT_EQ(result.p1_.x, expect.p1_.x);
  EXPECT_EQ(result.p1_.y, expect.p1_.y);
  EXPECT_EQ(result.p1_.z, expect.p1_.z);
  EXPECT_EQ(result.p2_.x, expect.p2_.x);
  EXPECT_EQ(result.p2_.y, expect.p2_.y);
  EXPECT_EQ(result.p2_.z, expect.p2_.z);
  EXPECT_EQ(result.r_, expect.r_);
}


/// time interval method is equal to standard reach if reachability set interval size is equal or larger than s_diff (with siciliano robot)
TEST_F(RobotReachTestTimeIntervals, EqualityTest1) {
  Motion start_config(0, std::vector<double>{0.0, 0.0, 0.0}, 0);
  Motion goal_config(0.1, std::vector<double>{M_PI / 8.0, M_PI / 8.0, M_PI / 8.0}, 0.1);
  std::vector<double> alpha_i{1.0, 1.0, 1.0};
  double s_diff = 0.1;
  std::vector<reach_lib::Capsule> standardCapsules = robot_reach_siciliano_->reach(start_config, goal_config, s_diff, alpha_i);
  std::vector<safety_shield::Motion> interval_edges_motions = {start_config, goal_config};
  std::vector<std::vector<reach_lib::Capsule>> timeIntervalCapsules = robot_reach_siciliano_->reachTimeIntervals(interval_edges_motions, alpha_i);
  EXPECT_TRUE(timeIntervalCapsules.size() == 1);
  for (int i = 0; i < standardCapsules.size(); i++) {
    reach_lib::Capsule result = timeIntervalCapsules[0][i];
    reach_lib::Capsule expect = standardCapsules[i];
    EXPECT_EQ(result.p1_.x, expect.p1_.x);
    EXPECT_EQ(result.p1_.y, expect.p1_.y);
    EXPECT_EQ(result.p1_.z, expect.p1_.z);
    EXPECT_EQ(result.p2_.x, expect.p2_.x);
    EXPECT_EQ(result.p2_.y, expect.p2_.y);
    EXPECT_EQ(result.p2_.z, expect.p2_.z);
    EXPECT_EQ(result.r_, expect.r_);
  }
}

/// time interval method with two intervals is equal to standard reach of the two separate intervals (with single joint robot)
TEST_F(RobotReachTestTimeIntervals, EqualityTest2) {
  Motion start_config(0, std::vector<double>{0.0}, 0);
  Motion mid_config(0.05, std::vector<double>{M_PI / 16.0}, 0.05);
  Motion goal_config(0.1, std::vector<double>{M_PI / 8.0}, 0.1);
  std::vector<double> alpha_i{1.0};
  double s_diff = 0.1;
  std::vector<reach_lib::Capsule> standardCapsules = robot_reach_single_joint_->reach(start_config, goal_config, s_diff, alpha_i);
  std::vector<safety_shield::Motion> interval_edges_motions = {start_config, mid_config, goal_config};
  std::vector<std::vector<reach_lib::Capsule>> timeIntervalCapsules = robot_reach_single_joint_->reachTimeIntervals(interval_edges_motions, alpha_i);
  EXPECT_TRUE(timeIntervalCapsules.size() == 2);
  for (int i = 0; i < standardCapsules.size(); i++) {
    for (int j = 0; j < timeIntervalCapsules.size(); j++) {
      reach_lib::Capsule result = timeIntervalCapsules[j][i];
      reach_lib::Capsule expect = standardCapsules[i];
      // Rotation only around p1, so p1 should be equal.
      EXPECT_DOUBLE_EQ(result.p1_.x, expect.p1_.x);
      EXPECT_DOUBLE_EQ(result.p1_.y, expect.p1_.y);
      EXPECT_DOUBLE_EQ(result.p1_.z, expect.p1_.z);
      // p2 and r should be different.
      EXPECT_NE(result.p2_.x, expect.p2_.x);
      // Rotation around y
      EXPECT_DOUBLE_EQ(result.p2_.y, expect.p2_.y);
      EXPECT_NE(result.p2_.z, expect.p2_.z);
      EXPECT_NE(result.r_, expect.r_);
    }
  }
}

/// time interval method with two intervals is equal to standard reach of the two separate intervals (with single joint robot)
TEST_F(RobotReachTestTimeIntervals, IntervalTest1) {
  Motion start_config(0, std::vector<double>{0.0}, 0);
  Motion middle_config(0.05, std::vector<double>{M_PI / 16.0}, 0.05);
  Motion goal_config(0.1, std::vector<double>{M_PI / 8.0}, 0.1);
  std::vector<double> alpha_i{1.0};
  std::vector<reach_lib::Capsule> firstIntervalCapsules = robot_reach_single_joint_->reach(start_config, middle_config, 0.05, alpha_i);
  std::vector<reach_lib::Capsule> secondIntervalCapsules = robot_reach_single_joint_->reach(middle_config, goal_config, 0.05, alpha_i);
  std::vector<std::vector<reach_lib::Capsule>> timeIntervalCapsules = robot_reach_single_joint_->reachTimeIntervals({start_config, middle_config, goal_config}, alpha_i);
  EXPECT_TRUE(timeIntervalCapsules.size() == 2);

  /// equality of first interval
  reach_lib::Capsule result1 = timeIntervalCapsules[0][0];
  reach_lib::Capsule expect1 = firstIntervalCapsules[0];
  EXPECT_EQ(result1.p1_.x, expect1.p1_.x);
  EXPECT_EQ(result1.p1_.y, expect1.p1_.y);
  EXPECT_EQ(result1.p1_.z, expect1.p1_.z);
  EXPECT_EQ(result1.p2_.x, expect1.p2_.x);
  EXPECT_EQ(result1.p2_.y, expect1.p2_.y);
  EXPECT_EQ(result1.p2_.z, expect1.p2_.z);
  EXPECT_EQ(result1.r_, expect1.r_);

  /// equality of second interval
  reach_lib::Capsule result2 = timeIntervalCapsules[1][0];
  reach_lib::Capsule expect2 = secondIntervalCapsules[0];
  EXPECT_EQ(result2.p1_.x, expect2.p1_.x);
  EXPECT_EQ(result2.p1_.y, expect2.p1_.y);
  EXPECT_EQ(result2.p1_.z, expect2.p1_.z);
  EXPECT_EQ(result2.p2_.x, expect2.p2_.x);
  EXPECT_EQ(result2.p2_.y, expect2.p2_.y);
  EXPECT_EQ(result2.p2_.z, expect2.p2_.z);
  EXPECT_EQ(result2.r_, expect2.r_);
}

/// time interval method with two intervals is equal to standard reach of the two separate intervals (with siciliano robot)
TEST_F(RobotReachTestTimeIntervals, IntervalTest2) {
  Motion start_config(0, std::vector<double>{0.0, 0.0, 0.0}, 0);
  Motion middle_config(0.1, std::vector<double>{M_PI / 16.0, M_PI / 16.0, M_PI / 16.0}, 0.05);
  Motion goal_config(0.1, std::vector<double>{M_PI / 8.0, M_PI / 8.0, M_PI / 8.0}, 0.1);
  std::vector<double> alpha_i{1.0, 1.0, 1.0};
  std::vector<reach_lib::Capsule> firstIntervalCapsules = robot_reach_siciliano_->reach(start_config, middle_config, 0.05, alpha_i);
  std::vector<reach_lib::Capsule> secondIntervalCapsules = robot_reach_siciliano_->reach(middle_config, goal_config, 0.05, alpha_i);
  std::vector<std::vector<reach_lib::Capsule>> timeIntervalCapsules = robot_reach_siciliano_->reachTimeIntervals({start_config, middle_config, goal_config}, alpha_i);
  EXPECT_TRUE(timeIntervalCapsules.size() == 2);

  /// equality of first interval
  for (int i = 0; i < firstIntervalCapsules.size(); i++) {
    reach_lib::Capsule result = timeIntervalCapsules[0][i];
    reach_lib::Capsule expect = firstIntervalCapsules[i];
    EXPECT_EQ(result.p1_.x, expect.p1_.x);
    EXPECT_EQ(result.p1_.y, expect.p1_.y);
    EXPECT_EQ(result.p1_.z, expect.p1_.z);
    EXPECT_EQ(result.p2_.x, expect.p2_.x);
    EXPECT_EQ(result.p2_.y, expect.p2_.y);
    EXPECT_EQ(result.p2_.z, expect.p2_.z);
    EXPECT_EQ(result.r_, expect.r_);
  }

  /// equality of second interval
  for (int i = 0; i < secondIntervalCapsules.size(); i++) {
    reach_lib::Capsule result = timeIntervalCapsules[1][i];
    reach_lib::Capsule expect = secondIntervalCapsules[i];
    EXPECT_EQ(result.p1_.x, expect.p1_.x);
    EXPECT_EQ(result.p1_.y, expect.p1_.y);
    EXPECT_EQ(result.p1_.z, expect.p1_.z);
    EXPECT_EQ(result.p2_.x, expect.p2_.x);
    EXPECT_EQ(result.p2_.y, expect.p2_.y);
    EXPECT_EQ(result.p2_.z, expect.p2_.z);
    EXPECT_EQ(result.r_, expect.r_);
  }
}

}  // namespace safety_shield

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}