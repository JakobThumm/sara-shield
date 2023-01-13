#include <gtest/gtest.h>
#include <math.h> 

#include <Eigen/Dense>
#include "spdlog/spdlog.h" 

#include "robot_reach_fixture.h"
#include "safety_shield/robot_reach.h"

namespace safety_shield {

TEST_F(RobotReachTest, InitializationTest){
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(RobotReachTest, ForwardKinematicTest0){
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  robot_reach_->forwardKinematic(0.0, 0.0, result);
  Eigen::Matrix4d expect;
  expect <<   1, 0, 0, 0, 
              0, 0, -1, 0, 
              0, 1, 0, 0.1, 
              0, 0, 0, 1;
  EXPECT_TRUE(result.isApprox(expect));
}

TEST_F(RobotReachTest, ForwardKinematicTest1){
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  robot_reach_->forwardKinematic(M_PI/2, 0.0, result);
  Eigen::Matrix4d expect;
  expect << 0.0, -1.0, 0.0, 0.0, 
            0.0, 0.0, -1.0, 0.0, 
            1.0, 0.0, 0.0, 0.1, 
            0.0, 0.0, 0.0, 1.0;
  EXPECT_TRUE(result.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest1) {
    std::vector<Eigen::Matrix4d> transformation_matrices_q;
    Eigen::Matrix4d T = robot_reach_->getTransformationMatrices()[0];
    transformation_matrices_q.push_back(T);
    std::vector<double> q = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    Motion motion(0.0, q);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
    for (int j = 0; j < 1; j++) {
        jacobian = robot_reach_->allKinematics(motion, transformation_matrices_q);
    }
    Eigen::Matrix<double, 6, 1> expect;
    expect << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(jacobian.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest2) {
    std::vector<Eigen::Matrix4d> transformation_matrices_q;
    Eigen::Matrix4d T = robot_reach_->getTransformationMatrices()[0];
    transformation_matrices_q.push_back(T);
    std::vector<double> q = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    Motion motion(0.0, q);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
    for (int j = 0; j < 2; j++) {
        jacobian = robot_reach_->allKinematics(motion, transformation_matrices_q);
    }
    Eigen::Matrix<double, 6, 2> expect;
    expect <<   0.0, -0.08,
                0.0, -0.13,
                0.0, 0.0,
                0.0, -0.84,
                0.0, 0.54,
                1.0, 0.0;
    //std::cout << "jacobian is" << std::endl;
    //std::cout << jacobian << std::endl;
    EXPECT_TRUE(jacobian.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest6) {
    std::vector<Eigen::Matrix4d> transformation_matrices_q;
    Eigen::Matrix4d T = robot_reach_->getTransformationMatrices()[0];
    transformation_matrices_q.push_back(T);
    std::vector<double> q = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    Motion motion(0.0, q);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
    for (int j = 0; j < 6; j++) {
        jacobian = robot_reach_->allKinematics(motion, transformation_matrices_q);
    }
    Eigen::Matrix<double, 6, 6> expect;
    expect <<   0.00, -0.08, 0.18,  0.25,  -0.27, 0.63,
        0.00, -0.13, 0.29,  -0.16, 0.59,  0.14,
        0.00,  0.00,  -0.29, 0.00,  -0.16, -0.21,
        0.00,  -0.84, 0.84,  -0.00, 0.91,  0.35,
        0.00,  0.54,  -0.54, 0.00,  0.42,  -0.77,
        1.00,  -0.00, 0.00,  1.00,  -0.00, 0.54;
    std::cout << "jacobian is" << std::endl;
    std::cout << jacobian << std::endl;
    EXPECT_TRUE(jacobian.isApprox(expect));
    }

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}