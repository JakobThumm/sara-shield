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

TEST_F(RobotReachTestVelocity, JacobianTest0) {
    robot_reach_->velocityOfMotion(motion_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(0);
    Eigen::Matrix<double, 6, 1> expect;
    expect << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    std::cout << "jacobian_1 is" << std::endl;
    std::cout << jacobian << std::endl;
    EXPECT_TRUE(jacobian.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest1) {
    robot_reach_->velocityOfMotion(motion_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(1);
    Eigen::Matrix<double, 6, 2> expect;
    expect <<   0.00000, 0.00000,
                0.00000, 0.00000,
                0.00000, 0.00000,
                0.00000, -0.84147,
                0.00000, 0.54030,
                1.00000, -0.00000;
    std::cout << "jacobian_2 is" << std::endl;
    std::cout << jacobian << std::endl;
    EXPECT_TRUE(jacobian.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest2) {
    robot_reach_->velocityOfMotion(motion_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(2);
    Eigen::Matrix<double, 6, 3> expect;
    expect <<
        -0.26780, -0.07869, 0.00000,
        0.17195,  -0.12256, 0.00000,
        0.00000,  -0.31825, 0.00000,
        0.00000,  -0.84147, 0.84147,
        0.00000,  0.54030,  -0.54030,
        1.00000,  -0.00000, 0.00001;
    std::cout << "jacobian_3 is" << std::endl;
    std::cout << jacobian << std::endl;
    EXPECT_TRUE(jacobian.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest3) {
    robot_reach_->velocityOfMotion(motion_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(3);
    Eigen::Matrix<double, 6, 4> expect;
    expect <<
        -0.26780, -0.07869, 0.00000,  0.00000,
        0.17195,  -0.12256, 0.00000,  0.00000,
        0.00000,  -0.31825, 0.00000,  0.00000,
        0.00000,  -0.84147, 0.84147,  -0.45465,
        0.00000,  0.54030,  -0.54030, -0.70807,
        1.00000,  -0.00000, 0.00001,  0.54030;
        std::cout << "jacobian_4 is" << std::endl;
    std::cout << jacobian << std::endl;
    EXPECT_TRUE(jacobian.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest4) {
    robot_reach_->velocityOfMotion(motion_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(4);
    Eigen::Matrix<double, 6, 5> expect;
    expect <<
        -0.05189, 0.00853,  -0.08723, 0.00235,  0.00000,
        0.02923,  0.01329,  -0.13585, -0.00372, 0.00000,
        0.00000,  -0.05945, -0.25880, -0.00289, 0.00000,
        0.00000,  -0.84147, 0.84147,  -0.45465, -0.77095,
        0.00000,  0.54030,  -0.54030, -0.70807, 0.00909,
        1.00000,  -0.00000, 0.00001,  0.54030,  -0.63683;
        std::cout << "jacobian_5 is" << std::endl;
    std::cout << jacobian << std::endl;
    EXPECT_TRUE(jacobian.isApprox(expect));
}

TEST_F(RobotReachTestVelocity, JacobianTest5) {
    robot_reach_->velocityOfMotion(motion_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = robot_reach_->getJacobian(5);
    Eigen::Matrix<double, 6, 6> expect;
    expect <<
        -0.05189, 0.00853,  -0.08723, 0.00235,  0.00000,  0.00000,
        0.02923,  0.01329,  -0.13585, -0.00372, 0.00000,  0.00000,
        0.00000,  -0.05945, -0.25880, -0.00289, 0.00000,  0.00000,
        0.00000,  -0.84147, 0.84147,  -0.45465, -0.77095, 0.29873,
        0.00000,  0.54030,  -0.54030, -0.70807, 0.00909,  -0.87793,
        1.00000,  -0.00000, 0.00001,  0.54030,  -0.63683, -0.37416;
    std::cout << "jacobian_6 is" << std::endl;
    std::cout << jacobian << std::endl << std::endl;
    std::vector<reach_lib::Capsule> capsules = robot_reach_->getVelocityCapsules();
    std::vector<Eigen::Vector3d> z_vectors = robot_reach_->getZvectors();
    std::vector<Eigen::Matrix4d> transformation_matrices_q = robot_reach_->getTransformationMatricesQ();
    for(int i = 0; i < capsules.size(); ++i) {
        std::cout << "capsule_" << i << "_p1  = " << std::endl << "(" << capsules[i].p1_.x << ", " << capsules[i].p1_.y << ", " << capsules[i].p1_.z << ")" << std::endl;
        std::cout << "capsule_" << i << "_p2  = " << std::endl << "(" << capsules[i].p2_.x << ", " << capsules[i].p2_.y << ", " << capsules[i].p2_.z << ")" << std::endl;
        std::cout << std::endl;
    }
    for(int i = 0; i < z_vectors.size(); ++i) {
        std::cout << "z_vector_" << i << " = " << std::endl << z_vectors[i] << std::endl;
        std::cout << std::endl;
    }
    for(int i = 0; i < transformation_matrices_q.size(); ++i) {
        std::cout << "transformation_matrix_" << i << " = " << std::endl << transformation_matrices_q[i] << std::endl;
        std::cout << std::endl;
    }
    EXPECT_TRUE(jacobian.isApprox(expect));
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}