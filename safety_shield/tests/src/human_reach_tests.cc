#include <gtest/gtest.h>

#include "human_reach_fixture.h"
#include "safety_shield/human_reach.h"

namespace safety_shield {

TEST_F(HumanReachTest, InitializationTest){
  EXPECT_DOUBLE_EQ(0, 0);
}

TEST_F(HumanReachTest, HumanReachMeasTest){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double time = 1.0;
  human_reach_->measurement(human_joint_pos, time);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachMeasVel0Test){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double time = 1.0;
  human_reach_->measurement(human_joint_pos, time);
  reach_lib::Point q(1, 2, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  time = 2.0;
  human_reach_->measurement(human_joint_pos, time);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachMeasVel1Test){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double time = 1.0;
  human_reach_->measurement(human_joint_pos, time);
  reach_lib::Point q(2, 3, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  time = 2.0;
  human_reach_->measurement(human_joint_pos, time);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachAnalysisVelTest){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double time = 1.0;
  human_reach_->measurement(human_joint_pos, time);
  double t_command = 1.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  std::vector<reach_lib::Capsule> v_cap = human_reach_->getArticulatedVelCapsules();
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.z, 3.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.z, 3.0);
  // radius = thickness + meas_err_pos + ((t_command-t_last) + t_break + t_delay) * v_max 
  EXPECT_DOUBLE_EQ(v_cap[0].r_, 0.21);
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}