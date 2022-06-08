#include <gtest/gtest.h>

#include "human_reach_fixture.h"
#include "safety_shield/human_reach.h"

namespace safety_shield {

TEST_F(HumanReachTest, InitializationTest){
  EXPECT_EQ(0, 0);
}

TEST_F(HumanReachTest, HumanReachMeasTest){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double time = 1.0;
  human_reach_->measurement(human_joint_pos, time);
  EXPECT_EQ(human_reach_->getLastMeasTimestep(), 1.0);
  EXPECT_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}