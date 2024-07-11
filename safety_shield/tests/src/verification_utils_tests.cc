#include <gtest/gtest.h>
#include "safety_shield/verification_utils.h"

namespace safety_shield {
TEST(VerifyTest, CapsuleCollisionTest) {
  reach_lib::Capsule cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 1.0);
  reach_lib::Capsule cap2(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 1.0);
  reach_lib::Capsule cap3(reach_lib::Point(2.01, 0.0, 0.0), reach_lib::Point(2.01, 0.0, 1.0), 1.0);
  EXPECT_TRUE(capsuleCollisionCheck(cap1, cap2));
  EXPECT_FALSE(capsuleCollisionCheck(cap1, cap3));
}

TEST(VerifyTest, RobotHumanCollisionTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap1(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps = {h_cap1, h_cap2};
  EXPECT_FALSE(robotHumanCollision(r_caps, h_caps));
}

TEST(VerifyTest, FindAllHumanRobotContactsTest) {
  reach_lib::Capsule r_cap0(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap0, r_cap1};
  reach_lib::Capsule h_cap0(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.5);
  reach_lib::Capsule h_cap1(reach_lib::Point(0.0, 0.0, -0.5), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps = {h_cap0, h_cap1};
  std::map<int, std::vector<int>> human_robot_contacts = findAllHumanRobotContacts(h_caps, r_caps);
  EXPECT_TRUE(human_robot_contacts.find(0) != human_robot_contacts.end());
  EXPECT_TRUE(human_robot_contacts.find(1) != human_robot_contacts.end());
  EXPECT_EQ(human_robot_contacts[0].size(), 1);
  EXPECT_EQ(human_robot_contacts[1].size(), 1);
  // Human capsule 0 should intersect with robot capsule 1
  EXPECT_EQ(human_robot_contacts[0][0], 1);
  // Human capsule 1 should intersect with robot capsule 0
  EXPECT_EQ(human_robot_contacts[1][0], 0);
}
} // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}