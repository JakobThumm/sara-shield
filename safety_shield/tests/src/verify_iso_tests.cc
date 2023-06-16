#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>

#include "reach_lib.hpp"
#include "safety_shield/verify_iso.h"
#include "spdlog/spdlog.h"
#include "verify_iso_fixture.h"

namespace safety_shield {

TEST_F(VerifyIsoTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(VerifyIsoTest, CapsuleCollisionTest) {
  reach_lib::Capsule cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 1.0);
  reach_lib::Capsule cap2(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 1.0);
  reach_lib::Capsule cap3(reach_lib::Point(2.01, 0.0, 0.0), reach_lib::Point(2.01, 0.0, 1.0), 1.0);
  EXPECT_TRUE(verify_iso_.capsuleCollisionCheck(cap1, cap2));
  EXPECT_FALSE(verify_iso_.capsuleCollisionCheck(cap1, cap3));
}

TEST_F(VerifyIsoTest, RobotHumanCollisionTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap1(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps = {h_cap1, h_cap2};
  EXPECT_FALSE(verify_iso_.robotHumanCollision(r_caps, h_caps));
}

TEST_F(VerifyIsoTest, VerifyHumanReachTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap11(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap12(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps1 = {h_cap11, h_cap12};
  reach_lib::Capsule h_cap21(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap22(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps2 = {h_cap21, h_cap22};
  std::vector<std::vector<reach_lib::Capsule>> h_caps_list = {h_caps1, h_caps2};
  EXPECT_TRUE(verify_iso_.verify_human_reach(r_caps, h_caps_list));
}

TEST_F(VerifyIsoTest, FindHumanRobotContactTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap1(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap3(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.1), 0.5);
  std::vector<int> human_robot_collisions_1 = verify_iso_.find_human_robot_contact(h_cap1, r_caps);
  EXPECT_EQ(human_robot_collisions_1.size(), 0);
  std::vector<int> human_robot_collisions_2 = verify_iso_.find_human_robot_contact(h_cap2, r_caps);
  EXPECT_EQ(human_robot_collisions_2.size(), 2);
  EXPECT_EQ(human_robot_collisions_2[0], 0);
  EXPECT_EQ(human_robot_collisions_2[1], 1);
  std::vector<int> human_robot_collisions_3 = verify_iso_.find_human_robot_contact(h_cap3, r_caps);
  EXPECT_EQ(human_robot_collisions_3.size(), 1);
  EXPECT_EQ(human_robot_collisions_3[0], 0);
}

TEST_F(VerifyIsoTest, FindHumanEnvironmentContactTest) {
  reach_lib::Capsule h_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule h_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  reach_lib::AABB env1({-1.0, -1.0, -0.5}, {1.0, 1.0, 0.0});
  reach_lib::AABB env2({-1.0, -1.0, -1.0}, {1.0, -0.5, 1.0});
  std::vector<reach_lib::AABB> envs = {env1, env2};
  std::vector<int> human_env_collisions_1 = verify_iso_.find_human_environment_contact(h_cap1, envs);
  EXPECT_EQ(human_env_collisions_1.size(), 1);
  EXPECT_EQ(human_env_collisions_1[0], 0);
  std::vector<int> human_env_collisions_2 = verify_iso_.find_human_environment_contact(h_cap2, envs);
  EXPECT_EQ(human_env_collisions_2.size(), 0);
}
}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}