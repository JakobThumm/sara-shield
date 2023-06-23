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

TEST_F(VerifyIsoTest, BuildContactMapsTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap1(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, -1.0), 0.5);
  reach_lib::Capsule h_cap3(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps = {h_cap1, h_cap2, h_cap3};
  reach_lib::AABB env1({-1.0, -1.0, 1.0}, {1.0, 1.0, 1.5});
  reach_lib::AABB env2({-1.0, -1.0, 1.9}, {1.0, 1.0, 2.0});
  std::vector<reach_lib::AABB> envs = {env1, env2};
  std::unordered_map<int, std::vector<int>> robot_collision_map;
  std::unordered_map<int, std::vector<int>> environment_collision_map;
  verify_iso_.build_contact_maps(r_caps, h_caps, envs, robot_collision_map, environment_collision_map);
  EXPECT_EQ(robot_collision_map.size(), 2);
  // h_cap1 collides with r_cap1 and r_cap2
  EXPECT_EQ(robot_collision_map[0].size(), 2);
  EXPECT_EQ(robot_collision_map[0][0], 0);
  EXPECT_EQ(robot_collision_map[0][1], 1);
  // h_cap2 doesn't collide with any robot capsule
  EXPECT_TRUE(robot_collision_map.find(1) == robot_collision_map.end());
  // h_cap3 collides with r_cap1
  EXPECT_EQ(robot_collision_map[2].size(), 1);
  EXPECT_EQ(robot_collision_map[2][0], 0);
  // h_cap1 collides with env1
  EXPECT_EQ(environment_collision_map[0].size(), 1);
  EXPECT_EQ(environment_collision_map[0][0], 0);
  // h_cap2 doesn't collide with any environment element
  EXPECT_TRUE(environment_collision_map.find(1) == environment_collision_map.end());
  // h_cap3 doesn't collide with any environment element
  EXPECT_TRUE(environment_collision_map.find(2) == environment_collision_map.end());
}

TEST_F(VerifyIsoTest, SelfConstrainedCollisionCheckTest0) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 1.0, 0.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 1.0, 0.0), 0.1);
  reach_lib::Capsule r_cap3(reach_lib::Point(1.0, 1.0, 0.0), reach_lib::Point(1.0, 1.1, 0.0), 0.1);
  reach_lib::Capsule r_cap4(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap5(reach_lib::Point(-0.5, 0.0, 0.5), reach_lib::Point(0.5, 0.0, 0.5), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2, r_cap3, r_cap4, r_cap5};
  std::unordered_map<int, std::set<int>> unclampable_enclosures_map;
  unclampable_enclosures_map[1] = {2};
  unclampable_enclosures_map[2] = {1};
  double d_human = 0.1;
  std::vector<int> robot_collisions_1 = {0, 1, 2};
  EXPECT_FALSE(
      verify_iso_.self_constrained_collision_check(robot_collisions_1, unclampable_enclosures_map, r_caps, d_human));
  std::vector<int> robot_collisions_2 = {0, 3};
  EXPECT_TRUE(
      verify_iso_.self_constrained_collision_check(robot_collisions_2, unclampable_enclosures_map, r_caps, d_human));
  std::vector<int> robot_collisions_3 = {3, 4};
  EXPECT_TRUE(
      verify_iso_.self_constrained_collision_check(robot_collisions_3, unclampable_enclosures_map, r_caps, d_human));
  d_human = 0.3;
  std::vector<int> robot_collisions_4 = {0, 4};
  EXPECT_TRUE(
      verify_iso_.self_constrained_collision_check(robot_collisions_4, unclampable_enclosures_map, r_caps, d_human));
  d_human = 0.29;
  std::vector<int> robot_collisions_5 = {0, 4};
  EXPECT_FALSE(
      verify_iso_.self_constrained_collision_check(robot_collisions_5, unclampable_enclosures_map, r_caps, d_human));
}

TEST_F(VerifyIsoTest, CalculateNormalVectorTest) {
  reach_lib::AABB environment_element({-1.0, -0.05, -1.0}, {1.0, 0.0, 1.0});
  reach_lib::Capsule robot_capsule0(reach_lib::Point(0.0, 1.0, -1.0), reach_lib::Point(0.0, 1.0, 1.0), 0.1);
  Eigen::Vector3d expected_normal0(0.0, 1.0, 0.0);
  reach_lib::Capsule robot_capsule1(reach_lib::Point(0.5, 0.2, 0.0), reach_lib::Point(0.5, 1.0, 0.0), 0.1999);
  Eigen::Vector3d expected_normal1(0.0, 1.0, 0.0);
  reach_lib::Capsule robot_capsule2(reach_lib::Point(0.5, -0.2, 0.0), reach_lib::Point(0.5, 0.2, 0.0), 0.1);
  // EXPECT FALSE
  reach_lib::Capsule robot_capsule3(reach_lib::Point(2.0, 0.0, 0.0), reach_lib::Point(0.0, 2.0, 0.0), 0.1);
  Eigen::Vector3d expected_normal3(sqrt(0.5), sqrt(0.5), 0.0);
  reach_lib::Capsule robot_capsule4(reach_lib::Point(1.5, 0.5, 1.5), reach_lib::Point(2.0, 1.0, 2.0), 0.1);
  Eigen::Vector3d expected_normal4(sqrt(1.0/3.0), sqrt(1.0/3.0), sqrt(1.0/3.0));
  Eigen::Vector3d normal;
  EXPECT_TRUE(verify_iso_.calculate_normal_vector(robot_capsule0, environment_element, normal));
  double tol = 1e-6;
  EXPECT_NEAR(normal.x(), expected_normal0.x(), tol);
  EXPECT_NEAR(normal.y(), expected_normal0.y(), tol);
  EXPECT_NEAR(normal.z(), expected_normal0.z(), tol);
  EXPECT_TRUE(verify_iso_.calculate_normal_vector(robot_capsule1, environment_element, normal));
  EXPECT_NEAR(normal.x(), expected_normal1.x(), tol);
  EXPECT_NEAR(normal.y(), expected_normal1.y(), tol);
  EXPECT_NEAR(normal.z(), expected_normal1.z(), tol);
  EXPECT_FALSE(verify_iso_.calculate_normal_vector(robot_capsule2, environment_element, normal));
  EXPECT_TRUE(verify_iso_.calculate_normal_vector(robot_capsule3, environment_element, normal));
  EXPECT_NEAR(normal.x(), expected_normal3.x(), tol);
  EXPECT_NEAR(normal.y(), expected_normal3.y(), tol);
  EXPECT_NEAR(normal.z(), expected_normal3.z(), tol);
  EXPECT_TRUE(verify_iso_.calculate_normal_vector(robot_capsule4, environment_element, normal));
  EXPECT_NEAR(normal.x(), expected_normal4.x(), tol);
  EXPECT_NEAR(normal.y(), expected_normal4.y(), tol);
  EXPECT_NEAR(normal.z(), expected_normal4.z(), tol);
}

}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}