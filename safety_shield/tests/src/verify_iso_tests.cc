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

// Test this function
/*
void VerifyISO::build_contact_maps(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      std::unordered_map<int, std::vector<int>>& robot_collision_map,
      std::unordered_map<int, std::vector<int>>& environment_collision_map) {
  for (int i = 0; i < human_capsules.size(); i++) {
    // Collisions with robot links
    std::vector<int> robot_collisions = find_human_robot_contact(human_capsules[i], robot_capsules);
    if (robot_collisions.size() > 0) {
      if (robot_collision_map.find(i) == robot_collision_map.end()) {
        robot_collision_map[i] = std::vector<int>();
      }
      robot_collision_map[i].insert(robot_collision_map[i].end(), robot_collisions.begin(), robot_collisions.end());
    }
    std::vector<int> environment_collisions = find_human_environment_contact(human_capsules[i], environment_elements);
    if (environment_collisions.size() > 0) {
      if (environment_collision_map.find(i) == environment_collision_map.end()) {
        environment_collision_map[i] = std::vector<int>();
      }
      environment_collision_map[i].insert(environment_collision_map[i].end(), environment_collisions.begin(), environment_collisions.end());
    }
  }
}
*/
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
}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}