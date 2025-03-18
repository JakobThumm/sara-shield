#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
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
  EXPECT_TRUE(verify_iso_.verifyHumanReach(r_caps, h_caps_list));
}

TEST_F(VerifyIsoTest, FindHumanRobotContactTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap1(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap3(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.1), 0.5);
  std::vector<int> human_robot_collisions_1 = findHumanRobotContact(h_cap1, r_caps);
  EXPECT_EQ(human_robot_collisions_1.size(), 0);
  std::vector<int> human_robot_collisions_2 = findHumanRobotContact(h_cap2, r_caps);
  EXPECT_EQ(human_robot_collisions_2.size(), 2);
  EXPECT_EQ(human_robot_collisions_2[0], 0);
  EXPECT_EQ(human_robot_collisions_2[1], 1);
  std::vector<int> human_robot_collisions_3 = findHumanRobotContact(h_cap3, r_caps);
  EXPECT_EQ(human_robot_collisions_3.size(), 1);
  EXPECT_EQ(human_robot_collisions_3[0], 0);
}

TEST_F(VerifyIsoTest, FindHumanEnvironmentContactTest) {
  reach_lib::Capsule h_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule h_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  reach_lib::AABB env1({-1.0, -1.0, -0.5}, {1.0, 1.0, 0.0});
  reach_lib::AABB env2({-1.0, -1.0, -1.0}, {1.0, -0.5, 1.0});
  std::vector<reach_lib::AABB> envs = {env1, env2};
  std::vector<int> human_env_collisions_1 = findHumanEnvironmentContacts(h_cap1, envs);
  EXPECT_EQ(human_env_collisions_1.size(), 1);
  EXPECT_EQ(human_env_collisions_1[0], 0);
  std::vector<int> human_env_collisions_2 = findHumanEnvironmentContacts(h_cap2, envs);
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
  std::unordered_map<int, std::unordered_set<int>> robot_collision_map;
  std::unordered_map<int, std::unordered_set<int>> environment_collision_map;
  buildContactMaps(r_caps, h_caps, envs, robot_collision_map, environment_collision_map);
  EXPECT_EQ(robot_collision_map.size(), 2);
  // h_cap1 collides with r_cap1 and r_cap2
  EXPECT_EQ(robot_collision_map[0].size(), 2);
  EXPECT_TRUE(robot_collision_map[0].find(0) != robot_collision_map[0].end());
  EXPECT_TRUE(robot_collision_map[0].find(1) != robot_collision_map[0].end());
  // h_cap2 doesn't collide with any robot capsule
  EXPECT_TRUE(robot_collision_map.find(1) == robot_collision_map.end());
  // h_cap3 collides with r_cap1
  EXPECT_EQ(robot_collision_map[2].size(), 1);
  EXPECT_TRUE(robot_collision_map[2].find(0) != robot_collision_map[2].end());
  // h_cap1 collides with env1
  EXPECT_EQ(environment_collision_map[0].size(), 1);
  EXPECT_TRUE(environment_collision_map[0].find(0) != environment_collision_map[0].end());
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
      selfConstrainedCollisionCheck(robot_collisions_1, unclampable_enclosures_map, r_caps, d_human));
  std::vector<int> robot_collisions_2 = {0, 3};
  EXPECT_TRUE(
      selfConstrainedCollisionCheck(robot_collisions_2, unclampable_enclosures_map, r_caps, d_human));
  std::vector<int> robot_collisions_3 = {3, 4};
  EXPECT_TRUE(
      selfConstrainedCollisionCheck(robot_collisions_3, unclampable_enclosures_map, r_caps, d_human));
  d_human = 0.3;
  std::vector<int> robot_collisions_4 = {0, 4};
  EXPECT_TRUE(
      selfConstrainedCollisionCheck(robot_collisions_4, unclampable_enclosures_map, r_caps, d_human));
  d_human = 0.29;
  std::vector<int> robot_collisions_5 = {0, 4};
  EXPECT_FALSE(
      selfConstrainedCollisionCheck(robot_collisions_5, unclampable_enclosures_map, r_caps, d_human));
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
  Eigen::Vector3d expected_normal3_0(1.0, 0.0, 0.0);
  Eigen::Vector3d expected_normal3_1(0.0, 1.0, 0.0);
  reach_lib::Capsule robot_capsule4(reach_lib::Point(1.5, 0.5, 1.5), reach_lib::Point(2.0, 1.0, 2.0), 0.1);
  Eigen::Vector3d expected_normal4_0(1.0, 0.0, 0.0);
  Eigen::Vector3d expected_normal4_1(0.0, 1.0, 0.0);
  Eigen::Vector3d expected_normal4_2(0.0, 0.0, 1.0);
  std::vector<Eigen::Vector3d> normals;
  EXPECT_TRUE(calculateNormalVectors(robot_capsule0, environment_element, normals));
  double tol = 1e-6;
  EXPECT_TRUE(normals.size() == 1);
  EXPECT_NEAR(normals[0].x(), expected_normal0.x(), tol);
  EXPECT_NEAR(normals[0].y(), expected_normal0.y(), tol);
  EXPECT_NEAR(normals[0].z(), expected_normal0.z(), tol);
  normals.clear();
  EXPECT_TRUE(calculateNormalVectors(robot_capsule1, environment_element, normals));
  EXPECT_TRUE(normals.size() == 1);
  EXPECT_NEAR(normals[0].x(), expected_normal1.x(), tol);
  EXPECT_NEAR(normals[0].y(), expected_normal1.y(), tol);
  EXPECT_NEAR(normals[0].z(), expected_normal1.z(), tol);
  normals.clear();
  EXPECT_FALSE(calculateNormalVectors(robot_capsule2, environment_element, normals));
  normals.clear();
  EXPECT_TRUE(calculateNormalVectors(robot_capsule3, environment_element, normals));
  EXPECT_TRUE(normals.size() == 2);
  EXPECT_NEAR(normals[0].x(), expected_normal3_0.x(), tol);
  EXPECT_NEAR(normals[0].y(), expected_normal3_0.y(), tol);
  EXPECT_NEAR(normals[0].z(), expected_normal3_0.z(), tol);
  EXPECT_NEAR(normals[1].x(), expected_normal3_1.x(), tol);
  EXPECT_NEAR(normals[1].y(), expected_normal3_1.y(), tol);
  EXPECT_NEAR(normals[1].z(), expected_normal3_1.z(), tol);
  normals.clear();
  EXPECT_TRUE(calculateNormalVectors(robot_capsule4, environment_element, normals));
  EXPECT_TRUE(normals.size() == 3);
  EXPECT_NEAR(normals[0].x(), expected_normal4_0.x(), tol);
  EXPECT_NEAR(normals[0].y(), expected_normal4_0.y(), tol);
  EXPECT_NEAR(normals[0].z(), expected_normal4_0.z(), tol);
  EXPECT_NEAR(normals[1].x(), expected_normal4_1.x(), tol);
  EXPECT_NEAR(normals[1].y(), expected_normal4_1.y(), tol);
  EXPECT_NEAR(normals[1].z(), expected_normal4_1.z(), tol);
  EXPECT_NEAR(normals[2].x(), expected_normal4_2.x(), tol);
  EXPECT_NEAR(normals[2].y(), expected_normal4_2.y(), tol);
  EXPECT_NEAR(normals[2].z(), expected_normal4_2.z(), tol);
}

TEST_F(VerifyIsoTest, CapsuleMovingTowardsElementTest) {
  Eigen::Vector3d normal(0.0, 1.0, 0.0);
  RobotReach::CapsuleVelocity capsule_velocity0(
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)),
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)));
  RobotReach::CapsuleVelocity capsule_velocity1(
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)),
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)));
  RobotReach::CapsuleVelocity capsule_velocity2(
      RobotReach::SE3Vel(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)),
      RobotReach::SE3Vel(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)));
  RobotReach::CapsuleVelocity capsule_velocity3(
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)),
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)));
  RobotReach::CapsuleVelocity capsule_velocity4(
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, -0.1, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)),
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, -0.1, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)));
  RobotReach::CapsuleVelocity capsule_velocity5(
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.1, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)),
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.1, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)));
  RobotReach::CapsuleVelocity capsule_velocity6(
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.111, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)),
      RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.111, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)));
  double radius = 0.1;
  double velocity_error = 0.01;
  EXPECT_TRUE(capsuleMovingTowardsElement(capsule_velocity0, normal, radius, velocity_error));
  EXPECT_TRUE(capsuleMovingTowardsElement(capsule_velocity1, normal, radius, velocity_error));
  EXPECT_TRUE(capsuleMovingTowardsElement(capsule_velocity2, normal, radius, velocity_error));
  EXPECT_FALSE(capsuleMovingTowardsElement(capsule_velocity3, normal, radius, velocity_error));
  EXPECT_TRUE(capsuleMovingTowardsElement(capsule_velocity4, normal, radius, velocity_error));
  EXPECT_TRUE(capsuleMovingTowardsElement(capsule_velocity5, normal, radius, velocity_error));
  EXPECT_FALSE(capsuleMovingTowardsElement(capsule_velocity6, normal, radius, velocity_error));
}

TEST_F(VerifyIsoTest, EnvironmentallyConstrainedCollisionCheckTest) {
  // Environment element
  std::vector<reach_lib::AABB> environment_elements{reach_lib::AABB({-1.0, -0.05, -1.0}, {1.0, 0.0, 1.0})};
  std::vector<int> environment_collisions = {0};
  // Human capsule
  double d_human = 0.1;
  // Robot capsules
  // 1 capsule moving upwards (+y) for the first 3 time steps and downwards for the next 3 time steps
  // The capsule is located at (0.5, 0.05, -0.5), (0.5, 0.05, 0.5) in the first step and moves only in y-direction.
  std::vector<int> robot_collisions = {0};
  std::vector<reach_lib::Capsule> robot_capsules = {reach_lib::Capsule({0.5, 0.05, -0.5}, {0.5, 0.05, 0.5}, 0.1)};
  std::vector<std::vector<RobotReach::CapsuleVelocity>> robot_capsule_velocities = {
       {RobotReach::CapsuleVelocity(RobotReach::SE3Vel(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)),
                                   RobotReach::SE3Vel(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)))},
       {RobotReach::CapsuleVelocity(RobotReach::SE3Vel(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)),
                                   RobotReach::SE3Vel(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)))},
       {RobotReach::CapsuleVelocity(
           RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.111, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)),
           RobotReach::SE3Vel(Eigen::Vector3d(0.0, 0.111, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)))},
       {RobotReach::CapsuleVelocity(RobotReach::SE3Vel(Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)),
                                   RobotReach::SE3Vel(Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)))},
       {RobotReach::CapsuleVelocity(RobotReach::SE3Vel(Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)),
                                   RobotReach::SE3Vel(Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)))},
       {RobotReach::CapsuleVelocity(RobotReach::SE3Vel(Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)),
                                   RobotReach::SE3Vel(Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)))}};
  // Velocity error = 1/2 \Delta s (\alpha_i + \beta_i r_i)
  // 0.01 = 1/2 * 0.01 * (1.0 + 10.0 * 0.1)
  std::vector<double> velocity_errors = {0.01};
  // Check if ECC could occur for upwards movement (no ECC should occur)
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_start =
      robot_capsule_velocities.begin();
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end =
      robot_capsule_velocities.end() - 3;
  EXPECT_FALSE(environmentallyConstrainedCollisionCheck(
      robot_collisions, robot_capsules, environment_collisions, environment_elements, d_human,
      robot_capsule_velocities_start, robot_capsule_velocities_end, velocity_errors));
  // Check if ECC could occur for downwards movement (ECC should occur)
  robot_capsule_velocities_start = robot_capsule_velocities.begin() + 3;
  robot_capsule_velocities_end = robot_capsule_velocities.end();
  EXPECT_TRUE(environmentallyConstrainedCollisionCheck(
      robot_collisions, robot_capsules, environment_collisions, environment_elements, d_human,
      robot_capsule_velocities_start, robot_capsule_velocities_end, velocity_errors));
}

TEST_F(VerifyIsoTest, BuildHumanContactGraphTest) {
  std::vector<reach_lib::Capsule> human_capsules = {
    // 0: intersects with -2-, 6
    reach_lib::Capsule({1, 1, 0}, {1, 3, 0}, 1),
    // 1: intersects with 6
    reach_lib::Capsule({4, 1, 0}, {4, 3, 0}, 1),
    // 2: intersects with -0-, 3
    reach_lib::Capsule({1, 0, 0}, {1, -3, 0}, 1),
    // 3: intersects with 2
    reach_lib::Capsule({2, -3, 0}, {2, -3, 0}, 1),
    // 4: intersects with nothing
    reach_lib::Capsule({5, -2, 0}, {5, -2, 0}, 1),
    // 5: intersects with 7
    reach_lib::Capsule({7, 4, 0}, {7, 4, 0}, 1),
    // 6: intersects with 0, 1
    reach_lib::Capsule({1, 4, 0}, {4, 4, 0}, 1),
    // 7: intersects with 5
    reach_lib::Capsule({8, 4, 0}, {8, 4, 0}, 1),
  };
  std::unordered_map<int, std::set<int>> unclampable_body_part_map = {
    {0, {1, 2}},
    {1, {2}},
    {3, {4, 5}},
    {4, {5}},
    {6, {7}}
  };
  int current_body_part = 0;
  std::unordered_set<int> human_contact_graph;
  human_contact_graph.insert(current_body_part);
  std::unordered_set<int> visited_body_parts = {current_body_part};
  buildHumanContactGraph(current_body_part, human_capsules, unclampable_body_part_map, visited_body_parts, human_contact_graph);
  // check if the human contact graph size is 3 and print the graph if not.
  std::string debug_out = "Human contact graph contents: ";
  for (const auto& body : human_contact_graph) {
    debug_out += std::to_string(body) + ", ";
  }
  // [0]: 0, 6 
  // [1]: 1, 6 
  // [2]: 2, 3
  // [3]: 4
  // [4]: 5, 7
  EXPECT_EQ(human_contact_graph.size(), 2) << debug_out;
  EXPECT_TRUE(human_contact_graph.find(0) != human_contact_graph.end()) << debug_out;
  EXPECT_TRUE(human_contact_graph.find(6) != human_contact_graph.end()) << debug_out;
  EXPECT_EQ(visited_body_parts.size(), 2);
  EXPECT_FALSE(visited_body_parts.find(0) == visited_body_parts.end());
  EXPECT_FALSE(visited_body_parts.find(6) == visited_body_parts.end());
  current_body_part = 1;
  human_contact_graph = {current_body_part};
  visited_body_parts.insert(current_body_part);
  buildHumanContactGraph(current_body_part, human_capsules, unclampable_body_part_map, visited_body_parts, human_contact_graph);
  EXPECT_EQ(human_contact_graph.size(), 2);
  EXPECT_TRUE(human_contact_graph.find(1) != human_contact_graph.end());
  EXPECT_TRUE(human_contact_graph.find(6) != human_contact_graph.end());
  EXPECT_EQ(visited_body_parts.size(), 3);
  EXPECT_FALSE(visited_body_parts.find(1) == visited_body_parts.end());
  current_body_part = 2;
  human_contact_graph = {current_body_part};
  visited_body_parts.insert(current_body_part);
  buildHumanContactGraph(current_body_part, human_capsules, unclampable_body_part_map, visited_body_parts, human_contact_graph);
  EXPECT_EQ(human_contact_graph.size(), 2);
  EXPECT_TRUE(human_contact_graph.find(2) != human_contact_graph.end());
  EXPECT_TRUE(human_contact_graph.find(3) != human_contact_graph.end());
  EXPECT_EQ(visited_body_parts.size(), 5);
  EXPECT_FALSE(visited_body_parts.find(2) == visited_body_parts.end());
  EXPECT_FALSE(visited_body_parts.find(3) == visited_body_parts.end());
}

TEST_F(VerifyIsoTest, BuildHumanContactGraphsTest) {
  std::vector<reach_lib::Capsule> human_capsules = {
    // 0: intersects with -2-, 6
    reach_lib::Capsule({1, 1, 0}, {1, 3, 0}, 1),
    // 1: intersects with 6
    reach_lib::Capsule({4, 1, 0}, {4, 3, 0}, 1),
    // 2: intersects with -0-, 3
    reach_lib::Capsule({1, 0, 0}, {1, -3, 0}, 1),
    // 3: intersects with 2
    reach_lib::Capsule({2, -3, 0}, {2, -3, 0}, 1),
    // 4: intersects with nothing
    reach_lib::Capsule({5, -2, 0}, {5, -2, 0}, 1),
    // 5: intersects with 7
    reach_lib::Capsule({7, 4, 0}, {7, 4, 0}, 1),
    // 6: intersects with 0, 1
    reach_lib::Capsule({1, 4, 0}, {4, 4, 0}, 1),
    // 7: intersects with 5
    reach_lib::Capsule({8, 4, 0}, {8, 4, 0}, 1),
  };
  std::unordered_map<int, std::set<int>> unclampable_body_part_map = {
    {0, {1, 2}},
    {1, {2}},
    {3, {4, 5}},
    {4, {5}},
    {6, {7}}
  };
  std::vector<std::unordered_set<int>> human_contact_graphs = buildHumanContactGraphs(human_capsules, unclampable_body_part_map);
  EXPECT_EQ(human_contact_graphs.size(), 5);
  // [0]: 0, 6 
  EXPECT_EQ(human_contact_graphs[0].size(), 2);
  EXPECT_TRUE(human_contact_graphs[0].find(0) != human_contact_graphs[0].end());
  EXPECT_TRUE(human_contact_graphs[0].find(6) != human_contact_graphs[0].end());
  // [1]: 1, 6 
  EXPECT_EQ(human_contact_graphs[1].size(), 2);
  EXPECT_TRUE(human_contact_graphs[1].find(1) != human_contact_graphs[0].end());
  EXPECT_TRUE(human_contact_graphs[1].find(6) != human_contact_graphs[0].end());
  // [2]: 2, 3
  EXPECT_EQ(human_contact_graphs[2].size(), 2);
  EXPECT_TRUE(human_contact_graphs[2].find(2) != human_contact_graphs[0].end());
  EXPECT_TRUE(human_contact_graphs[2].find(3) != human_contact_graphs[0].end());
  // [3]: 4
  EXPECT_EQ(human_contact_graphs[3].size(), 1);
  EXPECT_TRUE(human_contact_graphs[3].find(4) != human_contact_graphs[0].end());
  // [4]: 5, 7
  EXPECT_EQ(human_contact_graphs[4].size(), 2);
  EXPECT_TRUE(human_contact_graphs[4].find(5) != human_contact_graphs[0].end());
  EXPECT_TRUE(human_contact_graphs[4].find(7) != human_contact_graphs[0].end());
}

TEST_F(VerifyIsoTest, CombineContactMapsTest) {
  std::vector<reach_lib::Capsule> human_capsules = {
    // 0: intersects with -2-, 6
    reach_lib::Capsule({1, 1, 0}, {1, 3, 0}, 1),
    // 1: intersects with 6
    reach_lib::Capsule({4, 1, 0}, {4, 3, 0}, 1),
    // 2: intersects with -0-, 3
    reach_lib::Capsule({1, 0, 0}, {1, -3, 0}, 1),
    // 3: intersects with 2
    reach_lib::Capsule({2, -3, 0}, {2, -3, 0}, 1),
    // 4: intersects with nothing
    reach_lib::Capsule({5, -2, 0}, {5, -2, 0}, 1),
    // 5: intersects with 7
    reach_lib::Capsule({7, 4, 0}, {7, 4, 0}, 1),
    // 6: intersects with 0, 1
    reach_lib::Capsule({1, 4, 0}, {4, 4, 0}, 1),
    // 7: intersects with 5
    reach_lib::Capsule({8, 4, 0}, {8, 4, 0}, 1),
  };
  std::unordered_map<int, std::set<int>> unclampable_body_part_map = {
    {0, {1, 2}},
    {1, {2}},
    {3, {4, 5}},
    {4, {5}},
    {6, {7}}
  };
  std::vector<double> human_radii = {0.1, 10, 200, 3000, 40000, 500000, 6000000, 70000000};
  std::unordered_map<int, std::unordered_set<int>> robot_collision_map;
  robot_collision_map[0] = {0};
  std::unordered_map<int, std::unordered_set<int>> environment_collision_map;
  environment_collision_map[1] = {0};
  std::vector<double> combined_human_radii;
  combineContactMaps(
    human_capsules,
    human_radii,
    unclampable_body_part_map,
    robot_collision_map,
    environment_collision_map,
    combined_human_radii
  );
  // [0]: 0, 6 
  // [1]: 1, 6 
  // [2]: 2, 3
  // [3]: 4
  // [4]: 5, 7
  EXPECT_EQ(combined_human_radii.size(), 5);
  EXPECT_EQ(combined_human_radii[0], 6000000.1);
  EXPECT_EQ(combined_human_radii[1], 6000010);
  EXPECT_EQ(combined_human_radii[2], 3200);
  EXPECT_EQ(combined_human_radii[3], 40000);
  EXPECT_EQ(combined_human_radii[4], 70500000);
  EXPECT_EQ(robot_collision_map[0].size(), 1);
  EXPECT_TRUE(robot_collision_map[0].find(0) != robot_collision_map[0].end());
  EXPECT_EQ(robot_collision_map[1].size(), 0);
  EXPECT_EQ(robot_collision_map[2].size(), 0);
  EXPECT_EQ(robot_collision_map[3].size(), 0);
  EXPECT_EQ(environment_collision_map[0].size(), 0);
  EXPECT_EQ(environment_collision_map[1].size(), 1);
  EXPECT_TRUE(environment_collision_map[1].find(0) != environment_collision_map[1].end());
  EXPECT_EQ(environment_collision_map[2].size(), 0);
  EXPECT_EQ(environment_collision_map[3].size(), 0);
}

TEST_F(VerifyIsoTest, VerifyHumanReachTimeIntervalsTest) {
  // Timestep 0: all safe
  reach_lib::Capsule r_cap1_0(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2_0(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps_0 = {r_cap1_0, r_cap2_0};
  reach_lib::Capsule h_cap1_0(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2_0(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  std::vector<std::vector<reach_lib::Capsule>> h_caps_0 = {{h_cap1_0, h_cap2_0}};
  // Timestep 1: all safe
  reach_lib::Capsule r_cap1_1(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  reach_lib::Capsule r_cap2_1(reach_lib::Point(0.0, 0.0, 2.0), reach_lib::Point(0.0, 0.0, 3.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps_1 = {r_cap1_1, r_cap2_1};
  reach_lib::Capsule h_cap1_1(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  reach_lib::Capsule h_cap2_1(reach_lib::Point(1.0, 0.0, 2.0), reach_lib::Point(1.0, 0.0, 3.0), 0.5);
  std::vector<std::vector<reach_lib::Capsule>> h_caps_1 = {{h_cap1_1, h_cap2_1}};
  // Timestep 2: collision
  reach_lib::Capsule r_cap1_2(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.1);
  reach_lib::Capsule r_cap2_2(reach_lib::Point(1.0, 0.0, 2.0), reach_lib::Point(1.0, 0.0, 3.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps_2 = {r_cap1_2, r_cap2_2};
  reach_lib::Capsule h_cap1_2(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  reach_lib::Capsule h_cap2_2(reach_lib::Point(1.0, 0.0, 2.0), reach_lib::Point(1.0, 0.0, 3.0), 0.5);
  std::vector<std::vector<reach_lib::Capsule>> h_caps_2 = {{h_cap1_2, h_cap2_2}};

  std::vector<std::vector<std::vector<reach_lib::Capsule>>> h_caps_list_01 = {h_caps_0, h_caps_1};
  std::vector<std::vector<reach_lib::Capsule>> r_caps_list_01 = {r_caps_0, r_caps_1};
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> h_caps_list_012 = {h_caps_0, h_caps_1, h_caps_2};
  std::vector<std::vector<reach_lib::Capsule>> r_caps_list_012 = {r_caps_0, r_caps_1, r_caps_2};
  int collision_index;
  try {
    verify_iso_.verifyHumanReachTimeIntervals(r_caps_list_01, h_caps_list_012, collision_index);
    // We should never reach this point
    ASSERT_TRUE(false);
  } catch (std::length_error& e) {
    EXPECT_TRUE(true);
  }
  EXPECT_TRUE(verify_iso_.verifyHumanReachTimeIntervals(r_caps_list_01, h_caps_list_01, collision_index));
  EXPECT_EQ(collision_index, -1);
  EXPECT_FALSE(verify_iso_.verifyHumanReachTimeIntervals(r_caps_list_012, h_caps_list_012, collision_index));
  EXPECT_EQ(collision_index, 2);
}

TEST_F(VerifyIsoTest, VerifyHumanReachVelocityTest) {
  // Timestep 0: all safe
  reach_lib::Capsule r_cap1_0(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2_0(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps_0 = {r_cap1_0, r_cap2_0};
  reach_lib::Capsule h_cap1_0(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2_0(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  std::vector<std::vector<reach_lib::Capsule>> h_caps_0 = {{h_cap1_0, h_cap2_0}};
  // Timestep 1: all safe
  reach_lib::Capsule r_cap1_1(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  reach_lib::Capsule r_cap2_1(reach_lib::Point(0.0, 0.0, 2.0), reach_lib::Point(0.0, 0.0, 3.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps_1 = {r_cap1_1, r_cap2_1};
  reach_lib::Capsule h_cap1_1(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  reach_lib::Capsule h_cap2_1(reach_lib::Point(1.0, 0.0, 2.0), reach_lib::Point(1.0, 0.0, 3.0), 0.5);
  std::vector<std::vector<reach_lib::Capsule>> h_caps_1 = {{h_cap1_1, h_cap2_1}};
  // Timestep 2: collision
  reach_lib::Capsule r_cap1_2(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.1);
  reach_lib::Capsule r_cap2_2(reach_lib::Point(1.0, 0.0, 2.0), reach_lib::Point(1.0, 0.0, 3.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps_2 = {r_cap1_2, r_cap2_2};
  reach_lib::Capsule h_cap1_2(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  reach_lib::Capsule h_cap2_2(reach_lib::Point(1.0, 0.0, 2.0), reach_lib::Point(1.0, 0.0, 3.0), 0.5);
  std::vector<std::vector<reach_lib::Capsule>> h_caps_2 = {{h_cap1_2, h_cap2_2}};

  std::vector<std::vector<std::vector<reach_lib::Capsule>>> h_caps_list_01 = {h_caps_0, h_caps_1};
  std::vector<std::vector<reach_lib::Capsule>> r_caps_list_01 = {r_caps_0, r_caps_1};
  std::vector<std::vector<double>> robot_link_velocities_01 = {{0.4, 0.4}, {0.3, 0.3}};
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> h_caps_list_012 = {h_caps_0, h_caps_1, h_caps_2};
  std::vector<std::vector<reach_lib::Capsule>> r_caps_list_012 = {r_caps_0, r_caps_1, r_caps_2};
  std::vector<std::vector<double>> robot_link_velocities_012 = {{0.4, 0.4}, {0.3, 0.3}, {0.2, 0.2}};
  std::vector<std::vector<double>> maximal_contact_velocities_high = {{0.3, 0.4}};
  std::vector<std::vector<double>> maximal_contact_velocities_low = {{0.1, 0.1}};
  std::vector<std::vector<double>> maximal_contact_velocities_wrongly_defined = {{0.3, 0.4, 0.5}};
  std::vector<std::vector<double>> maximal_contact_velocities_wrongly_defined2 = {{0.3, 0.4, 0.5}, {0.3, 0.4, 0.5}};
  int collision_index;
  try {
    verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_012, robot_link_velocities_01, maximal_contact_velocities_high, collision_index);
    // We should never reach this point
    ASSERT_TRUE(false);
  } catch (std::length_error& e) {
    EXPECT_TRUE(true);
  }
  try {
    verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_01, robot_link_velocities_01, maximal_contact_velocities_wrongly_defined, collision_index);
    // We should never reach this point
    ASSERT_TRUE(false);
  } catch (std::length_error& e) {
    EXPECT_TRUE(true);
  }
  try {
    verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_01, robot_link_velocities_01, maximal_contact_velocities_wrongly_defined2, collision_index);
    // We should never reach this point
    ASSERT_TRUE(false);
  } catch (std::length_error& e) {
    EXPECT_TRUE(true);
  }
  // No collision
  EXPECT_TRUE(verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_01, robot_link_velocities_01, maximal_contact_velocities_high, collision_index));
  EXPECT_EQ(collision_index, -1);
  // No collision
  EXPECT_TRUE(verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_01, robot_link_velocities_01, maximal_contact_velocities_low, collision_index));
  EXPECT_EQ(collision_index, -1);
  // Collision but slow enough
  EXPECT_TRUE(verify_iso_.verifyHumanReachVelocity(r_caps_list_012, h_caps_list_012, robot_link_velocities_012, maximal_contact_velocities_high, collision_index));
  EXPECT_EQ(collision_index, -1);
  // Collision and too fast -> not safe
  EXPECT_FALSE(verify_iso_.verifyHumanReachVelocity(r_caps_list_012, h_caps_list_012, robot_link_velocities_012, maximal_contact_velocities_low, collision_index));
  EXPECT_EQ(collision_index, 2);
}

TEST(VerifyUtilsTest, calculateRobotLinkEnergiesMassVelTest) {
  std::vector<double> robot_link_velocities = {1.0, 2.0};
  std::vector<double> robot_link_reflected_masses = {2.0, 3.0};
  std::vector<double> energies = calculateRobotLinkEnergies(robot_link_velocities, robot_link_reflected_masses);
  EXPECT_EQ(energies.size(), 2);
  EXPECT_DOUBLE_EQ(energies[0], 1.0);
  EXPECT_DOUBLE_EQ(energies[1], 6.0);
}

TEST(VerifyUtilsTest, checkContactEnergySafetyTest) {
  std::map<int, std::vector<int>> human_robot_contacts;
  std::vector<double> robot_link_energies = {0.1, 0.5, 1.0};
  std::vector<double> maximal_contact_energies = {0.75};
  EXPECT_TRUE(checkContactEnergySafety(human_robot_contacts, robot_link_energies, maximal_contact_energies));
  human_robot_contacts[0] = {};
  EXPECT_TRUE(checkContactEnergySafety(human_robot_contacts, robot_link_energies, maximal_contact_energies));
  human_robot_contacts[0] = {0};
  EXPECT_TRUE(checkContactEnergySafety(human_robot_contacts, robot_link_energies, maximal_contact_energies));
  human_robot_contacts[0] = {0, 1};
  EXPECT_TRUE(checkContactEnergySafety(human_robot_contacts, robot_link_energies, maximal_contact_energies));
  human_robot_contacts[0] = {0, 1, 2};
  EXPECT_FALSE(checkContactEnergySafety(human_robot_contacts, robot_link_energies, maximal_contact_energies));
  human_robot_contacts[0] = {2};
  EXPECT_FALSE(checkContactEnergySafety(human_robot_contacts, robot_link_energies, maximal_contact_energies));
  human_robot_contacts[0] = {0, 2};
  EXPECT_FALSE(checkContactEnergySafety(human_robot_contacts, robot_link_energies, maximal_contact_energies));
}

TEST(VerifyUtilsTest, calculateMaxRobotLinkVelocitiesPerTimeIntervalTest) {
  Motion m1 = Motion(2);
  m1.setMaximumCartesianVelocities({1, 2});
  Motion m2 = Motion(2);
  m2.setMaximumCartesianVelocities({2, 2});
  Motion m3 = Motion(2);
  m3.setMaximumCartesianVelocities({3, 2});
  std::vector<Motion> robot_motions = {m1, m2, m3};
  std::vector<double> velocity_error = {0.01, 0.03};
  std::vector<double> velocity_error_wrong = {0.01, 0.03, 0.04};
  EXPECT_THROW(calculateMaxRobotLinkVelocitiesPerTimeInterval(robot_motions, velocity_error_wrong), std::length_error);
  EXPECT_EQ(calculateMaxRobotLinkVelocitiesPerTimeInterval({}, velocity_error_wrong).size(), 0);
  std::vector<std::vector<double>> max_vels = calculateMaxRobotLinkVelocitiesPerTimeInterval(robot_motions, velocity_error);
  EXPECT_EQ(max_vels.size(), 2);
  EXPECT_EQ(max_vels[0].size(), 2);
  EXPECT_DOUBLE_EQ(max_vels[0][0], 2.01);
  EXPECT_DOUBLE_EQ(max_vels[1][0], 3.01);
  EXPECT_DOUBLE_EQ(max_vels[0][1], 2.03);
  EXPECT_DOUBLE_EQ(max_vels[1][1], 2.03);
}

TEST(VerifyUtilsTest, calculateMaxRobotEnergiesFromReflectedMassesTest) {
    // Test case 1: Normal case
    std::vector<std::vector<double>> velocities = {{1.0, 2.0}, {2.0, 3.0}, {3.0, 4.0}};
    std::vector<std::vector<double>> masses = {{2.0, 3.0}, {2.0, 3.0}, {2.0, 3.0}};
    
    auto result = calculateMaxRobotEnergiesFromReflectedMasses(velocities, masses);
    
    ASSERT_EQ(result.size(), 3);
    ASSERT_EQ(result[0].size(), 2);
    EXPECT_DOUBLE_EQ(result[0][0], 1.0);
    EXPECT_DOUBLE_EQ(result[0][1], 6.0);
    EXPECT_DOUBLE_EQ(result[1][0], 4.0);
    EXPECT_DOUBLE_EQ(result[1][1], 13.5);
    EXPECT_DOUBLE_EQ(result[2][0], 9.0);
    EXPECT_DOUBLE_EQ(result[2][1], 24.0);

    // Test case 2: Empty input
    EXPECT_NO_THROW(calculateMaxRobotEnergiesFromReflectedMasses({}, {}));

    // Test case 3: Mismatched sizes of velocities and masses
    std::vector<std::vector<double>> wrong_masses = {{2.0, 3.0}, {2.0, 3.0}};
    EXPECT_THROW(calculateMaxRobotEnergiesFromReflectedMasses(velocities, wrong_masses), std::length_error);

    // Test case 4: Mismatched number of links
    std::vector<std::vector<double>> wrong_link_count = {{1.0, 2.0, 3.0}, {2.0, 3.0, 4.0}};
    EXPECT_THROW(calculateMaxRobotEnergiesFromReflectedMasses(wrong_link_count, masses), std::length_error);

    // Test case 5: Single time interval
    std::vector<std::vector<double>> single_interval_velocities = {{1.0, 2.0}};
    std::vector<std::vector<double>> single_interval_masses = {{2.0, 3.0}};
    auto single_result = calculateMaxRobotEnergiesFromReflectedMasses(single_interval_velocities, single_interval_masses);
    ASSERT_EQ(single_result.size(), 1);
    ASSERT_EQ(single_result[0].size(), 2);
    EXPECT_DOUBLE_EQ(single_result[0][0], 1.0);
    EXPECT_DOUBLE_EQ(single_result[0][1], 6.0);
}

TEST(VerifyUtilsTest, calculateMaxRobotEnergiesFromInertiaMatricesTest) {
    // Test case: Normal case
    std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> inertia_matrices = {
        {
            (Eigen::Matrix<double, 2, 2>() << 1.0, 0.0, 0.0, 0.0).finished(),
            (Eigen::Matrix<double, 2, 2>() << 1.0, 0.0, 0.0, 1.0).finished()
        },
        {
            (Eigen::Matrix<double, 2, 2>() << 2.0, 0.0, 0.0, 0.0).finished(),
            (Eigen::Matrix<double, 2, 2>() << 2.0, 0.0, 0.0, 1.0).finished()
        }
    };
    std::vector<std::vector<double>> joint_velocities = {{1.0, 2.0}, {2.0, 3.0}};

    auto result = calculateMaxRobotEnergiesFromInertiaMatrices(inertia_matrices, joint_velocities);

    ASSERT_EQ(result.size(), 2);
    ASSERT_EQ(result[0].size(), 2);
    EXPECT_NEAR(result[0][0], 0.5, 1e-6);
    EXPECT_NEAR(result[0][1], 2.5, 1e-6);
    EXPECT_NEAR(result[1][0], 4.0, 1e-6);
    EXPECT_NEAR(result[1][1], 8.5, 1e-6);

    // Test case: Different matrix sizes
    std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> different_size_matrices = {
        {
            (Eigen::Matrix<double, 3, 3>() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
            (Eigen::Matrix<double, 3, 3>() << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0).finished(),
            (Eigen::Matrix<double, 3, 3>() << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0).finished()
        }
    };
    std::vector<std::vector<double>> different_size_velocities = {{1.0, 4.0, 5.0}};
    auto different_size_result = calculateMaxRobotEnergiesFromInertiaMatrices(different_size_matrices, different_size_velocities);
    ASSERT_EQ(different_size_result.size(), 1);
    ASSERT_EQ(different_size_result[0].size(), 3);
    EXPECT_NEAR(different_size_result[0][0], 0.5 * (1), 1e-6);
    EXPECT_NEAR(different_size_result[0][1], 0.5 * (1 + 2*4*4), 1e-6);
    EXPECT_NEAR(different_size_result[0][2], 0.5 * (1 + 2*4*4 + 3*5*5), 1e-6);

    // Test case: Empty input
    EXPECT_NO_THROW(calculateMaxRobotEnergiesFromInertiaMatrices({}, {}));

    // Test case: Mismatched sizes of inertia matrices and joint velocities
    std::vector<std::vector<double>> wrong_velocities = {{1.0, 2.0}};
    EXPECT_THROW(calculateMaxRobotEnergiesFromInertiaMatrices(inertia_matrices, wrong_velocities), std::length_error);

    // New test case: Mismatched number of links
    std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> mismatched_links_matrices = {
        {
            (Eigen::Matrix<double, 2, 2>() << 1.0, 0.0, 0.0, 2.0).finished(),
            (Eigen::Matrix<double, 2, 2>() << 2.0, 0.0, 0.0, 3.0).finished(),
            (Eigen::Matrix<double, 2, 2>() << 3.0, 0.0, 0.0, 4.0).finished() // Extra link
        }
    };
    std::vector<std::vector<double>> mismatched_links_velocities = {{1.0, 2.0}}; // Only 2 links
    EXPECT_THROW({
        calculateMaxRobotEnergiesFromInertiaMatrices(mismatched_links_matrices, mismatched_links_velocities);
    }, std::length_error);

    // New test case: Mismatched number of joints (matrix size doesn't match velocity vector size)
    std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> mismatched_joints_matrices = {
        {
            (Eigen::Matrix<double, 3, 3>() << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0).finished(),
            (Eigen::Matrix<double, 3, 3>() << 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0).finished()
        }
    };
    std::vector<std::vector<double>> mismatched_joints_velocities = {{1.0, 2.0}}; // Only 2 joints, but matrices are 3x3
    EXPECT_THROW({
        calculateMaxRobotEnergiesFromInertiaMatrices(mismatched_joints_matrices, mismatched_joints_velocities);
    }, std::length_error);

    // New test case: Non-square inertia matrix
    std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> non_square_matrices = {
        {
            (Eigen::Matrix<double, 2, 3>() << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0).finished(),
            (Eigen::Matrix<double, 2, 3>() << 2.0, 0.0, 0.0, 0.0, 3.0, 0.0).finished()
        }
    };
    std::vector<std::vector<double>> non_square_velocities = {{1.0, 2.0}};
    EXPECT_THROW({
        calculateMaxRobotEnergiesFromInertiaMatrices(non_square_matrices, non_square_velocities);
    }, std::length_error);
}

TEST_F(VerifyIsoTest, VerifyHumanReachEnergyTest) {
    // Test case 1: Normal case - safe scenario
    // Test case: collision can occur between all capsules
    reach_lib::Capsule r_cap1_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.1);
    reach_lib::Capsule r_cap2_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.1);
    std::vector<reach_lib::Capsule> r_caps_2 = {r_cap1_2, r_cap2_2};
    reach_lib::Capsule h_cap1_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.5);
    reach_lib::Capsule h_cap2_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.5);
    std::vector<std::vector<reach_lib::Capsule>> h_caps_2 = {{h_cap1_2, h_cap2_2}};
    std::vector<std::vector<double>> robot_link_energies = {{1.0, 2.0}};
    std::vector<std::vector<double>> maximal_contact_energies = {{3.0, 4.0}};

    std::vector<std::vector<reach_lib::Capsule>> robot_reachable_sets = {r_caps_2};
    std::vector<std::vector<std::vector<reach_lib::Capsule>>> human_reachable_sets = {h_caps_2};

    int collision_index = 0;
    EXPECT_TRUE(verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index));
    EXPECT_EQ(collision_index, -1);

    // Test case 2: Unsafe scenario
    robot_link_energies = {{3.5, 4.5}};
    EXPECT_FALSE(verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index));
    EXPECT_EQ(collision_index, 0);

    // Test case 2: Unsafe scenario
    robot_link_energies = {{2.5, 4.5}};
    EXPECT_FALSE(verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index));
    EXPECT_EQ(collision_index, 0);

    // Test case 2: Unsafe scenario
    robot_link_energies = {{3.5, 1.5}};
    EXPECT_FALSE(verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index));
    EXPECT_EQ(collision_index, 0);

    // Test case 2: Unsafe scenario
    robot_link_energies = {{2.5, 3.5}};
    EXPECT_FALSE(verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index));
    EXPECT_EQ(collision_index, 0);

    // Test case 3: Mismatched time intervals
    std::vector<std::vector<reach_lib::Capsule>> mismatched_robot_sets = {
        {reach_lib::Capsule(), reach_lib::Capsule()}, {reach_lib::Capsule(), reach_lib::Capsule()}
    };
    EXPECT_THROW({
        verify_iso_.verifyHumanReachEnergy(mismatched_robot_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index);
    }, std::length_error);

    // Test case 4: Mismatched robot links
    std::vector<std::vector<double>> mismatched_robot_energies = {{1.0}, {1.5}};
    EXPECT_THROW({
        verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, mismatched_robot_energies, maximal_contact_energies, collision_index);
    }, std::length_error);

    // Test case 5: Mismatched human models
    std::vector<std::vector<double>> mismatched_max_energies = {{3.0, 4.0}, {3.0, 4.0}};
    EXPECT_THROW({
        verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, mismatched_max_energies, collision_index);
    }, std::length_error);

    // Test case 6: Mismatched human bodies
    std::vector<std::vector<double>> mismatched_body_energies = {{3.0}};
    EXPECT_THROW({
        verify_iso_.verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, mismatched_body_energies, collision_index);
    }, std::length_error);

    // Test case 7: Empty inputs
    std::vector<std::vector<reach_lib::Capsule>> empty_robot_sets;
    std::vector<std::vector<std::vector<reach_lib::Capsule>>> empty_human_sets;
    std::vector<std::vector<double>> empty_robot_energies;
    std::vector<std::vector<double>> empty_max_energies;
    EXPECT_FALSE(verify_iso_.verifyHumanReachEnergy(empty_robot_sets, empty_human_sets, empty_robot_energies, empty_max_energies, collision_index));
    EXPECT_EQ(collision_index, -1);
}

TEST_F(VerifyIsoTest, VerifyHumanReachEnergyReflectedMassesTest) {
    // Test case 1: Normal case - safe scenario
    // Test case: collision can occur between all capsules
    reach_lib::Capsule r_cap1_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.1);
    reach_lib::Capsule r_cap2_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.1);
    std::vector<reach_lib::Capsule> r_caps_2 = {r_cap1_2, r_cap2_2};
    reach_lib::Capsule h_cap1_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.5);
    reach_lib::Capsule h_cap2_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.5);
    std::vector<std::vector<reach_lib::Capsule>> h_caps_2 = {{h_cap1_2, h_cap2_2}};
    std::vector<std::vector<double>> robot_link_velocities = {{1.0, 2.0}};
    std::vector<std::vector<double>> robot_link_reflected_masses = {{2.0, 1.0}};
    std::vector<std::vector<double>> maximal_contact_energies = {{3.0, 4.0}};

    std::vector<std::vector<reach_lib::Capsule>> robot_reachable_sets = {r_caps_2};
    std::vector<std::vector<std::vector<reach_lib::Capsule>>> human_reachable_sets = {h_caps_2};

    int collision_index = 0;
    EXPECT_TRUE(verify_iso_.verifyHumanReachEnergyReflectedMasses(robot_reachable_sets, human_reachable_sets, robot_link_velocities, robot_link_reflected_masses, maximal_contact_energies, collision_index));
    robot_link_velocities = {{1.0, 3.0}};
    EXPECT_FALSE(verify_iso_.verifyHumanReachEnergyReflectedMasses(robot_reachable_sets, human_reachable_sets, robot_link_velocities, robot_link_reflected_masses, maximal_contact_energies, collision_index));
}

TEST_F(VerifyIsoTest, VerifyHumanReachEnergyInertiaMatricesTest) {
    // Test case 1: Normal case - safe scenario
    // Test case: collision can occur between all capsules
    reach_lib::Capsule r_cap1_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.1);
    reach_lib::Capsule r_cap2_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.1);
    std::vector<reach_lib::Capsule> r_caps_2 = {r_cap1_2, r_cap2_2};
    reach_lib::Capsule h_cap1_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.5);
    reach_lib::Capsule h_cap2_2(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 0.0), 0.5);
    std::vector<std::vector<reach_lib::Capsule>> h_caps_2 = {{h_cap1_2, h_cap2_2}};
    std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> robot_inertia_matrices = {
        {
            (Eigen::Matrix<double, 2, 2>() << 1.0, 0.0, 0.0, 0.0).finished(),
            (Eigen::Matrix<double, 2, 2>() << 1.0, 0.0, 0.0, 1.0).finished()
        }
    };
    std::vector<Motion> motions = {
        Motion(2)
    };
    motions[0].setVelocity({1.0, 2.0});
    std::vector<std::vector<double>> maximal_contact_energies = {{3.0, 4.0}};

    std::vector<std::vector<reach_lib::Capsule>> robot_reachable_sets = {r_caps_2};
    std::vector<std::vector<std::vector<reach_lib::Capsule>>> human_reachable_sets = {h_caps_2};

    int collision_index = 0;
    EXPECT_TRUE(verify_iso_.verifyHumanReachEnergyInertiaMatrices(robot_reachable_sets, human_reachable_sets, robot_inertia_matrices, motions, maximal_contact_energies, collision_index));
    motions[0].setVelocity({2.0, 3.0});
    EXPECT_FALSE(verify_iso_.verifyHumanReachEnergyInertiaMatrices(robot_reachable_sets, human_reachable_sets, robot_inertia_matrices, motions, maximal_contact_energies, collision_index));
}
}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}