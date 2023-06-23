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
  Eigen::Vector3d expected_normal4(sqrt(1.0 / 3.0), sqrt(1.0 / 3.0), sqrt(1.0 / 3.0));
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
  EXPECT_TRUE(verify_iso_.capsule_moving_towards_element(capsule_velocity0, normal, radius, velocity_error));
  EXPECT_TRUE(verify_iso_.capsule_moving_towards_element(capsule_velocity1, normal, radius, velocity_error));
  EXPECT_TRUE(verify_iso_.capsule_moving_towards_element(capsule_velocity2, normal, radius, velocity_error));
  EXPECT_FALSE(verify_iso_.capsule_moving_towards_element(capsule_velocity3, normal, radius, velocity_error));
  EXPECT_TRUE(verify_iso_.capsule_moving_towards_element(capsule_velocity4, normal, radius, velocity_error));
  EXPECT_TRUE(verify_iso_.capsule_moving_towards_element(capsule_velocity5, normal, radius, velocity_error));
  EXPECT_FALSE(verify_iso_.capsule_moving_towards_element(capsule_velocity6, normal, radius, velocity_error));
}

// Write a test for this function
/*
   * @brief Check if an environmentally constrained collision (ECC) could occur with the given human capsule.
   *
   * @param robot_collisions Robot capsule indices in contact with the given human capsule.
   * @param robot_capsules List of robot capsules.
   * @param environment_collisions Environment element indices in contact with the given human capsule.
   * @param environment_elements List of environment elements.
   * @param d_human Human diameter.
   * @param robot_capsule_velocities_start Pointer to the start of the list of robot capsule velocities.
   * @param robot_capsule_velocities_end Pointer to the end of the list of robot capsule velocities.
   * @param alpha_i List of max cart acceleration values for each robot capsule.
   * @param beta_i List of max angular acceleration values for each robot capsule.
   * @param delta_s Time difference between two time steps.
   * @return true if ECC could occur
   * @return false if ECC cannot occur
  bool environmentally_constrained_collision_check(const std::vector<int>& robot_collisions,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      const std::vector<int>& environment_collisions,
      const std::vector<reach_lib::AABB>& environment_elements,
      double d_human,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_start,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_end,
      const std::vector<double>& alpha_i,
      const std::vector<double>& beta_i,
      double delta_s);
*/
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
  std::vector<double> alpha_i = {1.0};
  std::vector<double> beta_i = {10.0};
  double delta_s = 0.01;
  // Check if ECC could occur for upwards movement (no ECC should occur)
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_start =
      robot_capsule_velocities.begin();
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end =
      robot_capsule_velocities.end() - 3;
  EXPECT_FALSE(verify_iso_.environmentally_constrained_collision_check(
      robot_collisions, robot_capsules, environment_collisions, environment_elements, d_human,
      robot_capsule_velocities_start, robot_capsule_velocities_end, alpha_i, beta_i, delta_s));
  // Check if ECC could occur for downwards movement (ECC should occur)
  robot_capsule_velocities_start = robot_capsule_velocities.begin() + 3;
  robot_capsule_velocities_end = robot_capsule_velocities.end();
  EXPECT_TRUE(verify_iso_.environmentally_constrained_collision_check(
      robot_collisions, robot_capsules, environment_collisions, environment_elements, d_human,
      robot_capsule_velocities_start, robot_capsule_velocities_end, alpha_i, beta_i, delta_s));
}

}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}