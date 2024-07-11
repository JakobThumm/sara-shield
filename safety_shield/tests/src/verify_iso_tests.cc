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
  EXPECT_DEATH(verify_iso_.verifyHumanReachTimeIntervals(r_caps_list_01, h_caps_list_012, collision_index), "");
  EXPECT_DEATH(verify_iso_.verifyHumanReachTimeIntervals(r_caps_list_012, h_caps_list_01, collision_index), "");
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
  EXPECT_DEATH(verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_012, robot_link_velocities_01, maximal_contact_velocities_high, collision_index), "");
  EXPECT_DEATH(verify_iso_.verifyHumanReachVelocity(r_caps_list_012, h_caps_list_01, robot_link_velocities_01, maximal_contact_velocities_high, collision_index), "");
  EXPECT_DEATH(verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_01, robot_link_velocities_012, maximal_contact_velocities_high, collision_index), "");
  EXPECT_DEATH(verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_01, robot_link_velocities_01, maximal_contact_velocities_wrongly_defined, collision_index), "");
  EXPECT_DEATH(verify_iso_.verifyHumanReachVelocity(r_caps_list_01, h_caps_list_01, robot_link_velocities_01, maximal_contact_velocities_wrongly_defined2, collision_index), "");
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

}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}