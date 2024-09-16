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