// -*- lsst-c++ -*/
/**
 * @file long_term_traj_fixture.h
 * @brief Defines the test fixture for verify long term trajectory class
 * @version 0.1
 * @copyright This file is part of SaRA-Shield.
 * SaRA-Shield is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 * SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with SaRA-Shield. 
 * If not, see <https://www.gnu.org/licenses/>. 
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "safety_shield/motion.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/config_utils.h"
#include "safety_shield/trajectory_utils.h"

#ifndef LONG_TERM_TRAJ_FIXTURE_H
#define LONG_TERM_TRAJ_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for verify long term trajectory class
 */
class LongTermTrajTest : public ::testing::Test {
 protected:
  /**
   * @brief The LTT object
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief Create the LTT object
   */
  void SetUp() override {
    long_term_trajectory_ = LongTermTraj();
  }
};

/**
 * @brief Test fixture for verify long term trajectory class with starting index
 */
class LongTermTrajTestIdx : public ::testing::Test {
 protected:
  /**
   * @brief The LTT object
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief Create the LTT object
   */
  void SetUp() override {
    std::vector<Motion> mo_vec;
    int n_joints = 2;
    std::vector<double> p0;
    std::vector<double> v0;
    std::vector<double> j0;
    for (int i = 0; i < n_joints; i++) {
        p0.push_back(0.0);
        v0.push_back(0.0);
        j0.push_back(0.0);
    }
    std::vector<double> a0;
    a0.push_back(12);
    a0.push_back(13);
    Motion m0(0, p0, v0, a0, j0);
    mo_vec.push_back(m0);
    std::vector<double> a1;
    a1.push_back(1);
    a1.push_back(2);
    Motion m1(1, p0, v0, a1, j0);
    mo_vec.push_back(m1);
    std::vector<double> a2;
    a2.push_back(78);
    a2.push_back(79);
    Motion m2(2, p0, v0, a2, j0);
    mo_vec.push_back(m2);
    std::vector<double> a3;
    a3.push_back(90);
    a3.push_back(91);
    Motion m3(3, p0, v0, a3, j0);
    mo_vec.push_back(m3);
    long_term_trajectory_ = LongTermTraj(mo_vec, 0.001, 3, {100.0, 100.0}, {1000.0, 1000.0}, {10000.0, 10000.0});
  }
};

/**
 * @brief Test fixture for verify long term trajectory class with starting index
 */
class LongTermTrajTestInterpolation : public ::testing::Test {
 protected:
  /**
   * @brief The LTT object
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief Create the LTT object
   */
  void SetUp() override {
    std::vector<Motion> mo_vec;
    int n_joints = 1;
    std::vector<double> p0;
    std::vector<double> v0;
    std::vector<double> a0;
    std::vector<double> j0;
    for (int i = 0; i < n_joints; i++) {
        p0.push_back(0.0);
        v0.push_back(1.0);
        a0.push_back(0.0);
        j0.push_back(0.0);
    }
    Motion m0(0, p0, v0, a0, j0);
    mo_vec.push_back(m0);
    std::vector<double> p1;
    p1.push_back(1);
    Motion m1(1, p1, v0, a0, j0);
    mo_vec.push_back(m1);
    std::vector<double> p2;
    p2.push_back(2);
    Motion m2(2, p2, v0, a0, j0);
    mo_vec.push_back(m2);
    std::vector<double> p3;
    p3.push_back(3);
    Motion m3(3, p3, v0, a0, j0);
    mo_vec.push_back(m3);
    std::vector<double> p4;
    p4.push_back(4);
    Motion m4(4, p4, v0, a0, j0);
    mo_vec.push_back(m4);
    long_term_trajectory_ = LongTermTraj(mo_vec, 1.0, 0, {100.0}, {1000.0}, {10000.0});
  }
};

/**
 * @brief Test fixture for verify long term trajectory class with velocity calculation
 */
class LongTermTrajInterpolateWithVelocityTest : public ::testing::Test {
protected:
    /**
     * @brief The LTT objects
     */
    LongTermTraj long_term_trajectory_;
    RobotReach robot_reach_;


    /**
     * @brief Create the LTT object
     */
    void SetUp() override {
        // setup for tests with jacobian matrix, testing with schunk robot
        std::filesystem::path config_file = std::filesystem::current_path().parent_path() / "config/robot_reach_test_single_joint.yaml";
        robot_reach_ = *(buildRobotReach(config_file.string(), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
        robot_reach_.setVelocityMethod(RobotReach::VelocityMethod::EXACT);
        std::vector<Motion> motions;
        int n_joints = 1;
        std::vector<double> p0 = {0.0};
        std::vector<double> v0 = {1.0};
        std::vector<double> a0 = {1.0};
        std::vector<double> j0 = {-1.0};
        Motion m0(0, p0, v0, a0, j0);
        motions.push_back(m0);
        std::vector<double> p1 = {0.104833333333333};
        std::vector<double> v1 = {1.095};
        std::vector<double> a1 = {0.9};
        std::vector<double> j1 = {0.0};
        Motion m1(0.1, p1, v1, a1, j1);
        motions.push_back(m1);
        std::vector<double> v_max_allowed = {2.0};
        std::vector<double> a_max_allowed = {10.0};
        std::vector<double> j_max_allowed = {100.0};
        int starting_index = 0;
        long_term_trajectory_ = LongTermTraj(motions, 0.1, robot_reach_, 0, v_max_allowed, a_max_allowed, j_max_allowed);
    }
};


/**
 * @brief Test fixture for verify long term trajectory class with velocity calculation
 */
class LongTermTrajTestVelocity : public ::testing::Test {
protected:
    /**
     * @brief The LTT objects
     */
    LongTermTraj long_term_trajectory_approximate_;
    LongTermTraj long_term_trajectory_exact_;
    RobotReach robot_reach_approximate_;
    RobotReach robot_reach_exact_;


    /**
     * @brief Create the LTT object
     */
    void SetUp() override {
        // setup for tests with jacobian matrix, testing with schunk robot
        std::filesystem::path config_file = std::filesystem::current_path().parent_path() / "config/robot_parameters_schunk.yaml";
        robot_reach_approximate_ = *(buildRobotReach(config_file.string(), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
        robot_reach_exact_ = *(buildRobotReach(config_file.string(), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
        robot_reach_approximate_.setVelocityMethod(RobotReach::VelocityMethod::APPROXIMATE);
        robot_reach_exact_.setVelocityMethod(RobotReach::VelocityMethod::EXACT);
        std::vector<Motion> mo_vec;
        for(int i = 1; i < 10; ++i) {
            double dub = i;
            std::vector<double> q = {dub, dub+1, dub+2, dub+3, dub+4, dub+5};
            mo_vec.push_back(Motion(0, q, q));
        }
        std::vector<double> large_values = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};
        long_term_trajectory_approximate_ = LongTermTraj(mo_vec, 0.001, robot_reach_approximate_, 0, large_values, large_values, large_values);
        long_term_trajectory_exact_ = LongTermTraj(mo_vec, 0.001, robot_reach_exact_, 0, large_values, large_values, large_values);
    }
};

} // namespace safety_shield

#endif // LONG_TERM_TRAJ_FIXTURE_H