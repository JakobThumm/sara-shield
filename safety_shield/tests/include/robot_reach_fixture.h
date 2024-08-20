// -*- lsst-c++ -*/
/**
 * @file robot_reach_fixture.h
 * @brief Defines the test fixture for the robot reach object
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
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "reach_lib.hpp"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/trajectory_utils.h"
#include "safety_shield/config_utils.h"

#ifndef ROBOT_REACH_FIXTURE_H
#define ROBOT_REACH_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for robot reach class
 */
class RobotReachTest : public ::testing::Test {
 protected:
  /**
   * @brief The robot reach object
   */
  RobotReach* robot_reach_;

  /**
   * @brief Create the robot reach object
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/robot_reach_test_single_joint.yaml";
    robot_reach_ = buildRobotReach(config_file.string(), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  }
};

/**
 * @brief Test fixture for robot reach class with velocity calculation
 */
class RobotReachTestVelocity : public ::testing::Test {
 protected:
  /**
   * @brief The robot reach object
   */
  RobotReach* robot_reach_;

  /**
   * @brief Create the robot reach object
   */
  void SetUp() override {
    // setup for tests with jacobian matrix, testing with siciliano planar robot from p. 69 and p. 114
    // a1 = a2 = a3 = 1
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/robot_reach_test_siciliano.yaml";
    robot_reach_ = buildRobotReach(config_file.string(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
};

/**
 * @brief Test fixture for time interval version of robot reach
 */
class RobotReachTestTimeIntervals : public ::testing::Test {
 protected:
  /**
   * @brief The robot reach objects
   */
  RobotReach* robot_reach_single_joint_;
  RobotReach* robot_reach_siciliano_;

  /**
   * @brief The safety shield object
   */
  SafetyShield safety_shield_;


  /**
   * @brief Create the robot reach objects
   */
  void SetUp() override {
    std::filesystem::path config_file_single_joint =
        std::filesystem::current_path().parent_path() / "config/robot_reach_test_single_joint.yaml";
    std::filesystem::path config_file_siciliano =
        std::filesystem::current_path().parent_path() / "config/robot_reach_test_siciliano.yaml";
    robot_reach_single_joint_ = buildRobotReach(config_file_single_joint.string(), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    robot_reach_siciliano_ = buildRobotReach(config_file_siciliano.string(), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  }
};

}  // namespace safety_shield

#endif  // ROBOT_REACH_FIXTURE_H