// -*- lsst-c++ -*/
/**
 * @file human_reach_fixture.h
 * @brief Defines the test fixture for the human reach object
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
#include <iostream>
#include <string>
#include <vector>

#include "reach_lib.hpp"
#include "safety_shield/human_reach.h"
#include "safety_shield/config_utils.h"

#ifndef HUMAN_REACH_FIXTURE_H
#define HUMAN_REACH_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for human reach class
 */
class HumanReachTest : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint.yaml";
    human_reach_ = buildHumanReach(config_file.string());
  }
};

/**
 * @brief Test fixture for human reach class with KF
 */
class HumanReachTestKF : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_kf_;
  HumanReach* human_reach_cm_kf_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file_kf =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint_kf.yaml";
    human_reach_kf_ = buildHumanReach(config_file_kf.string());
    std::filesystem::path config_file_cm_kf =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint_cm_kf.yaml";
    human_reach_cm_kf_ = buildHumanReach(config_file_cm_kf.string());
  }
};

/**
 * @brief Test fixture for human reach class with only combined model
 */
class HumanReachCombinedOnlyTest : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint_cm.yaml";
    human_reach_ = buildHumanReach(config_file.string());
  }
};

/**
 * @brief Test fixture for human reach class with only combined model
 */
class HumanReachCombinedOnlyArmTest : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_arm_pos_cm.yaml";
    human_reach_ = buildHumanReach(config_file.string());
  }
};

/**
 * @brief Test fixture for human reach class with errors and delay
 */
class HumanReachTestError : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint_error.yaml";
    human_reach_ = buildHumanReach(config_file.string());
  }
};

/**
 * @brief Test fixture for human reach class for the position model
 */
class HumanReachTestPos : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_arm_pos.yaml";
    human_reach_ = buildHumanReach(config_file.string());
  }
};

/**
 * @brief Test fixture for human reach timestep class with single joint
 */
class HumanReachTimeIntervalTestSingleJoint : public ::testing::Test {
 protected:
  /**
   * @brief The human reach objects
   */
  HumanReach* human_reach_single_joint_standard_;
  HumanReach* human_reach_single_joint_time_interval_;

  /**
   * @brief Create the human reach objects
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint.yaml";
    human_reach_single_joint_standard_ = buildHumanReach(config_file.string());
    human_reach_single_joint_time_interval_ = buildHumanReach(config_file.string());
  }
};


/**
 * @brief Test fixture for human reach time interval class with test arm
 */
class HumanReachTimeIntervalTestArm : public ::testing::Test {
 protected:
  /**
   * @brief The human reach objects
   */
  HumanReach* human_reach_test_arm_standard_;
  HumanReach* human_reach_test_arm_time_interval_;

  /**
   * @brief Create the human reach objects
   */
  void SetUp() override {
    std::filesystem::path config_file =
        std::filesystem::current_path().parent_path() / "config/human_reach_test_arm_pos.yaml";
    human_reach_test_arm_standard_ = buildHumanReach(config_file.string());
    human_reach_test_arm_time_interval_ = buildHumanReach(config_file.string());
  }
};

}  // namespace safety_shield

#endif  // HUMAN_REACH_FIXTURE_H