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

#include "safety_shield/motion.h"
#include "safety_shield/long_term_traj.h"

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
} // namespace safety_shield

#endif // LONG_TERM_TRAJ_FIXTURE_H