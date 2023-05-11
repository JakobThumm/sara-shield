// -*- lsst-c++ -*/
/**
 * @file verify.h
 * @brief Defines the abstract verify class
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

#include <algorithm>
#include <vector>

#include "reach_lib.hpp"

#ifndef VERIFY_H
#define VERIFY_H

namespace safety_shield {

/**
 * @brief Verifies if a given robot motion is safe with respect to a humans motion
 */
class Verify {
 public:
  /**
   * @brief A basic VerifyISO constructor
   */
  Verify() {}

  /**
   * @brief Check two capsules for collision
   * 
   * @param[in] cap1 Capsule 1
   * @param[in] cap2 Capsule 2
   * 
   * @returns true if capsules collide, false else
   */
  inline bool capsuleCollisionCheck(const reach_lib::Capsule& cap1, const reach_lib::Capsule& cap2) {
    return reach_lib::intersections::capsule_capsule_intersection(cap1, cap2);
  }
  
  /**
   * @brief Verify the robot motion against the reachable occupancy of the human in position, velocity, and acceleration
   * 
   * Pure virtual function.
   * 
   * @param[in] robot_capsules Reachable capsules of the robot
   * @param[in] human_capsules List of list of capsules. Each list of capsules corresponds to a human reachable set model.
   * 
   * @returns Whether the robot movement is unsafe for the human
   */
  virtual bool verify_human_reach(const std::vector<reach_lib::Capsule>& robot_capsules, 
      std::vector<std::vector<reach_lib::Capsule>> human_capsules) = 0;

  /**
   * @brief Verify if clamping between the robot, human, and environment is possible.
   * 
   * @param robot_capsules Reachable capsules of the robot
   * @param human_capsules List of list of capsules. Each list of capsules corresponds to a human reachable set model.
   * @param environment_elements List of environment elements
   * @param human_radii List of list of radii. Each list of radii corresponds to a human reachable set model.
   * @return true: if no clamping can occur
   * @return false: if clamping can occur
   */
  virtual bool verify_clamping(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<std::vector<reach_lib::Capsule>>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<std::vector<double>>& human_radii) = 0;
};
} // namespace safety_shield

#endif // VERIFY_H
