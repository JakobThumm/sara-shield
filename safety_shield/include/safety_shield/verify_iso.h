// -*- lsst-c++ -*/
/**
 * @file verify_iso.h
 * @brief Defines the verify ISO class
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
#include <set>

#include "reach_lib.hpp"
#include "safety_shield/verify.h"
#include "safety_shield/robot_reach.h"
#include "spdlog/spdlog.h"  // https://github.com/gabime/spdlog

#ifndef VERIFY_ISO_H
#define VERIFY_ISO_H

namespace safety_shield {

/**
 * @brief Verifies if a given robot motion is safe with respect to a humans motion
 */
class VerifyISO : public Verify {
 public:
  /**
   * @brief A basic VerifyISO constructor
   */
  VerifyISO() : Verify() {}

  /**
   * @brief Check a set of robot capsules if they collide with a set of human capsules
   *
   * @param[in] robot_capsules The robot capsules
   * @param[in] human_capsules The human occupancy capsules
   *
   * @returns Whether a collision between any two capsules of the robot and human set occured
   */
  bool robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules,
                           const std::vector<reach_lib::Capsule>& human_capsules);

  /**
   * @brief Verify the robot motion againt the reachability analysis of the human in position, velocity, and
   * acceleration
   *
   * @param[in] robot_capsules Reachable capsules of the robot
   * @param[in] human_capsules List of list of capsules. Each list of capsules corresponds to a human reachable set
   * model.
   *
   * @returns True: if the robot capsules do not collide with one set of the human capsules, i.e., the motion is safe.
   *          False: Otherwise
   */
  bool verify_human_reach(const std::vector<reach_lib::Capsule>& robot_capsules, 
      std::vector<std::vector<reach_lib::Capsule>> human_capsules);

  /**
   * @brief Verify if clamping between the robot, human, and environment is possible.
   * 
   * @details This function checks two types of constrained collisions.
   *    1) A human body part is clamped between the robot and the environment.
   *       We refer to this as an environmentally constrained collision (ECC).
   *    2) A human body part is clamped between two robot links.
   *       We refer to this as an internally constrained collision (ICC).
   * 
   * @param robot_capsules Reachable capsules of the robot
   * @param human_capsules List of human reachable set capsules.
   * @param environment_elements List of environment elements
   * @param human_radii List of radii of human body parts/extremities.
   * @param unclampable_enclosures_map List of pairs of robot links that cannot cause clamping.
   * @param robot_capsule_velocities_it Iterator pointing to the beginning of the list of robot capsule velocities for this short-term plan.
   * @param robot_capsule_velocities_end Iterator pointing to the end of the list of robot capsule velocities for this short-term plan.
   * @return true: if clamping between the given human model and the robot can occur
   * @return false: if no clamping can occur
   */
  bool clamping_possible(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<double>& human_radii,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end);

  /**
   * @brief Verify if clamping between the robot, human, and environment is possible.
   * 
   * @details This function checks two types of constrained collisions.
   *    1) A human body part is clamped between the robot and the environment.
   *       We refer to this as an environmentally constrained collision (ECC).
   *    2) A human body part is clamped between two robot links.
   *       We refer to this as an internally constrained collision (ICC).
   * 
   * @param robot_capsules Reachable capsules of the robot
   * @param human_capsules List of list of capsules. Each list of capsules corresponds to a human reachable set model.
   * @param environment_elements List of environment elements
   * @param human_radii List of list of radii. Each list of radii corresponds to a human reachable set model.
   * @param unclampable_enclosures_map List of pairs of robot links that cannot cause clamping.
   * @param robot_capsule_velocities_it Iterator pointing to the beginning of the list of robot capsule velocities for this short-term plan.
   * @param robot_capsule_velocities_end Iterator pointing to the end of the list of robot capsule velocities for this short-term plan.
   * @return true: if no clamping can occur
   * @return false: if clamping can occur
   */
  bool verify_clamping(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<std::vector<reach_lib::Capsule>>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<std::vector<double>>& human_radii,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end);
};
}  // namespace safety_shield

#endif  // VERIFY_ISO_H
