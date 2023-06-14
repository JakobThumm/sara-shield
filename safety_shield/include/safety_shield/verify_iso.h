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
#include <unordered_map>
#include <unordered_set>

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
   * @brief For a given human capsule, find all robot capsules in contact and return their indices.
   * 
   * @param human_capsule Human capsule to check for contact with robot capsules.
   * @param robot_capsules List of robot capsules.
   * @returns List of indices of robot capsules in contact with the given human capsule.
   */
  std::vector<int> find_human_robot_contact(const reach_lib::Capsule& human_capsule,
      const std::vector<reach_lib::Capsule>& robot_capsules);
  
  /**
   * @brief For a given human capsule, find all environment elements in contact and return their indices.
   * 
   * @param human_capsule Human capsule to check for contact with environment elements.
   * @param environment_elements List of environment elements.
   * @returns List of indices of environment elements in contact with the given human capsule.
   */
  std::vector<int> find_human_environment_contact(const reach_lib::Capsule& human_capsule,
      const std::vector<reach_lib::AABB>& environment_elements);

  /**
   * @brief Build two maps, which map robot and environment elements to their human capsule in contact.
   * 
   * @param[in] robot_capsules List of robot capsules.
   * @param[in] human_capsules List of human capsules.
   * @param[in] environment_elements List of environment elements.
   * @param[out] robot_collision_map Maps robot capsule indices to human capsule indices in contact.
   * @param[out] environment_collision_map Maps environment element indices to human capsule indices in contact.
   */
  void build_contact_maps(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      std::unordered_map<int, std::vector<int>>& robot_collision_map,
      std::unordered_map<int, std::vector<int>>& environment_collision_map);
  
  /**
   * @brief Check if a self-constrained collision (SCC) could occur with the given human capsule.
   * 
   * @param robot_collisions Robot capsule indices in contact with the given human capsule.
   * @param unclampable_enclosures_map List of pairs of robot links that cannot cause clamping.
   * @param robot_capsules List of robot capsules.
   * @param d_human Human diameter.
   * @return true if SCC could occur
   * @return false if SCC cannot occur
   */
  bool self_constrained_collision_check(const std::vector<int>& robot_collisions,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      double d_human);

  /**
   * @brief Check if an environmentally constrained collision (ECC) could occur with the given human capsule.
   * 
   * @param robot_collisions Robot capsule indices in contact with the given human capsule. 
   * @param robot_capsules List of robot capsules.
   * @param environment_collisions Environment element indices in contact with the given human capsule.
   * @param environment_elements List of environment elements.
   * @param d_human Human diameter.
   * @return true if ECC could occur
   * @return false if ECC cannot occur
   */
  bool environmentally_constrained_collision_check(const std::vector<int>& robot_collisions,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      const std::vector<int>& environment_collisions,
      const std::vector<reach_lib::AABB>& environment_elements,
      double d_human);

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
