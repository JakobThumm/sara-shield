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
   * @param[in] human_capsules List of list of capsules. Each list of capsules corresponds to a human reachable set
   * model.
   *
   * @returns Whether the robot movement is unsafe for the human
   */
  virtual bool verify_human_reach(const std::vector<reach_lib::Capsule>& robot_capsules,
                                  std::vector<std::vector<reach_lib::Capsule>> human_capsules) = 0;

  /**
   * @brief For a given human capsule, find all robot capsules in contact and return their indices.
   * 
   * @param human_capsule Human capsule to check for contact with robot capsules.
   * @param robot_capsules List of robot capsules.
   * @returns List of indices of robot capsules in contact with the given human capsule.
   */
  virtual std::vector<int> find_human_robot_contact(const reach_lib::Capsule& human_capsule,
      const std::vector<reach_lib::Capsule>& robot_capsules) = 0;

  /**
   * @brief For a given set of human capsule, find all robot capsules in contact and return their indices.
   * 
   * @param human_capsule Human capsule to check for contact with robot capsules.
   * @param robot_capsules List of robot capsules.
   * @returns Map that maps a list of robot link indices to the human capsule they are in contact with.
   */
  virtual std::map<int, std::vector<int>> find_all_human_robot_contacts(const std::vector<reach_lib::Capsule>& human_capsule,
      const std::vector<reach_lib::Capsule>& robot_capsules) = 0;

  /**
   * @brief Verify the robot motion against the reachable occupancy of the human for each separate time interval
   *
   * Pure virtual function.
   *
   * @param[in] robot_reachable_sets Reachable sets of the robot (first index is which timestep
   * and second index is which capsule).
   * @param[in] human_reachable_sets Reachable sets of the human (first index is which timestep,
   * second index is which model, and third index is which capsule).
   * @param[out] collision_index Index of the time interval where the collision happened
   *
   * @returns Whether the robot movement is unsafe for the human
   */
  virtual bool verify_human_reach_time_intervals(
    const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
    std::vector<std::vector<std::vector<reach_lib::Capsule>>> human_reachable_sets,
    int& collision_index
  ) = 0;

  /**
   * @brief Verify the robot motion against the reachable occupancy of the human for each separate time interval
   *
   * @param[in] robot_reachable_sets Reachable sets of the robot in each time interval. Size = [n_time_intervals, n_robot_links]
   * @param[in] human_reachable_sets Reachable sets of the human in each time interval. Size = [n_time_intervals, n_human_models, n_human_bodies]
   * @param[in] robot_link_velocities The maximal velocity of each robot link in each time interval. Size = [n_time_intervals, n_robot_links]
   * @param[in] maximal_contact_velocities The maximal contact velocity for each human body part. Size = [n_human_models, n_human_bodies]
   * @param[out] collision_index The index of the time step where the collision occured
   *
   * @returns True: if the robot capsules do not collide with one set of the human capsules in each time step, i.e., the motion is safe.
   *          False: Otherwise
   */
  virtual bool verify_human_reach_velocity(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
    const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
    const std::vector<std::vector<double>>& robot_link_velocities,
    const std::vector<std::vector<double>>& maximal_contact_velocities,
    int& collision_index) = 0;
};
}  // namespace safety_shield

#endif  // VERIFY_H
