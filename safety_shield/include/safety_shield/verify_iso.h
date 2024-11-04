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
  bool verifyHumanReach(const std::vector<reach_lib::Capsule>& robot_capsules,
                        std::vector<std::vector<reach_lib::Capsule>> human_capsules);

  /**
   * @brief Verify if clamping between the robot, all human models, and the environment is possible.
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
   * @param unclampable_body_part_maps List of pairs of human body parts that cannot cause clamping because they are part of the same body chain.
   * @param unclampable_enclosures_map List of pairs of robot links that cannot cause clamping.
   * @param robot_capsule_velocities_it Iterator pointing to the beginning of the list of robot capsule velocities for this short-term plan.
   * @param robot_capsule_velocities_end Iterator pointing to the end of the list of robot capsule velocities for this short-term plan.
   * @param velocity_errors upper bound of the velocity error for each joint
   * @return true: if no clamping can occur
   * @return false: if clamping can occur
   */
  bool verifyClamping(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<std::vector<reach_lib::Capsule>>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<std::vector<double>>& human_radii,
      const std::vector<std::unordered_map<int, std::set<int>>>& unclampable_body_part_maps,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end,
      std::vector<double> velocity_errors);

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
  bool verifyHumanReachTimeIntervals(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
                                         const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
                                         int& collision_index);

  /**
   * @brief Verify the robot motion against the reachable occupancy of the human for each separate time interval
   *
   * @param[in] robot_reachable_sets Reachable sets of the robot in each time interval. Size = [n_time_intervals, n_robot_links]
   * @param[in] human_reachable_sets Reachable sets of the human in each time interval. Size = [n_time_intervals, n_human_models, n_human_bodies]
   * @param[in] robot_link_velocities The maximal velocity of each robot link in each time interval. Size = [n_time_intervals, n_robot_links]
   * @param[in] robot_link_reflected_masses The reduced mass of each robot link. Size = [n_time_intervals, n_robot_links]
   * @param[in] maximal_contact_energies The maximal contact velocity for each human body part. Size = [n_human_models, n_human_bodies]
   * @param[out] collision_index The index of the time step where the collision occured
   *
   * @returns True: if the robot capsules do not collide with one set of the human capsules in each time step, i.e., the motion is safe.
   *          False: Otherwise
   */
  bool verifyHumanReachEnergyReflectedMasses(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
                                   const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
                                   const std::vector<std::vector<double>>& robot_link_velocities,
                                   const std::vector<std::vector<double>>& robot_link_reflected_masses,
                                   const std::vector<std::vector<double>>& maximal_contact_energies,
                                   int& collision_index);

    /**
   * @brief Verify the robot motion against the reachable occupancy of the human for each separate time interval
   *
   * @param[in] robot_reachable_sets Reachable sets of the robot in each time interval. Size = [n_time_intervals, n_robot_links]
   * @param[in] human_reachable_sets Reachable sets of the human in each time interval. Size = [n_time_intervals, n_human_models, n_human_bodies]
   * @param[in] robot_link_inertia_matrices The robot link inertia matrices per link. Size = [n_time_intervals, n_robot_links]
   * @param[in] motions The robot motions per timeinterval (used to get dq). Size = [n_time_intervals]
   * @param[in] maximal_contact_energies The maximal contact velocity for each human body part. Size = [n_human_models, n_human_bodies]
   * @param[out] collision_index The index of the time step where the collision occured
   *
   * @returns True: if the robot capsules do not collide with one set of the human capsules in each time step, i.e., the motion is safe.
   *          False: Otherwise
   */
  bool verifyHumanReachEnergyInertiaMatrices(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
                              const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
                              const std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>>& robot_inertia_matrices,
                              const std::vector<Motion>& motions,
                              const std::vector<std::vector<double>>& maximal_contact_energies,
                              int& collision_index);

  /**
   * @brief Verify the robot motion against the reachable occupancy of the human for each separate time interval
   *
   * @param[in] robot_reachable_sets Reachable sets of the robot in each time interval. Size = [n_time_intervals, n_robot_links]
   * @param[in] human_reachable_sets Reachable sets of the human in each time interval. Size = [n_time_intervals, n_human_models, n_human_bodies]
   * @param[in] robot_link_energies The maximal velocity of each robot link in each time interval. Size = [n_time_intervals, n_robot_links]
   * @param[in] maximal_contact_energies The maximal contact energies for each human body part. Size = [n_human_models, n_human_bodies]
   * @param[out] collision_index The index of the time step where the collision occured
   *
   * @returns True: if the robot capsules do not collide with one set of the human capsules in each time step, i.e., the motion is safe.
   *          False: Otherwise
   */
  bool verifyHumanReachEnergy(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
                                   const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
                                   const std::vector<std::vector<double>>& robot_link_energies,
                                   const std::vector<std::vector<double>>& maximal_contact_energies,
                                   int& collision_index);

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
  bool verifyHumanReachVelocity(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
                                   const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
                                   const std::vector<std::vector<double>>& robot_link_velocities,
                                   const std::vector<std::vector<double>>& maximal_contact_velocities,
                                   int& collision_index);
};
}  // namespace safety_shield

#endif  // VERIFY_ISO_H
