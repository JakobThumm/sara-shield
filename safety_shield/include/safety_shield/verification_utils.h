// -*- lsst-c++ -*/
/**
 * @file verification_utils.h
 * @brief Defines utility functions for verification
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

#include <Eigen/Dense>
#include <stdexcept>
#include <vector>
#include <map>

#include "safety_shield/motion.h"
#include "reach_lib.hpp"
#include "spdlog/spdlog.h"

#ifndef VERIFICATION_UTILS_H
#define VERIFICATION_UTILS_H

namespace safety_shield {

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
 * @brief For a given human capsule, find all robot capsules in contact and return their indices.
 * 
 * @param human_capsule Human capsule to check for contact with robot capsules.
 * @param robot_capsules List of robot capsules.
 * @returns List of indices of robot capsules in contact with the given human capsule.
 */
std::vector<int> findHumanRobotContact(const reach_lib::Capsule& human_capsule,
    const std::vector<reach_lib::Capsule>& robot_capsules);

/**
 * @brief For a given set of human capsule, find all robot capsules in contact and return their indices.
 * 
 * @param human_capsule Human capsule to check for contact with robot capsules.
 * @param robot_capsules List of robot capsules.
 * @returns Map that maps a list of robot link indices to the human capsule they are in contact with.
 *        The key is the human capsule index and the value is a list of robot link indices.
 */
std::map<int, std::vector<int>> findAllHumanRobotContacts(const std::vector<reach_lib::Capsule>& human_capsule,
    const std::vector<reach_lib::Capsule>& robot_capsules);

/**
 * @brief Checks if a contact map is empty.
 * 
 * @param contact_map map that contains list of integers for each key.
 * @return true if all lists are empty, false otherwise.
 */
inline bool checkNoContactsInMap(const std::map<int, std::vector<int>>& contact_map) {
  for (const auto& contact : contact_map) {
    if (!contact.second.empty()) {
      return false;
    }
  }
  return true;
};

std::vector<double> calculateRobotLinkEnergies(
  const std::vector<double>& robot_link_velocities,
  const std::vector<double>& robot_link_reflected_masses
);

/**
 * @brief Check if any link that gets into contact with the human is too fast.
 * 
 * @param human_robot_contacts Maps the human capsule index to the list of robot link indices in contact.
 *  Key is the human capsule index, value is the list of robot link indices.
 * @param robot_link_energies Robot link energy at the time of contact.
 * @param maximal_contact_energies Maximal admissible human velocities.
 * @return true All robot links are slower than the maximal contact velocity.
 * @return false Otherwise.
 */
bool checkContactEnergySafety(
  const std::map<int, std::vector<int>>& human_robot_contacts,
  const std::vector<double>& robot_link_energies,
  const std::vector<double>& maximal_contact_energies
);

/**
 * @brief Check if any link that gets into contact with the human is too fast.
 * 
 * @param human_robot_contacts Maps the human capsule index to the list of robot link indices in contact.
 *  Key is the human capsule index, value is the list of robot link indices.
 * @param robot_link_velocities Robot link velocity at the time of contact.
 * @param maximal_contact_velocities Maximal admissible human velocities.
 * @return true All robot links are slower than the maximal contact velocity.
 * @return false Otherwise.
 */
bool checkVelocitySafety(
  const std::map<int, std::vector<int>>& human_robot_contacts,
  const std::vector<double>& robot_link_velocities,
  const std::vector<double>& maximal_contact_velocities
);

/**
 * @brief Calculate the maximal Cartesian robot link velocities for each time interval.
 * 
 * @param robot_motions The motion of the robot at each edge of the time intervals.
 * @param velocity_error The maximal velocity error for each robot link.
 * @return std::vector<std::vector<double>> Maximal Cartesian velocity of each robot link for each time interval.
 */
std::vector<std::vector<double>> calculateMaxRobotLinkVelocitiesPerTimeInterval(
  const std::vector<Motion>& robot_motions,
  const std::vector<double>& velocity_error
);

/**
 * @brief Calculate the maximal robot link energies for each time interval.
 * 
 * @param robot_link_velocities The maximal velocity of each robot link in each time interval.
 * @param robot_link_reflected_masses The maximal reflected masses of each robot link in each time interval.
 * @return std::vector<std::vector<double>> Maximal energy of each robot link for each time interval.
 */
std::vector<std::vector<double>> calculateMaxRobotEnergiesFromReflectedMasses(
  const std::vector<std::vector<double>>& robot_link_velocities,
  const std::vector<std::vector<double>>& robot_link_reflected_masses
);

/**
 * @brief Calculate the maximal robot link energies for each time interval.
 * 
 * @param robot_inertia_matrices The inertia matrices of the robot links in each time interval.
 * @param dq The joint velocities of the robot in each time interval.
 * @return std::vector<std::vector<double>> Maximal energy of each robot link for each time interval.
 */
std::vector<std::vector<double>> calculateMaxRobotEnergiesFromInertiaMatrices(
  const std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>>& robot_inertia_matrices,
  std::vector<std::vector<double>> dq
);

/**
 * @brief Calculate the velocity error for a given robot link.
 * 
 * @param alpha Maximal Cartesian acceleration of the robot link in the given time frame.
 * @param beta Maximal angular velocity of the robot link in the given time frame.
 * @param delta_s Distance travelled by the robot link measured by path length.
 * @param r Radius of the robot capsule.
 * @return double Velocity error for the given robot link.
 */
inline double calculateVelocityError(
  double alpha,
  double beta,
  double delta_s,
  double r
) {
  return 0.5 * delta_s * (alpha + beta * r);
}

}  // namespace safety_shield

#endif // VERIFICATION_UTILS_H