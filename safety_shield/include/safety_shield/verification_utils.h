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

#include <assert.h>
#include <vector>
#include <map>

#include "safety_shield/motion.h"
#include "reach_lib.hpp"

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
std::vector<int> find_human_robot_contact(const reach_lib::Capsule& human_capsule,
    const std::vector<reach_lib::Capsule>& robot_capsules);

/**
 * @brief For a given set of human capsule, find all robot capsules in contact and return their indices.
 * 
 * @param human_capsule Human capsule to check for contact with robot capsules.
 * @param robot_capsules List of robot capsules.
 * @returns Map that maps a list of robot link indices to the human capsule they are in contact with.
 *        The key is the human capsule index and the value is a list of robot link indices.
 */
std::map<int, std::vector<int>> find_all_human_robot_contacts(const std::vector<reach_lib::Capsule>& human_capsule,
    const std::vector<reach_lib::Capsule>& robot_capsules);

}
#endif // VERIFICATION_UTILS_H