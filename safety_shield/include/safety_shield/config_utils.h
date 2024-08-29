// -*- lsst-c++ -*/
/**
 * @file config_utils.h
 * @brief Defines functions for reading in configuration files
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

#include <vector>
#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

#include "safety_shield/robot_reach.h"
#include "safety_shield/human_reach.h"
#include "spdlog/spdlog.h"

#ifndef CONFIG_UTILS_H
#define CONFIG_UTILS_H

namespace safety_shield {

/**
 * @brief Build the robot reach object from the robot configuration file
 * 
 * @param[in] robot_config_file Path to the robot configuration file
 * @param[in] init_x Base x pos
 * @param[in] init_y Base y pos
 * @param[in] init_z Base z pos
 * @param[in] init_roll Base roll
 * @param[in] init_pitch Base pitch
 * @param[in] init_yaw Base yaw
 * @return RobotReach* the robot reach object
 */
RobotReach* buildRobotReach(
  std::string robot_config_file,
  double init_x,
  double init_y,
  double init_z,
  double init_roll,
  double init_pitch,
  double init_yaw
);

/**
 * @brief Read the trajectory configuration file
 * 
 * @param[in] trajectory_config_file Path to the trajectory configuration file
 * @param[out] max_s_stop Maximum stop distance
 * @param[out] q_min_allowed Minimum joint values
 * @param[out] q_max_allowed Maximum joint values
 * @param[out] v_max_allowed Maximum joint velocities
 * @param[out] a_max_allowed Maximum joint accelerations
 * @param[out] j_max_allowed Maximum joint jerks
 * @param[out] a_max_ltt Maximum joint accelerations for LTT
 * @param[out] j_max_ltt Maximum joint jerks for LTT
 * @param[out] v_safe Safe velocity
 * @param[out] alpha_i_max Maximum alpha_i
 * @param[out] velocity_method Velocity method
 * @param[out] reachability_set_interval_size Reachability set interval size
 */
void readTrajectoryConfig(
  std::string trajectory_config_file,
  double& max_s_stop,
  std::vector<double>& q_min_allowed,
  std::vector<double>& q_max_allowed,
  std::vector<double>& v_max_allowed,
  std::vector<double>& a_max_allowed,
  std::vector<double>& j_max_allowed,
  std::vector<double>& a_max_ltt,
  std::vector<double>& j_max_ltt,
  double& v_safe,
  double& alpha_i_max,
  int& velocity_method,
  int& reachability_set_interval_size
);

HumanReach* buildHumanReach(
  std::string human_config_file
);

} // namespace safety_shield
#endif // CONFIG_UTILS_H