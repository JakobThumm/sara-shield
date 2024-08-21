// -*- lsst-c++ -*/
/**
 * @file trajectory_utils.h
 * @brief Defines utility functions for trajectories
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
#include <assert.h>
#include <vector>

#include "safety_shield/motion.h"
#include "safety_shield/path.h"
#include "safety_shield/long_term_traj.h"

#ifndef TRAJECTORY_UTILS_H
#define TRAJECTORY_UTILS_H

namespace safety_shield {

/**
 * @brief Seperate the time horizon into equidistant intervals.
 * @param begin of time horizon
 * @param end of time horizon
 * @param reachability_set_duration desired duration of the time intervals. The last interval may be shorter.
 * @return list of edges of time intervals
 */
std::vector<double> calcTimePointsForEquidistantIntervals(double begin, double end, double reachability_set_duration);

/**
 * @brief Get the motions of all time points from a path on a long term trajectory.
 * 
 * @param ltt The long term trajectory to sample motions from.
 * @param path The path that defines the movement on the path.
 * @param time_points The time points at which to sample the motions.
 * @return std::vector<Motion> The motions
 */
std::vector<Motion> getMotionsOfAllTimeStepsFromPath(const LongTermTraj& ltt, const Path& path, const std::vector<double>& time_points);

/**
 * @brief Get the inertia matrices of all robot links of all time points from a path on a long term trajectory.
 * 
 * @param ltt The long term trajectory to sample motions from.
 * @param path The path that defines the movement on the path.
 * @param time_points The time points at which to sample the motions.
 * @return std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> 
 */
std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> getInertiaMatricesOfAllTimeStepsFromPath(
  const LongTermTraj& ltt, const Path& path, const std::vector<double>& time_points
);

} // namespace safety_shield

#endif // TRAJECTORY_UTILS_H