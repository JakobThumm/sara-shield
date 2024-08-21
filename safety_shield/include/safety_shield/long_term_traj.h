// -*- lsst-c++ -*/
/**
 * @file long_term_traj.h
 * @brief Defines the long term trajectory class
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

#include <algorithm>
#include <deque>
#include <utility>
#include <vector>

#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "spdlog/spdlog.h"  // https://github.com/gabime/spdlog

#ifndef LONG_TERM_TRAJ_H
#define LONG_TERM_TRAJ_H

namespace safety_shield {

/**
 * @brief A long term trajectory has a series of motions that the robot should move to.
 *
 * When the trajectory is over, it will output its last position forever.
 */
class LongTermTraj {
 private:
  /**
   * @brief Trajectory length.
   */
  int length_;

  /**
   * @brief Number of modules of the robot.
   */
  int nb_modules_;

  /**
   * @brief Time between two timesteps.
   */
  double sample_time_;

  /**
   * @brief The long term trajectory.
   */
  std::vector<Motion> long_term_traj_;

  
  /**
   * @brief maximum joint velocity allowed
   */
  std::vector<double> v_max_allowed_;

  /**
   * @brief maximum joint acceleration allowed
   */
  std::vector<double> a_max_allowed_;

  /**
   * @brief maximum joint jerk allowed
   */
  std::vector<double> j_max_allowed_;

  /**
   * @brief Maximum joint acceleration for next k steps.
   */
  std::vector<std::vector<double>> max_acceleration_window_;

  /**
   * @brief Maximum joint jerk for next k steps.
   */
  std::vector<std::vector<double>> max_jerk_window_;

  /**
   * @brief The current timestamp.
   */
  int current_pos_;

  /**
   * @brief Starting index if the trajectory is not initialized at s=0.
   */
  int starting_index_;

  /**
   * @brief maximum cartesian acceleration of robot joints (+ end effector!)
   */
  std::vector<double> alpha_i_;

  /**
   * @brief maximum angular acceleration of robot joints (+ end effector!)
   */
  std::vector<double> beta_i_;

  /**
   * @brief LTT-maximum of maximum cartesian velocity
   */
  double max_cart_vel_;

  /**
   * @brief The SE3 velocities of the robot for each capsule for each motion.
   * 
   * @details capsule_velocities_[motion][capsule]
   *    This is used in clamping prevention for the velocity criterion. 
   * @details This is saved here and not as part of the motions as the motion does not have a notion of a capsule.
   *    I.e., it does not depend on `RobotReach` and therefore does not have access to `RobotReach::CapsuleVelocity`.
   */
  std::vector<std::vector<RobotReach::CapsuleVelocity>> capsule_velocities_;

  /**
   * @brief The inertia matrices of the robot links in each time interval.
   */
  std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> inertia_matrices_;

  /**
   * @brief sets maximum cartesian velocity and the inertia matrices for all motions in LTT
   * @param[in] robot_reach used to calculate jacobians and velocities
   */
  void velocitiesOfAllMotions(RobotReach& robot_reach);

  /**
   * @brief Calculates the max cartesian acceleration and jerk of each joint during the LTT.
   * @sets alpha_i_
   * @sets beta_i_
   */
  void calculateAlphaBeta();

 public:
  /**
   * @brief Construct a new Long Term Traj object.
   */
  LongTermTraj() : current_pos_(0), starting_index_(0), length_(1), nb_modules_(1), sample_time_(0.01),
                    v_max_allowed_({1.0}), a_max_allowed_({1.0}), j_max_allowed_({1.0}) {
    long_term_traj_.push_back(Motion(1));
    for (int i = 0; i < 2; i++) {
      alpha_i_.push_back(1.0);
    }
  }

  /**
   * @brief Construct a new Long Term Traj object.
   *
   * @param long_term_traj Vector of motions that make up the LTT.
   * @param sample_time Sample time of the LTT.
   * @param starting_index Index of the first motion in the LTT
   * @param v_max_allowed Maximum allowed joint velocities
   * @param a_max_allowed Maximum allowed joint accelerations
   * @param j_max_allowed Maximum allowed joint jerks
   * @param sliding_window_k Size of sliding window for max acc and jerk calculation
   * @param alpha_i_max Maximum Carthesian acceleration
   */
  LongTermTraj(const std::vector<Motion>& long_term_traj, double sample_time, int starting_index,
               std::vector<double> v_max_allowed, std::vector<double> a_max_allowed, std::vector<double> j_max_allowed,
               int sliding_window_k = 10, double alpha_i_max = 1.0)
      : long_term_traj_(long_term_traj), sample_time_(sample_time), current_pos_(0), starting_index_(starting_index),
        v_max_allowed_(v_max_allowed), a_max_allowed_(a_max_allowed), j_max_allowed_(j_max_allowed) {
    length_ = long_term_traj_.size();
    assert (length_ > 0);
    nb_modules_ = long_term_traj_[0].getNbModules();
    calculateMaxAccJerkWindow(long_term_traj_, sliding_window_k);
    for (int i = 0; i < long_term_traj_[0].getNbModules(); i++) {
      alpha_i_.push_back(alpha_i_max);
    }
  }

  /**
   * @brief constructor for maximum velocity functionality of LTT
   *
   * @param long_term_traj Vector of motions that make up the LTT.
   * @param sample_time Sample time of the LTT.
   * @param robot_reach The robot that executes this trajectory. Needed for forward kinematics
   * @param starting_index Index of the first motion in the LTT
   * @param v_max_allowed Maximum allowed joint velocities
   * @param a_max_allowed Maximum allowed joint accelerations
   * @param j_max_allowed Maximum allowed joint jerks
   * @param sliding_window_k Size of sliding window for max acc and jerk calculation
   */
  LongTermTraj(const std::vector<Motion>& long_term_traj, double sample_time, RobotReach& robot_reach, int starting_index,
               std::vector<double> v_max_allowed, std::vector<double> a_max_allowed, std::vector<double> j_max_allowed, int sliding_window_k = 10);
  
  /**
   * @brief Destroy the Long Term Traj object
   */
  ~LongTermTraj() {}

  /**
   * @brief Interpolates the position, velocity, and acceleration from the trajectory at the given s position.
   * @details Mathematical explanation in http://mediatum.ub.tum.de/doc/1443612/652879.pdf eq. 2.4a and b (P.19).
   *
   * @param s the point's time
   * @param ds the percentage of the maximum path velocity, 0 = stand still, 1 = full velocity
   * @param dds the derivative of ds to time, 1 = accelerate from v=0 to full velocity in 1 second
   * @param ddds the derivative of dds to time
   * @return Motion
   */
  Motion interpolate(double s, double ds, double dds, double ddds) const;

  /**
   * @brief Increment the current pos
   */
  inline void increasePosition() {
    current_pos_ = std::min(current_pos_ + 1, length_ - 1);
  }

  /**
   * @brief Calculate the maximum acceleration and jerk for each timestep with a sliding window approach.
   *
   * Sets the `max_acceleration_window_` and `max_jerk_window_` values
   *
   * @param long_term_traj Long term trajectory
   * @param k sliding window size
   */
  void calculateMaxAccJerkWindow(std::vector<Motion>& long_term_traj, int k);

  /**
   * @brief Set the Long Term Trajectory object
   *
   * @param long_term_traj vector of motions to form the new long-term trajectory
   */
  inline void setLongTermTrajectory(const std::vector<Motion>& long_term_traj) {
    long_term_traj_ = long_term_traj;
    length_ = long_term_traj_.size();
    assert (length_ > 0);
    nb_modules_ = long_term_traj_[0].getNbModules();
  }

  /**
   * @brief Set the Long Term Trajectory object with new sample time
   *
   * @param long_term_traj vector of motions to form the new long term trajectory
   * @param sample_time the sample time of the new long-term trajectory
   */
  inline void setLongTermTrajectory(const std::vector<Motion>& long_term_traj, double sample_time) {
    long_term_traj_ = long_term_traj;
    length_ = long_term_traj_.size();
    assert (length_ > 0);
    nb_modules_ = long_term_traj_[0].getNbModules();
    sample_time_ = sample_time;
  }

  /**
   * @brief Set the starting position of the LTT.
   *
   * @param starting_index index at which the LTT starts.
   */
  inline void setStartingIndex(int starting_index) {
    starting_index_ = starting_index;
  }

  /**
   * @brief Set the maximum allowed joint velocities
   * 
   * @param v_max_allowed 
   */
  inline void setVMaxAllowed(const std::vector<double>& v_max_allowed) {
    v_max_allowed_ = v_max_allowed;
  }

  /**
   * @brief Set the maximum allowed joint accelerations
   * 
   * @param a_max_allowed 
   */
  inline void setAMaxAllowed(const std::vector<double>& a_max_allowed) {
    a_max_allowed_ = a_max_allowed;
  }

  /**
   * @brief Set the maximum allowed joint jerks
   * 
   * @param j_max_allowed 
   */
  inline void setJMaxAllowed(const std::vector<double>& j_max_allowed) {
    j_max_allowed_ = j_max_allowed;
  }

  /**
   * @brief Get the length of the LTT
   *
   * @return int length
   */
  inline int getLength() const {
    return length_;
  }

  /**
   * @brief Get the number of modules (joints of the robot).
   * 
   * @return int 
   */
  inline int getNbModules() const {
    return nb_modules_;
  }

  /**
   * @brief Get the current index of the LTT
   *
   * @return int current_pos_
   */
  inline int getCurrentPos() const {
    return current_pos_;
  }

  /**
   * @brief Get the current motion of the LTT.
   *
   * Equals getNextMotionAtIndex(0);
   *
   * @return Current motion
   */
  inline Motion getCurrentMotion() const {
    return long_term_traj_[current_pos_];
  }

  /**
   * @brief Get the next motion of the LTT
   *
   * Equals getNextMotionAtIndex(1);
   *
   * @return Next motion
   */
  inline Motion getNextMotion() const {
    return long_term_traj_[std::min(current_pos_ + 1, length_ - 1)];
  }

  /**
   * @brief Get the floor index of the LTT at the given s position.
   * 
   * @param s continuous trajectory time
   * @return int LTT index
   */
  inline int getLowerIndex(double s) const {
    assert(sample_time_ != 0);
    return getTrajectoryIndex(static_cast<int>(floor(s / sample_time_)));
  }

  /**
   * @brief Get the ceil index of the LTT at the given s position.
   * 
   * @param s continuous trajectory time
   * @return int LTT index
   */
  inline int getUpperIndex(double s) const {
    assert(sample_time_ != 0);
    return getTrajectoryIndex(static_cast<int>(ceil(s / sample_time_)));
  }

  /**
   * @brief Get the floating-point remainder of the index given s position and the lower LTT index.
   * 
   * @param s continuous trajectory time
   * @return int LTT index difference
   */
  inline double getModIndex(double s) const {
    assert(sample_time_ != 0);
    double ind = s / sample_time_;
    double intpart;
    return modf(ind, &intpart);
  }

  /**
   * @brief Get the motion at index from current pos
   *
   * @param index steps from current pos
   * @return Motion
   */
  inline Motion getNextMotionAtIndex(int index) const {
    return long_term_traj_[getTrajectoryIndex(index)];
  }

  inline int getTrajectoryIndex(int index) const {
    int desired_pos = std::min(index - starting_index_, length_ - 1);
    if (desired_pos < current_pos_) {
      desired_pos = current_pos_;
    }
    return desired_pos;
  }

  /**
   * @brief Get the starting index of the LTT
   * 
   * @return int 
   */
  inline int getStartingIndex() const {
    return starting_index_;
  }

  /**
   * @brief Get the maximum allowed joint velocities.
   * 
   * @return std::vector<double> 
   */
  inline std::vector<double> getVMaxAllowed() const {
    return v_max_allowed_;
  }

  /**
   * @brief Get the maximum allowed joint accelerations.
   * 
   * @return std::vector<double> 
   */
  inline std::vector<double> getAMaxAllowed() const {
    return a_max_allowed_;
  }

  /**
   * @brief Get the maximum allowed joint jerks.
   * 
   * @return std::vector<double> 
   */
  inline std::vector<double> getJMaxAllowed() const {
    return j_max_allowed_;
  }

  /**
   * @brief Get the Max Acceleration Window at index i
   *
   * @param index
   * @return std::vector<double>
   */
  inline std::vector<double> getMaxAccelerationWindow(int index) const {
    return max_acceleration_window_[getTrajectoryIndex(index)];
  }

  /**
   * @brief Get the Alpha I object
   * 
   * @return std::vector<double> 
   */
  inline std::vector<double> getAlphaI() const {
    return alpha_i_;
  }

  /**
   * @brief Get the Beta I object
   * 
   * @return std::vector<double> 
   */
  inline std::vector<double> getBetaI() const {
    return beta_i_;
  }

  /**
   * @brief Get the Max Jerk Window at index i
   *
   * @param index
   * @return std::vector<double>
   */
  inline std::vector<double> getMaxJerkWindow(int index) const {
    return max_jerk_window_[getTrajectoryIndex(index)];
  }

  /**
   * @brief gets specific motion of LTT
   *
   * @param index of motion in LTT
   */
  inline Motion getMotion(unsigned long index) const {
    return long_term_traj_.at(index);
  }

  /**
   * @brief gets the inertia matrices of the robot links in a given time step
   *
   * @param index of motion in LTT
   * @return std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> inertia matrices of the robot links
   */
  inline std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> getInertiaMatrices(int index) const {
    return inertia_matrices_[index];
  }

  /**
   * @brief Gets a const. pointer to a the velocity capsule given by the index
   *
   * @param index of motion in LTT
   */
  inline std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator getVelocityCapsuleIterator(int index) {
    assert (index >= 0 && index < capsule_velocities_.size());
    return capsule_velocities_.begin() + index;
  }

  /**
   * @brief gets LTT-maximum of maximum cartesian velocity
   */
  double getMaxofMaximumCartesianVelocity() const;

  /**
   * @brief gets maximum of cartesian velocity until s is reached
   * @param s look at Motions until trajectory_time s
   */
  double getMaxofMaximumCartesianVelocityWithS(double s) const;
};
}  // namespace safety_shield
#endif  // LONG_TERM_TRAJ_H
