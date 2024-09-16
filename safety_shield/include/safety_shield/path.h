// -*- lsst-c++ -*/
/**
 * @file path.h
 * @brief Defines the path class
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

#include <stdexcept>
#include <array>
#include <iostream>

#include "spdlog/spdlog.h"

#ifndef PATH_H
#define PATH_H

namespace safety_shield {

/**
 * @brief Exception thrown when the human model is not found in the list.
 */
class PathVelocityNegativeException : public std::exception {
  std::string msg_;
 public:

  PathVelocityNegativeException(double vel)
    : msg_(std::string("[PathVelocityNegativeException] The path velocity ") + std::to_string(vel) +
           std::string(" is below zero."))
  {}

  virtual const char* what() const throw()
  {
    return msg_.c_str();
  }
};

/**
 * @brief A class that represents a path
 */
class Path {
 private:
  /**
   * @brief The end times of the three phases of the path
   * 
   * @details These values are continuously decremented by the sample time
   */
  std::array<double, 3> times_;

  /**
   * @brief The jerks of the three phases of the path
   */
  std::array<double, 3> jerks_;

  /**
   * @brief The starting position
   */
  double pos_;

  /**
   * @brief The starting velocity
   */
  double vel_;

  /**
   * @brief The starting acceleration
   */
  double acc_;

  /**
   * @brief The starting jerk
   */
  double jerk_;

  /**
   * @brief Is the path the current one?
   */
  bool is_current_;

  /**
   * @brief Calculate the position after the time duration
   *
   * @param p_0 starting position
   * @param dt time duration
   * @param v velocity
   * @param a acceleration
   * @param j jerk
   * @return double position after the time duration
   */
  inline double calculatePos(double p_0, double dt, double v, double a, double j) const {
    return p_0 + v * dt + 0.5 * a * dt * dt + j * dt * dt * dt / 6;
  }

  /**
   * @brief Calculate the velocity after the time duration
   * 
   * @details Clips the velocity to 0 if it is negative
   * 
   * @param v_0 starting velocity
   * @param dt time duration
   * @param a acceleration
   * @param j jerk
   * @return double velocity after the time duration
   */
  inline double calculateVel(double v_0, double dt, double a, double j) const {
    double v = v_0 + a * dt + 0.5 * j * dt * dt;
    if (v < 0) {
      throw PathVelocityNegativeException(v);
    } else {
      return v;
    }
  }

  /**
   * @brief Calculate the acceleration after the time duration
   * 
   * @param a_0 starting acceleration
   * @param dt time duration
   * @param j jerk
   * @return double acceleration after the time duration
   */
  inline double calculateAcc(double a_0, double dt, double j) const {
    return a_0 + j * dt;
  }

  /**
   * @brief Increment the motion by the given jerk
   * 
   * @param[out] pos current position -> will be updated 
   * @param[out] vel current velocity -> will be updated
   * @param[out] acc current acceleration -> will be updated
   * @param[in] jerk current jerk -> will not be updated
   * @param[in] duration duration
   */
  void incrementMotion(double& pos, double& vel, double& acc, const double& jerk, const double& duration);

 public:
  /**
   * @brief empty path constructor
   * Initializes a path where every parameter is set to 0 except the velocity setted to 1
   */
  Path();

  /**
   * @brief A path destructor
   */
  ~Path() {}

  inline double calculateTimeForVel(double v_limit, double v, double a, double j) const {
    if (std::abs(v - v_limit) < 1e-10) {
      return 0;
    }
    // Solve v_limit = prev_vel + prev_acc * dt + jerk * dt^2 / 2 for dt
    double square = std::sqrt(a * a - 2 * j * (v - v_limit));
    double left = (-a - square) / j;
    double right = (-a + square) / j;
    if (left > 0 && right > 0) {
      return std::min(left, right);
    } else if (left > 0) {
      return left;
    } else if (right > 0) {
      return right;
    } else {
      // already under v_iso
      if (std::isnan(square)) {
        spdlog::error("Error in Path::getMotionUnderVel: square is NaN");
      } else {
        spdlog::error("Error in Path::getMotionUnderVel: Quadratic-Function only has negative zero-values");
      }
      return -10;
    }
  }

  /**
   * @brief Returns the phases
   */
  inline std::array<double, 3> getTimes() const {
    return times_;
  }

  /**
   * @brief Returns the jerks
   */
  inline std::array<double, 3> getJerks() const {
    return jerks_;
  }

  /**
   * @brief Prints the path as an array.
   *
   * Format : [phases, is_current, pos, vel, acc]
   */
  inline void printPath() const {
    std::cout << times_[0] << "," << times_[1] << "," << times_[2] << "," << jerks_[0] << "," << jerks_[1] << ","
              << jerks_[2] << "," << is_current_ << "," << pos_ << "," << vel_ << "," << acc_ << std::endl;
  }

  /**
   * @brief Returns a given phase's end time
   *
   * @param i the phase number
   * @return the associated end time
   */
  inline double getTime(int phase) {
    return times_[phase];
  }

  /**
   * @brief Returns a given phase's jerk
   *
   * @param phase the phase number
   * @return the associated jerk
   */
  inline double getJerk(int phase) {
    return jerks_[phase];
  }

  /**
   * @brief Returns the path starting position
   *
   * @return the starting position
   */
  inline double getPosition() {
    return pos_;
  }

  /**
   * @brief Returns the path starting velocity
   *
   * @return the starting velocity
   */
  inline double getVelocity() {
    if (vel_ < 0) {
      spdlog::debug("Negative path velocity detected. Setting to 0.");
      return 0;
    } else {
      return vel_;
    }
  }

  /**
   * @brief Returns the path starting acceleration
   *
   * @return the starting acceleration
   */
  inline double getAcceleration() {
    return acc_;
  }

  /**
   * @brief Returns the path starting jerk
   *
   * @return the starting jerk
   */
  inline double getJerk() {
    return jerk_;
  }

  /**
   * @brief Returns the information about the current path
   *
   * @return is this path the current one ?
   */
  inline bool isCurrent() {
    return is_current_;
  }

  /**
   * @brief Sets the path phases
   *
   * @param times the new end times of the phases
   * @param jerks the new jerks of the phases
   */
  inline void setPhases(const std::array<double, 3>& times, const std::array<double, 3>& jerks) {
    for (int i = 0; i < 3; i++) {
      if (times[i] < 0) {
        throw std::invalid_argument("Time " + std::to_string(i) + " = " + std::to_string(times[i]) + " is smaller than zero");
      }
      if (i > 0 && times[i] < times[i - 1]) {
        throw std::invalid_argument("times[" + std::to_string(i) + "]= " + std::to_string(times[i]) + " is smaller than times[" + std::to_string(i-1) + "]= " + std::to_string(times[i-1]));
      }
    }
    times_ = times;
    jerks_ = jerks;
  }

  /**
   * @brief Sets the current path information
   *
   * @param is_current the new current path information
   */
  inline void setCurrent(bool is_current) {
    is_current_ = is_current;
  }

  /**
   * @brief Sets the starting position
   *
   * @param pos the new starting position
   */
  inline void setPosition(double pos) {
    pos_ = pos;
  }

  /**
   * @brief Sets the starting velocity
   *
   * @param vel the new starting velocity
   */
  inline void setVelocity(double vel) {
    vel_ = vel;
  }

  /**
   * @brief Sets the starting acceleration
   *
   * @param acc the new starting acceleration
   */
  inline void setAcceleration(double acc) {
    acc_ = acc;
  }

  /**
   * @brief Increments the path by the given duration
   *
   * @param duration the duration to increment the path by
   */
  void increment(double duration);

  /**
   * @brief Return the last position, velocity, and acceleration of the path.
   * @details Please note that the final jerk is always zero and therefore not returned.
   *
   * @param[out] final_pos last position
   * @param[out] final_vel last velocity
   * @param[out] final_acc last acceleration
   */
  void getFinalMotion(double& final_pos, double& final_vel, double& final_acc);

  /**
   * @brief Computes s-values when path falls under v_limit.
   * 
   * @details Not used in the current implementation and therefore not tested.
   * @param[in] v_limit velocity limit
   * @param[out] time time when velocity falls under v_limit
   * @param[out] position position when velocity falls under v_limit
   * @param[out] vel velocity when velocity falls under v_limit
   * @param[out] acc acceleration when velocity falls under v_limit
   * @param[out] jerk jerk when velocity falls under v_limit
   */
  void getMotionUnderVel(double v_limit, double& time, double& position, double& vel, double& acc, double& jerk);

  /**
   * @brief Get the maximum velocity of the path.
   * 
   * @return double: maximium velocity
   */
  double getMaxVelocity();
};
}  // namespace safety_shield

#endif  // PATH_H