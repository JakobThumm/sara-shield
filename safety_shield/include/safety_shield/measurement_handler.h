// -*- lsst-c++ -*/
/**
 * @file measurement_handler.h
 * @brief Defines the MeasurementHandler class
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

#include <map>
#include <vector>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "reach_lib.hpp"
#include "spdlog/spdlog.h"
#include "kalman_filter.h"

#ifndef MEASUREMENT_HANDLER_H
#define MEASUREMENT_HANDLER_H

namespace safety_shield {

struct Observation {
  double time;
  std::vector<reach_lib::Point> position;
  std::vector<reach_lib::Point> velocity;
  std::vector<double> pos_variance;
  std::vector<double> vel_variance;
};

/**
 * @brief Class that handles the human measurements with the Kalman Filter
 */
class MeasurementHandler {
 private:
  /**
   * @brief Vector of Kalman Filters, one for each human joint
   */
  std::vector<KalmanFilter<6, 3>> kalman_filters_;

  /**
   * @brief Number of joint measurements.
   */
  int n_joints_meas_;
  
  /**
   * @brief Last measured time.
   */
  double last_meas_timestep_ = -1;

  /**
   * @brief Power spectral density of the system noise
   */
  double s_w_ = 2e2;
  
  /**
   * @brief Power spectral density of the measurement noise
   */
  double s_v_ = 1e-6;

  /**
   * @brief Variance of initial measured position.
   */
  double initial_pos_var_ = 0.003;

  /**
   * @brief Variance of initial velocity.
   */
  double initial_vel_var_ = 0.5;

  /**
   * @brief Transition matrix
   * 
   * @details This matrix is used to predict the next state of the Kalman Filter.
   * x(t+dt) = A(dt) * x(t) + B(dt) * u(t) + w(t)
   * We assume a constant velocity model without input, so:
   * A(dt) = [[1, dt, 0,  0, 0,  0],
   *          [0,  1, 0,  0, 0,  0],
   *          [0,  0, 1, dt, 0,  0],
   *          [0,  0, 0,  1, 0,  0],
   *          [0,  0, 0,  0, 1, dt],
   *          [0,  0, 0,  0, 0,  1]]
   */
  Matrix<6, 6> A_;

  /**
   * @brief The covariance matrix of the input noise 
   * 
   * @details Determines the uncertainty of the input, which is used to model system noise.
   * x(t+dt) = A(dt) * x(t) + B(dt) * u(t) + w(t)
   * We assume a constant velocity model and i.i.d. noise, so:
   * C_w = s_w_ * [[pow(dt,3)/3, pow(dt,2)/2,           0,           0,           0,           0],
   *               [pow(dt,2)/2,          dt,           0,           0,           0,           0],
   *               [          0,           0, pow(dt,3)/3, pow(dt,2)/2,           0,           0],
   *               [          0,           0, pow(dt,2)/2,          dt,           0,           0],
   *               [          0,           0,           0,           0, pow(dt,3)/3, pow(dt,2)/2],
   *               [          0,           0,           0,           0, pow(dt,2)/2,          dt]]
   */
  Matrix<6, 6> C_w_;

  /**
   * @brief Measurement matrix
   * 
   * @details This matrix is used to update the Kalman Filter with the new measurements.
   * y(t) = H * x(t) + v(t)
   * We assume a constant velocity model, so:
   * H = [[1, 0, 0, 0, 0, 0],
   *      [0, 0, 1, 0, 0, 0],
   *      [0, 0, 0, 0, 1, 0]]
   */
  Matrix<3, 6> H_;

  /**
   * @brief The covariance matrix of the measurement noise 
   * 
   * @details Determines the uncertainty of the measurement, which is used to model measurement noise.
   * y(t) = H * x(t) + v(t)
   * We assume i.i.d. noise, so:
   * C_v = s_v_ * [[1, 0, 0],
   *               [0, 1, 0],
   *               [0, 0, 1]]
   */
  Matrix<3, 3> C_v_;

  /**
   * @brief Build the system noise covariance matrix.
   * 
   * @param dt Prediction horizon.
   */
  void inline buildCw(double dt) {
    C_w_ << s_w_ * pow(dt, 3) / 3, s_w_ * pow(dt, 2) / 2, 0, 0, 0, 0,
            s_w_ * pow(dt, 2) / 2, s_w_ * dt, 0, 0, 0, 0,
            0, 0, s_w_ * pow(dt, 3) / 3, s_w_ * pow(dt, 2) / 2, 0, 0,
            0, 0, s_w_ * pow(dt, 2) / 2, s_w_ * dt, 0, 0,
            0, 0, 0, 0, s_w_ * pow(dt, 3) / 3, s_w_ * pow(dt, 2) / 2,
            0, 0, 0, 0, s_w_ * pow(dt, 2) / 2, s_w_ * dt;
  }

  /**
   * @brief Build the measurement noise covariance matrix.
   */
  void inline buildCv() {
    C_v_ = s_v_ * Matrix<3, 3>::Identity();
  }

  /**
   * @brief Build the transition matrix.
   * 
   * @param dt Prediction horizon.
   */
  void inline buildA(double dt) {
    A_ << 1, dt, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, dt, 0, 0,
          0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, dt,
          0, 0, 0, 0, 0, 1;
  }

  /**
   * @brief Build the measurement matrix.
   */
  void inline buildH() {
    H_ << 1, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 1, 0;
  }

  /**
   * @brief Set the initial measurement values of the Kalman filters.
   * 
   * @param measurements The initial measurements.
   */
  void initialMeasurement(const std::vector<reach_lib::Point>& measurements);

 public:
  /**
   * @brief Default constructor of the Measurement Handler object
   */
  MeasurementHandler() {
    n_joints_meas_ = 0;
    buildH();
    buildCv();
  }

  /**
   * @brief Construct a new Measurement Handler object with default values.
   * 
   * @param n_joints_meas Number of joint measurements.
   */
  MeasurementHandler(int n_joints_meas) :
    n_joints_meas_(n_joints_meas) {
    kalman_filters_.resize(n_joints_meas);
    for (int i = 0; i < n_joints_meas; i++) {
      kalman_filters_[i] = KalmanFilter<6, 3>();
    }
    buildH();
    buildCv();
  }

  /**
   * @brief Construct a new Measurement Handler object with custom values.
   * 
   * @param n_joints_meas Number of joint measurements.
   * @param s_w_ Power spectral density of the system noise
   * @param s_v_ Power spectral density of the measurement noise
   */
  MeasurementHandler(int n_joints_meas, double s_w_, double s_v_) :
    n_joints_meas_(n_joints_meas), s_w_(s_w_), s_v_(s_v_) {
    kalman_filters_.resize(n_joints_meas);
    for (int i = 0; i < n_joints_meas; i++) {
      kalman_filters_[i] = KalmanFilter<6, 3>();
    }
    buildH();
    buildCv();
  }

  /**
   * @brief Construct a new Measurement Handler object with custom values.
   * 
   * @param n_joints_meas Number of joint measurements.
   * @param s_w_ Power spectral density of the system noise
   * @param s_v_ Power spectral density of the measurement noise
   * @param initial_pos_var Variance of initial measured position.
   * @param initial_vel_var Variance of initial velocity.
   */
  MeasurementHandler(int n_joints_meas, double s_w_, double s_v_,
                     double initial_pos_var, double initial_vel_var) :
    n_joints_meas_(n_joints_meas), s_w_(s_w_), s_v_(s_v_),
    initial_pos_var_(initial_pos_var), initial_vel_var_(initial_vel_var) {
    kalman_filters_.resize(n_joints_meas);
    for (int i = 0; i < n_joints_meas; i++) {
      kalman_filters_[i] = KalmanFilter<6, 3>();
    }
    buildH();
    buildCv();
  }

  /**
   * @brief Destroy the Measurement Handler object
   */
  ~MeasurementHandler() {}

  /**
   * @brief Update the Kalman Filter with the new measurements.
   * 
   * @param measurements Vector of human joint measurements.
   * @param time Time of the measurements.
   * 
   * @return Observation Filtered human joint measurements.
   */
  Observation filterMeasurements(const std::vector<reach_lib::Point>& measurements, double time);

  /**
   * @brief Get the number of joint measurements.
   * 
   * @return int 
   */
  inline int getNJointsMeas() const {
    return n_joints_meas_;
  }

  /**
   * @brief Get the last measured time.
   * 
   * @return double 
   */
  inline double getLastMeasTimestep() const {
    return last_meas_timestep_;
  }

  /**
   * @brief Get the power spectral density of the measurement noise.
   * 
   * @return double 
   */
  inline double getSv() const {
    return s_v_;
  }

  /**
   * @brief Get the power spectral density of the system noise.
   * 
   * @return double 
   */
  inline double getSw() const {
    return s_w_;
  }

  /**
   * @brief Set the power spectral density of the measurement noise.
   * 
   * @param s_v power spectral density of the measurement noise.
   */
  inline void setSv(double s_v) {
    s_v_ = s_v;
    buildCv();
  }

  /**
   * @brief Set the power spectral density of the system noise.
   * 
   * @param s_w power spectral density of the system noise.
   */
  inline void setSw(double s_w) {
    s_w_ = s_w;
  }

};
}  // namespace safety_shield

#endif  // MEASUREMENT_HANDLER_H