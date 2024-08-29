#include "safety_shield/measurement_handler.h"

namespace safety_shield {

void MeasurementHandler::initialMeasurement(const std::vector<reach_lib::Point>& measurements) {
  if (measurements.size() != n_joints_meas_) {
    throw std::length_error("MeasurementHandler::initialMeasurement: measurements vector must have the same size as n_joints_meas_");
  }
  for (int i = 0; i < n_joints_meas_; i++) {
    Vector<6> initial_state;
    initial_state << measurements[i].x, 0.0, measurements[i].y, 0.0, measurements[i].z, 0.0;
    Matrix<6, 6> initial_cov;
    initial_cov << initial_pos_var_, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, initial_vel_var_, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, initial_pos_var_, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, initial_vel_var_, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, initial_pos_var_, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, initial_vel_var_;
    kalman_filters_[i].reset(initial_state, initial_cov);
  }
}

Observation MeasurementHandler::filterMeasurements(
  const std::vector<reach_lib::Point>& measurements, double time) {
  if (measurements.size() != n_joints_meas_) {
    throw std::length_error("MeasurementHandler::filterMeasurements: measurements vector must have the same size as n_joints_meas_");
  }
  if (time < 0.0) {
    throw std::invalid_argument("MeasurementHandler::filterMeasurements: time must be greater than or equal to 0.0");
  }
  Observation filtered_measurements;
  filtered_measurements.time = time;
  if (last_meas_timestep_ == -1) {
    // First measurement
    initialMeasurement(measurements);
    last_meas_timestep_ = time;
    filtered_measurements.position = measurements;
    filtered_measurements.velocity = std::vector<reach_lib::Point>(n_joints_meas_, reach_lib::Point(0.0, 0.0, 0.0));
    filtered_measurements.pos_variance = std::vector<double>(n_joints_meas_, initial_pos_var_);
    filtered_measurements.vel_variance = std::vector<double>(n_joints_meas_, initial_vel_var_);
    return filtered_measurements;
  }
  double dt = time - last_meas_timestep_;
  buildA(dt);
  buildCw(dt);
  filtered_measurements.position.resize(n_joints_meas_);
  filtered_measurements.velocity.resize(n_joints_meas_);
  filtered_measurements.pos_variance.resize(n_joints_meas_);
  filtered_measurements.vel_variance.resize(n_joints_meas_);
  for (int i = 0; i < n_joints_meas_; i++) {
    kalman_filters_[i].predict(A_, C_w_);
    Vector<3> measurement;
    measurement << measurements[i].x, measurements[i].y, measurements[i].z;
    kalman_filters_[i].update(measurement, C_v_, H_);
    Vector<6> state = kalman_filters_[i].getState();
    filtered_measurements.position[i] = reach_lib::Point(state(0), state(2), state(4));
    filtered_measurements.velocity[i] = reach_lib::Point(state(1), state(3), state(5));
    Matrix<6, 6> covariance = kalman_filters_[i].getCovariance();
    filtered_measurements.pos_variance[i] = 1.0/3.0 * (covariance(0, 0) + covariance(2, 2) + covariance(4, 4));
    filtered_measurements.vel_variance[i] = 1.0/3.0 * (covariance(1, 1) + covariance(3, 3) + covariance(5, 5));
  }
  return filtered_measurements;
}

} // namespace safety_shield