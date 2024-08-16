// -*- lsst-c++ -*/
/**
 * @file kalman_filter.h
 * @brief Defines the Kalman filter class
 * @details Insipired by https://github.com/JakobThumm/OpenKF/blob/main/cpp/kalman_filter/kalman_filter.h
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

#include <stdint.h>

#include "safety_shield/safety_shield_logger.h"

#include <Eigen/Dense>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

namespace safety_shield {

using float32_t = float;
/**
 * @brief Define a matrix type.
 * 
 * @tparam ROW 
 * @tparam COL 
 */
template<size_t ROW, size_t COL>
using Matrix = Eigen::Matrix<float32_t, ROW, COL>;

/**
 * @brief Define a vector type.
 * 
 * @tparam ROW 
 */
template<size_t ROW>
using Vector = Eigen::Matrix<float32_t, ROW, 1>;

/**
 * @brief The KalmanFilter class.
 * 
 * @details This class implements a Kalman filter for LTI systems in discrete time.
 * This class is templated on the dimension of the state vector [DIM_X] 
 * and the dimension of the measurement vector [DIM_Y].
 * Both predict and update functions take all matrices as input to allow for varying time intervals.
 * 
 * @tparam DIM_X State vector dimension
 * @tparam DIM_Y Measurement vector dimension
 */
template<size_t DIM_X, size_t DIM_Y>
class KalmanFilter {
 public:
  /**
   * @brief Construct a new Kalman Filter object
   */
  KalmanFilter() {}

  /**
   * @brief Destroy the Kalman Filter object
   */
  ~KalmanFilter() {}

  /**
   * @brief Get the state vector.
   * 
   * @return Vector<DIM_X> 
   */
  inline Vector<DIM_X> getState() const { 
    return x_; 
  }

  /**
   * @brief Get the covariance matrix.
   * 
   * @return Matrix<DIM_X, DIM_X> 
   */
  inline Matrix<DIM_X, DIM_X> getCovariance() const {
    return C_; 
  }

  /**
   * @brief Reset the Kalman filter.
   * 
   * @param initial_state Initial state estimate.
   * @param initial_covariance Covariance of the initial state estimate.
   */
  inline void reset(Vector<DIM_X> initial_state, Matrix<DIM_X, DIM_X> initial_covariance) {
    initialized_ = true;
    x_ = initial_state;
    C_ = initial_covariance;
  }

  /**
   * @brief Prediction step of the Kalman filter with a linear process model.
   * @details x_{k+1} = A * x_k
   *          C_{k+1} = A * C_k * A^T + C_w
   * 
   * @param A The state transition matrix.
   * @param C_w The process noise covariance matrix.
   * @throws std::runtime_error if the Kalman filter was not initialized.
   */
  void predict(const Matrix<DIM_X, DIM_X>& A, const Matrix<DIM_X, DIM_X>& C_w);

  /**
   * @brief The update step of the Kalman filter with a linear measurement model.
   * @details x_{k+1} = x_k + K * (y - H * x_k)
   *          C_{k+1} = (I - K * H) * C_k
   *          with K = C_k * H^T * (H * C_k * H^T + C_v)^{-1}
   * 
   * @param y The measurement vector.
   * @param C_v The measurement noise covariance matrix.
   * @param H The measurement transition matrix (measurement model).
   * @throws std::runtime_error if the Kalman filter was not initialized.
   */
  void update(const Vector<DIM_Y>& y, const Matrix<DIM_Y, DIM_Y>& C_v, const Matrix<DIM_Y, DIM_X>& H);

protected:

  /**
   * @brief Indicates if the reset function was called once.
   */
  bool initialized_ = false;

  /**
   * @brief The state vector.
   * @details Depending on the last function call (update or predict) this vector contains the predicted or updated state.
   */
  Vector<DIM_X> x_{ Vector<DIM_X>::Zero() };

  /**
   * @brief The state covariance matrix.
   * @details Depending on the last function call (update or predict) this matrix contains the predicted or updated state covariance.
   */
  Matrix<DIM_X, DIM_X> C_{ Matrix<DIM_X, DIM_X>::Zero() };

  /**
   * @brief Identity matrix.
   */
  Matrix<DIM_X, DIM_X> I_{ Matrix<DIM_X, DIM_X>::Identity() };
};
} // namespace safety_shield

#endif // KALMAN_FILTER_H