#include "safety_shield/kalman_filter.h"

namespace safety_shield {

template <size_t DIM_X, size_t DIM_Y>
void KalmanFilter<DIM_X, DIM_Y>::predict(const Matrix<DIM_X, DIM_X> & A, const Matrix<DIM_X, DIM_X> & C_w) {
  x_ = A * x_;
  C_ = A * C_ * A.transpose() + C_w;
}

template <size_t DIM_X, size_t DIM_Y>
void KalmanFilter<DIM_X, DIM_Y>::update(const Vector<DIM_Y>& y, const Matrix<DIM_Y, DIM_Y>& C_v, const Matrix<DIM_Y, DIM_X>& H) {
  const Matrix<DIM_Y, DIM_Y> matSk{ H * C_ * H.transpose() + C_v }; // Innovation covariance
  const Matrix<DIM_X, DIM_Y> K{ C_ * H.transpose() * matSk.inverse() }; // Kalman Gain
  x_ = x_ + K * (y - (H * x_));
  C_ = (I_ - K * H) * C_;
}

// Explicit template instantiation
template class KalmanFilter<4, 4>;
template class KalmanFilter<6, 6>;
}  // namespace safety_shield