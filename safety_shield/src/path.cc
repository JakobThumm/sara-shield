#include "safety_shield/path.h"

#include <cmath>

#include "spdlog/spdlog.h"

namespace safety_shield {

Path::Path() : pos_(0.0), vel_(0.0), acc_(0.0), jerk_(0.0), is_current_(false) {
  for (int i = 0; i < 6; i++) {
    phases_[i] = 0;
  }
}

void Path::increment(double sample_time) {
  double jerk = 0;
  double epsilon = 0.000001;
  int i = 0;

  while (i < 3) {
    if (epsilon < phases_[i]) {
      jerk = phases_[i + 3];
      i = 3;
    }
    i += 1;
  }
  jerk_ = jerk;

  if (vel_ > -epsilon) {
    pos_ +=
        vel_ * sample_time + acc_ * sample_time * sample_time / 2 + jerk * sample_time * sample_time * sample_time / 6;
  }
  vel_ += acc_ * sample_time + jerk * sample_time * sample_time / 2;
  acc_ += jerk * sample_time;

  for (i = 0; i < 3; i++) {
    phases_[i] -= sample_time;
  }
}

void Path::getFinalMotion(double& final_pos, double& final_vel, double& final_acc) {
  final_pos = pos_;
  final_vel = vel_;
  final_acc = acc_;
  double l_time = 0;
  for (int i = 0; i < 3; i++) {
    double dt = (phases_[i] - l_time);
    final_pos += final_vel * dt + final_acc * dt * dt / 2 + phases_[i + 3] * dt * dt * dt / 6;
    final_vel += final_acc * dt + phases_[i + 3] * dt * dt / 2;
    final_acc += phases_[i + 3] * dt;
    l_time = phases_[i];
  }
}

void Path::getMotionUnderVel(double v_limit, double& time, double& pos, double& vel, double& acc, double& jerk) {
  // TODO: does that make sense, it time correct?
  if (vel_ < v_limit) {
    time = phases_[0];
    pos = pos_;
    vel = vel_;
    acc = acc_;
    jerk = jerk_;
    return;
  }
  double prev_pos, next_pos = pos_;
  double prev_vel, next_vel = vel_;
  double prev_acc, next_acc = acc_;
  double l_time = 0;
  for (int i = 0; i < 3; i++) {
    // we need to save previous and next values
    prev_pos = next_pos;
    prev_vel = next_vel;
    prev_acc = next_acc;

    // compute like in getFinalMotion()
    double dt = (phases_[i] - l_time);
    next_pos += next_vel * dt + next_acc * dt * dt / 2 + phases_[i + 3] * dt * dt * dt / 6;
    next_vel += next_acc * dt + phases_[i + 3] * dt * dt / 2;
    next_acc += phases_[i + 3] * dt;
    l_time = phases_[i];

    // if in this phase, next_vel falls below vel, recompute values but with correct time via formula
    double epsilon = 1e-6;
    if (next_vel < v_limit + epsilon) {
      jerk = phases_[i + 3];
      double square = std::sqrt(prev_acc * prev_acc - 2 * jerk * (prev_vel - v_limit));
      double left = (-prev_acc - square) / jerk;
      double right = (-prev_acc + square) / jerk;
      if (left > 0 && right > 0) {
        dt = std::min(left, right);
      } else if (left > 0) {
        dt = left;
      } else if (right > 0) {
        dt = right;
      } else {
        // already under v_iso
        time = -10;
        if (std::isnan(square)) {
          spdlog::error("Error in Path::getMotionUnderVel: square is NaN");
          return;
        } else {
          spdlog::error("Error in Path::getMotionUnderVel: Quadratic-Function only has negative zero-values");
          return;
        }
      }
      time = phases_[i] + dt;
      pos = prev_vel * dt + prev_acc * dt * dt / 2 + jerk * dt * dt * dt / 6 + prev_pos;
      acc = jerk * dt + prev_acc;
      vel = prev_acc * dt + 0.5 * jerk * dt * dt + prev_vel;
      return;
    }
  }
  // if it is not in any phase, it can't be a failsafe-path
  time = -1;
  spdlog::error("Error in Path::getMotionUnderVel: it is not in any phase");
}

double Path::getMaxVelocity() {
  double max_vel = vel_;
  double l_time = 0;
  double vel = vel_;
  double acc = acc_;
  for (int i = 0; i < 3; i++) {
    double dt = (phases_[i] - l_time);
    if (acc > 0 && phases_[i + 3] < 0 && -acc / phases_[i + 3] < dt) {
      // if acc is positive and jerk is negative, we could have a maximum in the middle of this phase
      // 0 = acc + jerk * t
      // t = -acc / jerk < dt
      double mid_t = -acc / phases_[i + 3];
      double mid_vel = vel + acc * mid_t + phases_[i + 3] * mid_t * mid_t / 2;
      max_vel = std::max(max_vel, mid_vel);
    }
    vel += acc * dt + phases_[i + 3] * dt * dt / 2;
    acc += phases_[i + 3] * dt;
    max_vel = std::max(max_vel, vel);
    l_time = phases_[i];
  }
  return max_vel;
}
}  // namespace safety_shield
