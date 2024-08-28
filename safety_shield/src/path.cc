#include "safety_shield/path.h"

#include <cmath>

#include "spdlog/spdlog.h"

namespace safety_shield {

Path::Path() : pos_(0.0), vel_(0.0), acc_(0.0), jerk_(0.0), is_current_(false) {
  for (int i = 0; i < 3; i++) {
    times_[i] = 0;
    jerks_[i] = 0;
  }
}

void Path::increment(double duration) {
  double dt;
  for (int i = 0; i < 4; i++) {
    if (i == 4) {
      // If we exceeded the last phase, we increment the path with the remaining duration and set the jerk to zero
      jerk_ = 0.0;
      dt = duration;
    } else {
      if (times_[i] <= 0.0) {
        continue;
      }
      jerk_ = jerks_[i];
      dt = std::min(times_[i], duration);
    }
    pos_ = calculatePos(getPosition(), dt, getVelocity(), getAcceleration(), getJerk());
    vel_ = calculateVel(getVelocity(), dt, getAcceleration(), getJerk());
    acc_ = calculateAcc(getAcceleration(), dt, getJerk());
    // Decrease the time of all phases by dt
    duration -= dt;
    for (int j = i; j < 3; j++) {
      times_[j] = std::max(times_[j] - dt, 0.0);
    }
    if (duration <= 0.0) {
      break;
    }
  }
}

void Path::getFinalMotion(double& final_pos, double& final_vel, double& final_acc) {
  final_pos = getPosition();
  final_vel = getVelocity();
  final_acc = getAcceleration();
  double l_time = 0;
  for (int i = 0; i < 3; i++) {
    if (times_[i] <= 0.0) {
      continue;
    }
    double dt = (times_[i] - l_time);
    double current_jerk = jerks_[i];
    // Constant jerk motion
    final_pos = calculatePos(final_pos, dt, final_vel, final_acc, current_jerk);
    final_vel = calculateVel(final_vel, dt, final_acc, current_jerk);
    final_acc = calculateAcc(final_acc, dt, current_jerk);
    l_time = times_[i];
  }
}

void Path::getMotionUnderVel(double v_limit, double& time, double& pos, double& vel, double& acc, double& jerk) {
  if (getVelocity() < v_limit) {
    time = times_[0];
    pos = getPosition();
    vel = getVelocity();
    acc = getAcceleration();
    jerk = getJerk();
    return;
  }
  double prev_pos, next_pos = getPosition();
  double prev_vel, next_vel = getVelocity();
  double prev_acc, next_acc = getAcceleration();
  double l_time = 0;
  for (int i = 0; i < 3; i++) {
    if (times_[i] <= 0.0) {
      continue;
    }
    // we need to save previous and next values
    prev_pos = next_pos;
    prev_vel = next_vel;
    prev_acc = next_acc;

    // compute next pos, vel, acc
    double dt = (times_[i] - l_time);
    double current_jerk = jerks_[i];
    // Constant jerk motion
    next_pos = calculatePos(next_pos, dt, prev_vel, prev_acc, current_jerk);
    next_vel = calculateVel(next_vel, dt, prev_acc, current_jerk);
    next_acc = calculateAcc(next_acc, dt, current_jerk);
    l_time = times_[i];

    // if in this phase, next_vel falls below vel, recompute values but with correct time via formula
    double epsilon = 1e-8;
    if (next_vel < v_limit + epsilon) {
      double dt_limit = calculateTimeForVel(v_limit, prev_vel, prev_acc, jerks_[i]);
      assert(dt_limit >= 0);  // time must be positive
      time = times_[i] + dt_limit;
      pos = calculatePos(prev_pos, dt_limit, prev_vel, prev_acc, jerk);
      vel = calculateVel(prev_vel, dt_limit, prev_acc, jerk);
      acc = calculateAcc(prev_acc, dt_limit, jerk);
      return;
    }
  }
  // if it is not in any phase, it can't be a failsafe-path
  time = -1;
  spdlog::error("Error in Path::getMotionUnderVel: it is not in any phase");
}

double Path::getMaxVelocity() {
  double max_vel = getVelocity();
  double l_time = 0;
  double vel = getVelocity();
  double acc = getAcceleration();
  for (int i = 0; i < 3; i++) {
    if (times_[i] <= 0.0) {
      continue;
    }
    double dt = (times_[i] - l_time);
    if (acc > 0 && jerks_[i] < 0 && -acc / jerks_[i] < dt) {
      // if acc is positive and jerk is negative, we could have a maximum in the middle of this phase
      // 0 = acc + jerk * t
      // t = -acc / jerk < dt
      double mid_t = -acc / jerks_[i];
      double mid_vel = calculateVel(vel, mid_t, acc, jerks_[i]);
      max_vel = std::max(max_vel, mid_vel);
    }
    vel = calculateVel(vel, dt, acc, jerks_[i]);
    acc = calculateAcc(acc, dt, jerks_[i]);
    max_vel = std::max(max_vel, vel);
    l_time = times_[i];
  }
  return max_vel;
}
}  // namespace safety_shield
