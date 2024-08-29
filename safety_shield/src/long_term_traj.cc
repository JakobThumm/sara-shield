#include "safety_shield/long_term_traj.h"

namespace safety_shield {

LongTermTraj::LongTermTraj(
  const std::vector<Motion>& long_term_traj,
  double sample_time,
  RobotReach& robot_reach,
  int starting_index,
  std::vector<double> v_max_allowed,
  std::vector<double> a_max_allowed,
  std::vector<double> j_max_allowed,
  int sliding_window_k
) : current_pos_(0), starting_index_(starting_index),
      v_max_allowed_(v_max_allowed), a_max_allowed_(a_max_allowed), j_max_allowed_(j_max_allowed) {
  setLongTermTrajectory(long_term_traj, sample_time);
  calculateMaxAccJerkWindow(long_term_traj_, sliding_window_k);
  velocitiesOfAllMotions(robot_reach);
  calculateAlphaBeta();
}

Motion LongTermTraj::interpolate(double s, double ds, double dds, double ddds) const {
  // Example: s=2.465, sample_time = 0.004 --> ind = 616.25
  int ind1 = getLowerIndex(s);
  double dt = getModIndex(s) * sample_time_;
  std::vector<double> q1 = getNextMotionAtIndex(ind1).getAngle();
  std::vector<double> dq1 = getNextMotionAtIndex(ind1).getVelocity();
  std::vector<double> ddq1 = getNextMotionAtIndex(ind1).getAcceleration();
  std::vector<double> dddq1 = getNextMotionAtIndex(ind1).getJerk();
  std::vector<double> q(q1.size());
  std::vector<double> dq(q1.size());
  std::vector<double> ddq(q1.size());
  std::vector<double> dddq(q1.size());
  std::vector<double> cartesian_velocities = getNextMotionAtIndex(ind1).getMaximumCartesianVelocities();
  for (int i = 0; i < q1.size(); i++) {
    // Linearly interpolate between lower and upper index of position
    q[i] = q1[i] + dt * dq1[i] + 1.0 / 2 * dt * dt * ddq1[i] + 1.0 / 6 * dt * dt * dt * dddq1[i];
    // Calculate LTT velocity
    double v_max_int = dq1[i] + dt * ddq1[i] + 1.0 / 2 * dt * dt * dddq1[i];
    double v_int = v_max_int * ds;
    dq[i] = std::clamp(v_int, -v_max_allowed_[i], v_max_allowed_[i]);
    // Calculate Acceleration
    double a_max_int = ddq1[i] + dt * dddq1[i];
    double a_int = v_max_int * dds + ds * ds * a_max_int;
    ddq[i] = std::clamp(a_int, -a_max_allowed_[i], a_max_allowed_[i]);
    dddq[i] = std::clamp(dddq1[i] * ds * ds * ds + 3.0 * a_max_int * dds * ds + v_max_int * ddds, -j_max_allowed_[i], j_max_allowed_[i]);
    // Only update the cartesian velocities if they are available
    if (cartesian_velocities.size() == q1.size()) {
      cartesian_velocities[i] *= ds;
    }
  }
  Motion m = Motion(0.0, q, dq, ddq, dddq, s);
  if (cartesian_velocities.size() == q1.size()) {
    m.setMaximumCartesianVelocities(cartesian_velocities);
    m.setMaximumCartesianVelocity(getNextMotionAtIndex(ind1).getMaximumCartesianVelocity() * ds);
  }
  return m;
}

void LongTermTraj::calculateMaxAccJerkWindow(std::vector<Motion>& long_term_traj, int k) {
  int traj_length = long_term_traj.size();
  // It must be k <= trajectory length.
  k = std::min(traj_length, k);
  int n_joints = long_term_traj[0].getAngle().size();
  max_acceleration_window_.reserve(traj_length);
  max_jerk_window_.reserve(traj_length);

  // We use method 3 of https://www.geeksforgeeks.org/sliding-window-maximum-maximum-of-all-subarrays-of-size-k/
  std::vector<std::deque<int>> max_queue_acc;
  std::vector<std::deque<int>> max_queue_jerk;
  for (int j = 0; j < n_joints; ++j) {
    max_queue_acc.push_back(std::deque<int>());
    max_queue_jerk.push_back(std::deque<int>());
  }
  /* Process first k (or first window)
    elements of array */
  int i;
  for (i = 0; i < k; ++i) {
    for (int j = 0; j < n_joints; ++j) {
      // ACCELERATION
      // For every element, the previous smaller elements are useless so remove them from queue
      while (!max_queue_acc[j].empty() &&
             long_term_traj[i].getAcceleration()[j] >= long_term_traj[max_queue_acc[j].back()].getAcceleration()[j]) {
        // Remove from rear
        max_queue_acc[j].pop_back();
      }
      // Add new element at rear of queue
      max_queue_acc[j].push_back(i);

      // JERK
      while ((!max_queue_jerk[j].empty()) &&
             long_term_traj[i].getJerk()[j] >= long_term_traj[max_queue_jerk[j].back()].getJerk()[j]) {
        max_queue_jerk[j].pop_back();
      }
      max_queue_jerk[j].push_back(i);
    }
  }
  // Process rest of the elements,
  // i.e., from arr[k] to arr[n-1]
  for (; i < traj_length + k; ++i) {
    std::vector<double> max_acc;
    std::vector<double> max_jerk;
    max_acc.reserve(n_joints);
    max_jerk.reserve(n_joints);
    for (int j = 0; j < n_joints; ++j) {
      // ACCELERATION
      // The element at the front of the queue is the largest element of previous window
      max_acc.push_back(long_term_traj[max_queue_acc[j].front()].getAcceleration()[j]);
      // Remove the elements which are out of this window
      while ((!max_queue_acc[j].empty()) && max_queue_acc[j].front() <= i - k) {
        // Remove from front of queue
        max_queue_acc[j].pop_front();
      }
      if (i < traj_length) {
        // Remove all elements smaller than the currently being added element (remove useless elements)
        while ((!max_queue_acc[j].empty()) &&
               long_term_traj[i].getAcceleration()[j] >= long_term_traj[max_queue_acc[j].back()].getAcceleration()[j]) {
          max_queue_acc[j].pop_back();
        }
        // Add current element at the rear of Qi
        max_queue_acc[j].push_back(i);
      }

      // JERK
      max_jerk.push_back(long_term_traj[max_queue_jerk[j].front()].getJerk()[j]);
      while ((!max_queue_jerk[j].empty()) && max_queue_jerk[j].front() <= i - k) {
        max_queue_jerk[j].pop_front();
      }
      if (i < traj_length) {
        while ((!max_queue_jerk[j].empty()) &&
               long_term_traj[i].getJerk()[j] >= long_term_traj[max_queue_jerk[j].back()].getJerk()[j]) {
          max_queue_jerk[j].pop_back();
        }
        max_queue_jerk[j].push_back(i);
      }
    }
    max_acceleration_window_.push_back(max_acc);
    max_jerk_window_.push_back(max_jerk);
  }
}

double LongTermTraj::getMaxofMaximumCartesianVelocity() const {
  return max_cart_vel_;
}

double LongTermTraj::getMaxofMaximumCartesianVelocityWithS(double s) const {
  unsigned long i = getCurrentPos();
  double max = getMotion(i).getMaximumCartesianVelocity();
  while (i < length_) {
    max = std::max(max, getMotion(i).getMaximumCartesianVelocity());
    if (getMotion(i).getTime() > s) {
      return max;
    }
    ++i;
  }
  return max;
}

void LongTermTraj::velocitiesOfAllMotions(RobotReach& robot_reach) {
  max_cart_vel_ = 0;
  capsule_velocities_.resize(length_);
  // iterate through each motion
  for (int i = 0; i < length_; i++) {
    capsule_velocities_[i].resize(nb_modules_);
    Motion& motion = long_term_traj_[i];
    double max_motion_vel = 0;
    std::vector<double> motion_vels;
    robot_reach.calculateAllTransformationMatricesAndCapsules(motion.getAngleRef());
    for (int j = 0; j < nb_modules_; j++) {
      capsule_velocities_[i][j] = robot_reach.calculateVelocityOfCapsule(j, motion.getVelocityRef());
      double motion_vel = robot_reach.approximateVelOfCapsule(j, capsule_velocities_[i][j].v2.v, capsule_velocities_[i][j].v2.w);
      motion_vels.push_back(motion_vel);
      max_motion_vel = std::max(max_motion_vel, motion_vel);
    }
    // Max velocity of this motion
    motion.setMaximumCartesianVelocity(max_motion_vel);
    motion.setMaximumCartesianVelocities(motion_vels);
    // Max velocity of the entire LTT.
    max_cart_vel_ = std::max(max_cart_vel_, max_motion_vel);
    // Calculate the link inertia matrices
    inertia_matrices_.push_back(robot_reach.calculateAllInertiaMatrices());
  }
}

void LongTermTraj::calculateAlphaBeta() {
  for (int i = 0; i < nb_modules_; i++) {
    alpha_i_.push_back(0.0);
    beta_i_.push_back(0.0);
  }
  capsule_velocities_.resize(length_);
  // iterate through each motion
  for (int i = 1; i < length_; i++) {
    for (int j = 0; j < nb_modules_; j++) {
        double dt = long_term_traj_[i].getTime() - long_term_traj_[i-1].getTime();
        double alpha_1 = (std::abs(capsule_velocities_[i][j].v1.v.norm() - capsule_velocities_[i-1][j].v1.v.norm())) / dt;
        double alpha_2 = (std::abs(capsule_velocities_[i][j].v2.v.norm() - capsule_velocities_[i-1][j].v2.v.norm())) / dt;
        double beta_1 = (std::abs(capsule_velocities_[i][j].v1.w.norm() - capsule_velocities_[i-1][j].v1.w.norm())) / dt;
        double beta_2 = (std::abs(capsule_velocities_[i][j].v2.w.norm() - capsule_velocities_[i-1][j].v2.w.norm())) / dt;
        alpha_i_[j] = std::max(alpha_i_[j], std::max(alpha_1, alpha_2));
        beta_i_[j] = std::max(beta_i_[j], std::max(beta_1, beta_2));
    }
  }
}

}  // namespace safety_shield