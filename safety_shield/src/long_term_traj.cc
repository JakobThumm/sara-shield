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
) : long_term_traj_(long_term_traj), sample_time_(sample_time), current_pos_(0), starting_index_(starting_index),
      v_max_allowed_(v_max_allowed), a_max_allowed_(a_max_allowed), j_max_allowed_(j_max_allowed) {
  length_ = long_term_traj.size();
  calculate_max_acc_jerk_window(long_term_traj_, sliding_window_k);
  // Initialize alpha_i_ with 0
  for (int i = 0; i < long_term_traj_[0].getNbModules(); i++) {
    alpha_i_.push_back(0.0);
  }
  max_cart_vel_ = 0;
  std::vector<RobotReach::CapsuleVelocity> previous_capsule_velocities;
  // iterate through each motion
  for (int i = 0; i < getLength(); i++) {
    Motion& motion = long_term_traj_[i];
    double motion_vel = 0;
    robot_reach.calculateAllTransformationMatricesAndCapsules(motion.getAngleRef());
    for (int j = 0; j < long_term_traj_[i].getNbModules(); j++) {
      RobotReach::CapsuleVelocity capsule_velocity = robot_reach.getVelocityOfCapsule(j, motion.getVelocityRef());
      motion_vel = robot_reach.approximateVelOfCapsule(j, capsule_velocity.second.first, capsule_velocity.second.second);
      if (i > 0) {
        double dt = motion.getTime() - long_term_traj_[i-1].getTime();
        double alpha_1 = (std::abs(capsule_velocity.first.first.norm() - previous_capsule_velocities[j].first.first.norm())) / dt;
        double alpha_2 = (std::abs(capsule_velocity.second.first.norm() - previous_capsule_velocities[j].second.first.norm())) / dt;
        alpha_i_[j] = std::max(alpha_i_[j], std::max(alpha_1, alpha_2));
        previous_capsule_velocities[j] = capsule_velocity;
      } else {
        previous_capsule_velocities.push_back(capsule_velocity);
      }
    }
    // Max velocity of this motion
    motion.setMaximumCartesianVelocity(motion_vel);
    // Max velocity of the entire LTT.
    max_cart_vel_ = std::max(max_cart_vel_, motion_vel);
  }
}

Motion LongTermTraj::interpolate(double s, double ds, double dds, double ddds) const {
  // Example: s=2.465, sample_time = 0.004 --> ind = 616.25
  assert(sample_time_ != 0);
  double ind = s / sample_time_;
  double intpart;
  // Example: intpart = 616.0, ind_mod = 0.25
  double ind_mod = modf(ind, &intpart);
  // floor(s/sample_time) + 1 ==> lower index
  int ind1 = static_cast<int>(intpart);
  // ceil(s/sample_time) + 1 ==> upper index
  int ind2 = static_cast<int>(ceil(ind));
  // time from first index to interpolation point
  double dt = ind_mod * sample_time_;
  std::vector<double> q1 = getNextMotionAtIndex(ind1).getAngle();
  std::vector<double> dq1 = getNextMotionAtIndex(ind1).getVelocity();
  std::vector<double> ddq1 = getNextMotionAtIndex(ind1).getAcceleration();
  std::vector<double> dddq1 = getNextMotionAtIndex(ind1).getJerk();
  std::vector<double> q(q1.size());
  std::vector<double> dq(q1.size());
  std::vector<double> ddq(q1.size());
  std::vector<double> dddq(q1.size());
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
  }
  return Motion(0.0, q, dq, ddq, dddq, s);
}

void LongTermTraj::calculate_max_acc_jerk_window(std::vector<Motion>& long_term_traj, int k) {
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
  // iterate through each motion
  for (int i = 0; i < getLength(); i++) {
    Motion& motion = long_term_traj_[i];
    double motion_vel = robot_reach.maxVelocityOfMotion(motion);
    motion.setMaximumCartesianVelocity(motion_vel);
    // save maximum of whole LTT for TRIVIAL_CARTESIAN
    max_cart_vel_ = std::max(motion_vel, max_cart_vel_);
  }
}

}  // namespace safety_shield