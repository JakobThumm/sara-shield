#include "safety_shield/safety_shield.h"

namespace safety_shield {

SafetyShield::SafetyShield()
    : max_s_stop_(0),
      v_max_allowed_({0, 0, 0}),
      a_max_allowed_({0, 0, 0}),
      j_max_allowed_({0, 0, 0}),
      a_max_ltt_({0, 0, 0}),
      j_max_ltt_({0, 0, 0}) {
  robot_reach_ = new RobotReach();
  human_reach_ = new HumanReach();
  verify_ = new VerifyISO();
  spdlog::info("Safety shield created.");
}

SafetyShield::SafetyShield(int nb_joints, double sample_time, double max_s_stop,
                           const std::vector<double>& v_max_allowed, const std::vector<double>& a_max_allowed,
                           const std::vector<double>& j_max_allowed, const std::vector<double>& a_max_path,
                           const std::vector<double>& j_max_path, const LongTermTraj& long_term_trajectory,
                           RobotReach* robot_reach, HumanReach* human_reach,
                           VerifyISO* verify, const std::vector<reach_lib::AABB> &environment_elements, ShieldType shield_type,
                           ContactType eef_contact_type)
    : nb_joints_(nb_joints),
      max_s_stop_(max_s_stop),
      v_max_allowed_(v_max_allowed),
      a_max_allowed_(a_max_allowed),
      j_max_allowed_(j_max_allowed),
      a_max_ltt_(a_max_path),
      j_max_ltt_(j_max_path),
      sample_time_(sample_time),
      path_s_(0),
      path_s_discrete_(0),
      long_term_trajectory_(long_term_trajectory),
      robot_reach_(robot_reach),
      human_reach_(human_reach),
      verify_(verify),
      environment_elements_(environment_elements),
      shield_type_(shield_type),
      eef_contact_type_(eef_contact_type) {
  sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
  std::vector<double> prev_dq;
  for (int i = 0; i < nb_joints_; i++) {
    prev_dq.push_back(0.0);
  }

  if (shield_type_ == ShieldType::OFF) {
    is_safe_ = true;
  } else {
    is_safe_ = false;
  }
  Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
  next_motion_ = determineNextMotion(is_safe_);
  std::vector<double> q_min(nb_joints, -3.141);
  std::vector<double> q_max(nb_joints, -3.141);
  ltp_ = long_term_planner::LongTermPlanner(nb_joints, sample_time, q_min, q_max, v_max_allowed, a_max_path, j_max_path);
  reachability_set_duration_ = reachability_set_interval_size_ * sample_time_;
  buildMaxContactEnergies();
  spdlog::info("Safety shield created.");
}

SafetyShield::SafetyShield(double sample_time, std::string trajectory_config_file, std::string robot_config_file,
                           std::string mocap_config_file, double init_x, double init_y, double init_z, double init_roll,
                           double init_pitch, double init_yaw, const std::vector<double>& init_qpos,
                           const std::vector<reach_lib::AABB> &environment_elements, ShieldType shield_type, ContactType eef_contact_type)
    : sample_time_(sample_time), path_s_(0), path_s_discrete_(0), environment_elements_(environment_elements),
     shield_type_(shield_type), eef_contact_type_(eef_contact_type) {
  ///////////// Build robot reach
  robot_reach_ = buildRobotReach(robot_config_file, init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
  nb_joints_ = robot_reach_->getNbJoints();
  ////////////// Setting trajectory variables
  int velocity_method_int;
  readTrajectoryConfig(
    trajectory_config_file,
    max_s_stop_,
    q_min_allowed_,
    q_max_allowed_,
    v_max_allowed_,
    a_max_allowed_,
    j_max_allowed_,
    a_max_ltt_,
    j_max_ltt_,
    v_safe_,
    alpha_i_max_,
    velocity_method_int,
    reachability_set_interval_size_
  );
  ltp_ = long_term_planner::LongTermPlanner(nb_joints_, sample_time, q_min_allowed_, q_max_allowed_, v_max_allowed_,
                                            a_max_ltt_, j_max_ltt_);
  RobotReach::VelocityMethod velocity_method = static_cast<RobotReach::VelocityMethod>(velocity_method_int);
  robot_reach_->setVelocityMethod(velocity_method);
  // Initialize the long term trajectory
  sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
  std::vector<Motion> long_term_traj;
  long_term_traj.push_back(Motion(0.0, init_qpos));
  if (shield_type_ == ShieldType::SSM || shield_type_ == ShieldType::OFF) {
    long_term_trajectory_ = LongTermTraj(
      long_term_traj, sample_time_, path_s_discrete_,
      v_max_allowed_, a_max_allowed_, j_max_allowed_,
      sliding_window_k_, alpha_i_max_
    );
  } else {
    long_term_trajectory_ = LongTermTraj(
      long_term_traj, sample_time_, *robot_reach_, path_s_discrete_,
      v_max_allowed_, a_max_allowed_, j_max_allowed_, sliding_window_k_
    );
  }
  reachability_set_duration_ = reachability_set_interval_size_ * sample_time_;
  //////////// Build human reach
  human_reach_ = buildHumanReach(mocap_config_file);
  buildMaxContactEnergies();
  ///////////// Build verifier
  verify_ = new safety_shield::VerifyISO();
  /////////// Other settings
  sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
  std::vector<double> prev_dq;

  for (int i = 0; i < nb_joints_; i++) {
    prev_dq.push_back(0.0);
  }
  if (shield_type_ == ShieldType::OFF) {
    is_safe_ = true;
  } else {
    is_safe_ = false;
  }
  Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
  next_motion_ = determineNextMotion(is_safe_);
  spdlog::info("Safety shield created.");
}

void SafetyShield::reset(double init_x, double init_y, double init_z, double init_roll, double init_pitch,
                         double init_yaw, const std::vector<double>& init_qpos, double current_time,
                         const std::vector<reach_lib::AABB> &environment_elements, ShieldType shield_type,
                         ContactType eef_contact_type) {
  shield_type_ = shield_type;
  robot_reach_->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
  human_reach_->reset();
  eef_contact_type_ = eef_contact_type;
  buildMaxContactEnergies();
  environment_elements_ = environment_elements;
  std::vector<double> prev_dq;
  for (int i = 0; i < nb_joints_; i++) {
    prev_dq.push_back(0.0);
  }
  if (shield_type_ == ShieldType::OFF) {
    is_safe_ = true;
  } else {
    is_safe_ = false;
  }
  new_ltt_ = false;
  new_goal_ = false;
  new_ltt_processed_ = false;
  recovery_path_correct_ = false;
  path_s_ = 0;
  path_s_discrete_ = 0;
  cycle_begin_time_ = current_time;
  recovery_path_ = Path();
  failsafe_path_ = Path();
  failsafe_path_2_ = Path();
  safe_path_ = Path();
  // Initialize the long term trajectory
  std::vector<Motion> long_term_traj;
  long_term_traj.push_back(Motion(0.0, init_qpos));
  if (shield_type_ == ShieldType::SSM || shield_type_ == ShieldType::OFF) {
    long_term_trajectory_ = LongTermTraj(
      long_term_traj, sample_time_, path_s_discrete_,
      v_max_allowed_, a_max_allowed_, j_max_allowed_,
      sliding_window_k_, alpha_i_max_
    );
  } else {
    long_term_trajectory_ = LongTermTraj(
      long_term_traj, sample_time_, *robot_reach_, path_s_discrete_,
      v_max_allowed_, a_max_allowed_, j_max_allowed_, sliding_window_k_
    );
  }
  Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
  next_motion_ = determineNextMotion(is_safe_);
  spdlog::info("Safety shield resetted.");
}

bool SafetyShield::planSafetyShield(double pos, double vel, double acc, double ve, double a_max, double j_max,
                                    Path& path) {
  if (a_max < 0 || fabs(acc) > a_max) {
    return false;
  }
  try {
    path.setCurrent(false);
    path.setPosition(pos);
    path.setVelocity(vel);
    path.setAcceleration(acc);
    double epsilon = 1e-6;

    // If current velocity is close to goal and acceleration is approx zero -> Do nothing
    if (fabs(vel - ve) < epsilon && fabs(acc) < epsilon) {
      std::array<double, 3> new_times = {sample_time_, sample_time_, sample_time_};
      std::array<double, 3> new_jerks = {0.0, 0.0, 0.0};
      path.setPhases(new_times, new_jerks);
      return true;
    }
    // Time to reach zero acceleration with maximum jerk
    double t_to_a_0 = roundToTimestep(abs(acc) / j_max);
    // Velocity at zero acceleration
    double v_at_a_0 = vel + acc * t_to_a_0 / 2;
    if (v_at_a_0 < 0) {
      spdlog::warn("Velocity at zero acceleration is negative. The path calculations will be incorrect.");
    }

    // If the velocity at zero acceleration is the goal velocity, use t_to_a_0 and max jerk and return
    if (fabs(v_at_a_0 - ve) < epsilon) {
      std::array<double, 3> new_times = {t_to_a_0, t_to_a_0, t_to_a_0};
      std::array<double, 3> new_jerks = {-acc / t_to_a_0, 0, 0};
      path.setPhases(new_times, new_jerks);
      return true;
    }

    double a_peak, t01, t12, t23, v01, v12, v23, correct_int;
    // If we are accelerating
    if (acc > 0) {
      // Reducing the acceleration to zero with max jerk would lead to a velocity larger than target
      if (v_at_a_0 > ve + epsilon) {
        if (vel - (a_max * a_max + acc * acc / 2) / j_max > ve) {
          a_peak = -a_max;
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
          v12 = a_peak * t12;
        } else {
          a_peak = -sqrt(fabs((vel - ve) * j_max + acc * acc / 2));
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      } else {
        if (vel + (a_max * a_max - acc * acc / 2) / j_max < ve) {
          a_peak = a_max;
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
          v12 = a_peak * t12;
        } else {
          a_peak = sqrt(fabs((ve - vel) * j_max + acc * acc / 2));
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      }
    }
    // If we are decelerating (a <= 0)
    else {
      if (v_at_a_0 > ve + epsilon) {
        if (vel - (a_max * a_max - acc * acc / 2) / j_max > ve) {
          a_peak = -a_max;
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / (a_peak));
          v12 = a_peak * t12;
        } else {
          a_peak = -sqrt(fabs((vel - ve) * j_max + acc * acc / 2));
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      } else {
        if (vel + (a_max * a_max + acc * acc / 2) / j_max < ve) {
          a_peak = a_max;
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
          v12 = a_peak * t12;
        } else {
          a_peak = sqrt(fabs((ve - vel) * j_max + acc * acc / 2));
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      }
    }
    correct_int = ve - vel - v01 - v12 - v23;
    a_peak += correct_int / ((t01 + t23) / 2 + t12 + epsilon);
    double j01;
    if (t01 >= epsilon) {
      j01 = (a_peak - acc) / (t01);
    } else {
      j01 = 0;
    }
    double j12 = 0;
    double j23;
    if (t23 >= epsilon) {
      j23 = -a_peak / (t23);
    } else {
      j23 = 0;
    }
    if (t01 < 0 || t12 < 0 || t23 < 0) {
      spdlog::debug("planSafetyShield calculated time negative. t01 = {}, t12 = {}, t23 = {}", t01, t12, t23);
      return false;
    }
    std::array<double, 3> new_times = {t01, t01 + t12, t01 + t12 + t23};
    std::array<double, 3> new_jerks = {j01, j12, j23};
    path.setPhases(new_times, new_jerks);
    return true;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::planSafetyShield: {}", exc.what());
    return false;
  }
}

void SafetyShield::calculateMaxAccJerk(const std::vector<double>& prev_speed, const std::vector<double>& a_max_part,
                                       const std::vector<double>& j_max_part, double& a_max_manoeuvre,
                                       double& j_max_manoeuvre) {
  double new_c, new_d;
  // Calculate dds_max
  double denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
  double min_c = (a_max_allowed_[0] - std::abs(a_max_part[0])) / denom;
  for (int i = 1; i < a_max_allowed_.size(); i++) {
    denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
    new_c = (a_max_allowed_[i] - std::abs(a_max_part[i])) / denom;
    min_c = (new_c < min_c) ? new_c : min_c;
  }
  a_max_manoeuvre = (min_c < 0) ? 0 : min_c;
  // Calculate ddds_max
  denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
  double min_d = (j_max_allowed_[0] - 3 * std::abs(a_max_part[0]) * a_max_manoeuvre - std::abs(j_max_part[0])) / denom;
  for (int i = 1; i < a_max_allowed_.size(); i++) {
    denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
    new_d = (j_max_allowed_[i] - 3 * std::abs(a_max_part[i]) * a_max_manoeuvre - std::abs(j_max_part[i])) / denom;
    min_d = (new_d < min_d) ? new_d : min_d;
  }
  j_max_manoeuvre = (min_d < 0) ? 0 : min_d;
}

Motion SafetyShield::computesPotentialTrajectory(bool v, const std::vector<double>& prev_speed) {
  try {
    // s_int indicates the index of the entire traveled way
    while (path_s_ >= (path_s_discrete_ + 1) * sample_time_) {
      long_term_trajectory_.increasePosition();
      if (new_ltt_) {
        new_long_term_trajectory_.increasePosition();
      }
      path_s_discrete_++;
    }
    // If verified safe, take the recovery path, otherwise, take the failsafe path
    if (v && recovery_path_correct_) {
      recovery_path_.setCurrent(true);
      // discard old FailsafePath and replace with new one
      failsafe_path_ = failsafe_path_2_;
      // repair path already incremented
    } else {
      failsafe_path_.setCurrent(true);
      // discard RepairPath
      recovery_path_.setCurrent(false);
      failsafe_path_.increment(sample_time_);
    }

    // find maximum acceleration and jerk authorised
    double a_max_manoeuvre, j_max_manoeuvre;
    // One could use new_long_term_trajectory_.getMaxAccelerationWindow(path_s_discrete_) instead of a_max_ltt but there
    // are many bugs to be solved before.
    if (!new_ltt_) {
      calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_, a_max_manoeuvre, j_max_manoeuvre);
    } else {
      calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_, a_max_manoeuvre, j_max_manoeuvre);
    }

    // Desired movement, one timestep
    //  if not already on the repair path, plan a repair path
    if (!recovery_path_.isCurrent()) {
      // plan repair path and replace
      recovery_path_correct_ =
          planSafetyShield(failsafe_path_.getPosition(), failsafe_path_.getVelocity(), failsafe_path_.getAcceleration(),
                           1, a_max_manoeuvre, j_max_manoeuvre, recovery_path_);
    }
    // Only plan new failsafe trajectory if the recovery path planning was successful.
    if (recovery_path_correct_) {
      // advance one step on repair path
      recovery_path_.increment(sample_time_);
      bool failsafe_2_planning_success;
      failsafe_2_planning_success =
          planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(),
                          0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
      // Check the validity of the planned path
      if (!failsafe_2_planning_success || recovery_path_.getPosition() < failsafe_path_.getPosition()) {
        recovery_path_correct_ = false;
      }
    }
    // If all planning was correct, use new failsafe path with single recovery step
    if (recovery_path_correct_) {
      potential_path_ = failsafe_path_2_;
    } else {
      // If planning failed, use previous failsafe path
      potential_path_ = failsafe_path_;
    }

    //// Calculate start and goal pos of intended motion
    double s_d, ds_d, dds_d;
    potential_path_.getFinalMotion(s_d, ds_d, dds_d);
    double ddds_d = 0.0;
    // Calculate goal
    Motion goal_motion;
    if (new_ltt_) {
      goal_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d);
    } else {
      goal_motion = long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d);
    } 
    goal_motion.setTime(potential_path_.getTime(2));
    return goal_motion;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::computesPotentialTrajectory: {}", exc.what());
    throw exc;
    return Motion();
  }
}

bool SafetyShield::planPFLFailsafe(double a_max_manoeuvre, double j_max_manoeuvre) {
  // Calculate maximal Carthesian velocity in the short-term plan
  double s_d, ds_d, dds_d;
  // Calculate goal
  bool is_under_iso_velocity = false;
  // v_max is maximum of LTT or STP and v_limit is how much path velocity needs to be scaled to be under v_iso
  // First, plan a failsafe path that ends in a complete stop. This is the longest failsafe path possible.
  bool planning_success = planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(), 0.0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
  if (!planning_success) {
    return false;
  }
  failsafe_path_2_.getFinalMotion(s_d, ds_d, dds_d);
  double v_max;
  if(!new_ltt_) {
    v_max = long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  } else {
    v_max = new_long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  }
  // Check if the entire long-term trajectory is under iso-velocity
  is_under_iso_velocity = v_max <= v_safe_;
  double v_limit;
  // TODO: if the current velocity is under iso-velocity, we dont need to compute the failsafe-path of the PFL-criterion
  if(!is_under_iso_velocity) {
    v_limit = v_safe_ / v_max;
    if(v_limit > 1 || v_limit < 0) {
      spdlog::error("v_limit not between 0 and 1");
    }
  } else {
    v_limit = recovery_path_.getVelocity();
  }
  // This is the PFL failsafe path that ends in a velocity that is under v_iso.
  double eps_ds = 0.001;
  // Plan to be slightly lower than v_limit to eliminate numerical errors
  planning_success = planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(), v_limit - eps_ds, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
  // If the entire long-term trajectory is under iso-velocity, we dont need to check the velocity of the failsafe-path
  if(!is_under_iso_velocity) {
    // Check if the entire failsafe path is under iso-velocity
    // The failsafe path could start at d_s < 1, so the velocity of the failsafe path could be smaller than the LTT velocity.
    double max_d_s = failsafe_path_2_.getMaxVelocity();
    is_under_v_limit_ = max_d_s < v_limit;
  } else {
    is_under_v_limit_ = true;
  }
  return planning_success;
}

bool SafetyShield::checkMotionForJointLimits(Motion& motion) {
  for (int i = 0; i < nb_joints_; i++) {
    if (motion.getAngle()[i] < q_min_allowed_[i] || motion.getAngle()[i] > q_max_allowed_[i]) {
      return false;
    }
  }
  return true;
}

Motion SafetyShield::determineNextMotion(bool is_safe) {
  Motion next_motion;
  double s_d, ds_d, dds_d, ddds_d;
  if (is_safe) {
    // Fill potential buffer with position and velocity from recovery path
    if (recovery_path_.getPosition() >= failsafe_path_.getPosition()) {
      s_d = recovery_path_.getPosition();
      ds_d = recovery_path_.getVelocity();
      dds_d = recovery_path_.getAcceleration();
      ddds_d = recovery_path_.getJerk();
    } else {
      potential_path_.increment(sample_time_);
      s_d = potential_path_.getPosition();
      ds_d = potential_path_.getVelocity();
      dds_d = potential_path_.getAcceleration();
      ddds_d = potential_path_.getJerk();
    }

    // Interpolate from new long term buffer
    if (new_ltt_) {
      next_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d);
    } else {
      next_motion =
          long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d);
    }
    // Set potential path as new verified safe path
    safe_path_ = potential_path_;
  } else {
    // interpolate from old safe path
    safe_path_.increment(sample_time_);
    s_d = safe_path_.getPosition();
    ds_d = safe_path_.getVelocity();
    dds_d = safe_path_.getAcceleration();
    ddds_d = safe_path_.getJerk();
    next_motion =
        long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d);
  }
  /// !!! Set s to the new path position !!!
  path_s_ = s_d;
  // Return the calculated next motion
  return next_motion;
}

Motion SafetyShield::step(double cycle_begin_time) {
  cycle_begin_time_ = cycle_begin_time;
  try {
    // Get current motion
    Motion current_motion = getCurrentMotion();
    std::vector<double> alpha_i;
    evaluateNewLTTProcessed(current_motion);
    // Check if there is a new goal motion
    if (new_goal_) {
      newGoalPlanning(current_motion);
    }
    if (new_ltt_) {
      alpha_i = new_long_term_trajectory_.getAlphaI();
    } else {
      alpha_i = long_term_trajectory_.getAlphaI();
    }
    // If there is a new long term trajectory (LTT), always override is_safe with false.
    if (new_ltt_ && !new_ltt_processed_) {
      is_safe_ = false;
    }
    // Compute a new potential trajectory
    Motion goal_motion = computesPotentialTrajectory(is_safe_, next_motion_.getVelocity());
    is_safe_ = verifySafety(current_motion, goal_motion, alpha_i);
    // Select the next motion based on the verified safety
    next_motion_ = determineNextMotion(is_safe_);
    next_motion_.setTime(cycle_begin_time);
    new_ltt_processed_ = true;
    return next_motion_;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::getNextCycle: {}", exc.what());
    return getCurrentMotion();
  }
}

void SafetyShield::evaluateNewLTTProcessed(Motion& current_motion) {
  // If the new LTT was processed at least once and is labeled safe (stopped = safe for replanning), replace old LTT with new one.
  if (new_ltt_ && new_ltt_processed_ && (is_safe_ || current_motion.isStopped())) {
    long_term_trajectory_ = new_long_term_trajectory_;
    new_ltt_ = false;
    new_goal_ = false;
  }
}

void SafetyShield::newGoalPlanning(Motion& current_motion) {
  if (!new_goal_) {
    throw std::logic_error("SafetyShield::newGoalPlanning called without new goal.");
  }
  // Check if current motion has acceleration and jerk values that lie in the plannable ranges
  if (!checkCurrentMotionForReplanning(current_motion)) {
    new_ltt_ = false;
    return;
  }
  // Check if the starting position of the last replanning was very close to the current position
  bool last_close = new_ltt_ && abs(path_s_ - last_replan_s_) < sample_time_;
  // Only replan if the current joint position is different from the last.
  bool new_ltt_calculated = false;
  if (!last_close) {
    new_ltt_calculated = calculateLongTermTrajectory(
      current_motion.getAngle(),
      current_motion.getVelocity(),
      current_motion.getAcceleration(), 
      new_goal_motion_.getAngle(),
      new_long_term_trajectory_
    );
  }
  // A replanning happend, so set the last replan position to the current position
  if (!last_close && new_ltt_calculated) {
    last_replan_s_ = path_s_;
  }
  // If we calculated a new LTT, we need to process it in the next step
  // If the last replan was close to the current position, we treat this like a new plan
  if (last_close || new_ltt_calculated) {
    new_ltt_ = true;
    new_ltt_processed_ = false;
  } else {
    new_ltt_ = false;
  }
}

bool SafetyShield::verifySafety(Motion& current_motion, Motion& goal_motion, const std::vector<double>& alpha_i) {
  if (shield_type_ == ShieldType::OFF) {
    return true;
  }
  // Check motion for joint limits (goal motion needed here as it is end of failsafe)
  if (!checkMotionForJointLimits(goal_motion)) {
    return false;
  }
  bool is_safe = false;
  // Compute the robot reachable set for the potential trajectory
  std::vector<double> time_points = calcTimePointsForEquidistantIntervals(current_motion.getTime(), goal_motion.getTime(), reachability_set_duration_);
  std::vector<Motion> interval_edges_motions = getMotionsFromCurrentLTTandPath(time_points);
  robot_capsules_time_intervals_ = robot_reach_->reachTimeIntervals(interval_edges_motions, alpha_i);
  // Compute the human reachable sets for the potential trajectory
  // humanReachabilityAnalysis(t_command, t_brake)
  human_capsules_time_intervals_ = human_reach_->humanReachabilityAnalysisTimeIntervals(cycle_begin_time_, time_points);
  int collision_index = -1;
  if (shield_type_ == ShieldType::PFL) {
    is_safe = verifyContactEnergySafetyByContactType(time_points, interval_edges_motions, collision_index);
    /*
    // is_safe = unconstrainedContactConstraint AND constrainedContactConstraint
    is_safe = verifyContactEnergySafety(time_points, interval_edges_motions, collision_index);
    // Weird way of writing AND to make sure that the collision index is set to the correct value.
    if (is_safe) {
      is_safe = verifyConstrainedContactSafety(time_points, interval_edges_motions, collision_index);    
    }
    */
    // Debugging
    // if (!is_safe) {
    //   spdlog::warn("Collision detected at time point {} with robot EEF velocity {}.", time_points[collision_index], robot_link_velocities[collision_index][nb_joints_ - 1]);
    // }
  } else {
    // Verify if the robot and human reachable sets are collision free
    is_safe = verify_->verifyHumanReachTimeIntervals(robot_capsules_time_intervals_, human_capsules_time_intervals_, collision_index);
  }
  // for visualization in hrgym, reachability sets of last timestep are used
  if (is_safe) {
    robot_capsules_ = robot_capsules_time_intervals_[robot_capsules_time_intervals_.size()-1];
    human_capsules_ = human_capsules_time_intervals_[human_capsules_time_intervals_.size()-1];
  } else {
    robot_capsules_ = robot_capsules_time_intervals_[collision_index];
    human_capsules_ = human_capsules_time_intervals_[collision_index];
  }
  return is_safe;
}

bool SafetyShield::verifyContactEnergySafety(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index) {
  std::vector<std::vector<double>> maximal_contact_energies = human_reach_->getMaxContactEnergy();
  std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> inertias_beginning_of_intervals = 
    getInertiaMatricesFromCurrentLTTandPath(std::vector<double>(time_points.begin(), time_points.end() - 1));
  bool is_safe = verify_->verifyHumanReachEnergyInertiaMatrices(
    robot_capsules_time_intervals_,
    human_capsules_time_intervals_,
    inertias_beginning_of_intervals,
    std::vector<Motion>(interval_edges_motions.begin(), interval_edges_motions.end() - 1),
    maximal_contact_energies,
    collision_index
  );
  return is_safe;
}

bool SafetyShield::verifyContactVelocitySafety(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index) {
  double dt = time_points[1] - time_points[0];
  std::vector<double> velocity_errors = robot_reach_->calculateMaxVelErrors(dt, v_max_allowed_, a_max_allowed_, j_max_allowed_);
  std::vector<std::vector<double>> robot_link_velocities = calculateMaxRobotLinkVelocitiesPerTimeInterval(
    interval_edges_motions,
    velocity_errors
  );
  std::vector<std::vector<double>> robot_link_reflected_masses = robot_reach_->calculateRobotLinkReflectedMassesPerTimeInterval(interval_edges_motions);
  std::vector<std::vector<double>> maximal_contact_energies = human_reach_->getMaxContactEnergy();
  bool is_safe = verify_->verifyHumanReachEnergyReflectedMasses(
    robot_capsules_time_intervals_,
    human_capsules_time_intervals_,
    robot_link_velocities,
    robot_link_reflected_masses,
    maximal_contact_energies,
     collision_index);
  return is_safe;
}

bool SafetyShield::verifyContactEnergySafetyByContactType(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index) {
  std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator> vel_cap_start, vel_cap_end;
  buildRobotVelocityPointers(interval_edges_motions, vel_cap_start, vel_cap_end);
  std::vector<double> velocity_errors = robot_reach_->calculateMaxVelErrors(sample_time_, v_max_allowed_, a_max_allowed_, j_max_allowed_);
  std::vector<std::vector<double>> human_radii = human_reach_->getAllHumanRadii();
  std::vector<std::unordered_map<int, std::set<int>>> unclampable_body_part_maps = human_reach_->getUnclampableMap();
  std::unordered_map<int, std::set<int>> unclampable_enclosures_map = robot_reach_->getUnclampableEnclosures();
  std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> inertias_beginning_of_intervals = 
    getInertiaMatricesFromCurrentLTTandPath(std::vector<double>(time_points.begin(), time_points.end() - 1));
  return verify_->verifyHumanReachEnergyContactCases(
    robot_capsules_time_intervals_,
    human_capsules_time_intervals_,
    inertias_beginning_of_intervals,
    std::vector<Motion>(interval_edges_motions.begin(), interval_edges_motions.end() - 1),
    max_contact_energies_unconstrained_,
    max_contact_energies_constrained_,
    environment_elements_,
    human_radii,
    unclampable_body_part_maps,
    unclampable_enclosures_map,
    vel_cap_start,
    vel_cap_end,
    velocity_errors,
    collision_index);
}

void SafetyShield::buildRobotVelocityPointers(
    const std::vector<Motion>& interval_edges_motions,
    std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator>& vel_cap_start,
    std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator>& vel_cap_end) {
  for (int i = 0; i < interval_edges_motions.size()-1; i++) {
    // We don't move this loop to the verification class as we need to call the getVelocityCapsuleIterator function here.
    if (new_ltt_) {
      vel_cap_start.push_back(new_long_term_trajectory_.getVelocityCapsuleIterator(
        new_long_term_trajectory_.getLowerIndex(interval_edges_motions[i].getS())));
      vel_cap_end.push_back(new_long_term_trajectory_.getVelocityCapsuleIterator(
        new_long_term_trajectory_.getUpperIndex(interval_edges_motions[i+1].getS())));
    } else {
      vel_cap_start.push_back(long_term_trajectory_.getVelocityCapsuleIterator(
        long_term_trajectory_.getLowerIndex(interval_edges_motions[i].getS())));
      vel_cap_end.push_back(long_term_trajectory_.getVelocityCapsuleIterator(
        long_term_trajectory_.getUpperIndex(interval_edges_motions[i+1].getS())));
    }
  }
}

bool SafetyShield::verifyConstrainedContactSafety(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index) {
  std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator> vel_cap_start, vel_cap_end;
  buildRobotVelocityPointers(interval_edges_motions, vel_cap_start, vel_cap_end);
  std::vector<double> velocity_errors = robot_reach_->calculateMaxVelErrors(sample_time_, v_max_allowed_, a_max_allowed_, j_max_allowed_);
  std::vector<std::vector<double>> human_radii = human_reach_->getAllHumanRadii();
  std::vector<std::unordered_map<int, std::set<int>>> unclampable_body_part_maps = human_reach_->getUnclampableMap();
  std::unordered_map<int, std::set<int>> unclampable_enclosures_map = robot_reach_->getUnclampableEnclosures();
  bool is_safe = true;
  collision_index = -1;
  for (int i = 0; i < time_points.size()-2; i++) {
    is_safe = verify_->verifyClamping(robot_capsules_time_intervals_[i], human_capsules_time_intervals_[i], environment_elements_,
      human_radii, unclampable_body_part_maps, unclampable_enclosures_map,
      vel_cap_start[i], vel_cap_end[i], velocity_errors);
    if (!is_safe) {
      collision_index = i;
      break;
    }
  }
  return is_safe;
}

Motion SafetyShield::getCurrentMotion() {
  Motion current_pos;
  if (!recovery_path_.isCurrent()) {
    current_pos = long_term_trajectory_.interpolate(failsafe_path_.getPosition(), failsafe_path_.getVelocity(),
                                                    failsafe_path_.getAcceleration(), failsafe_path_.getJerk());
  } else {
    current_pos = long_term_trajectory_.interpolate(recovery_path_.getPosition(), recovery_path_.getVelocity(),
                                                    recovery_path_.getAcceleration(), recovery_path_.getJerk());
  }
  return current_pos;
}

bool SafetyShield::checkCurrentMotionForReplanning(Motion& current_motion) {
  for (int i = 0; i < nb_joints_; i++) {
    if (std::abs(current_motion.getAcceleration()[i]) > a_max_ltt_[i]) {
      return false;
    }
  }
  return true;
}

void SafetyShield::newLongTermTrajectory(const std::vector<double>& goal_position,
                                         const std::vector<double>& goal_velocity) {
  try {
    std::vector<double> motion_q;
    motion_q.reserve(nb_joints_);
    std::vector<double> motion_dq;
    motion_dq.reserve(nb_joints_);
    for (int i = 0; i < nb_joints_; i++) {
      motion_q.push_back(std::clamp(
        goal_position[i],
        q_min_allowed_[i] + planning_qpos_tolerance_,
        q_max_allowed_[i] - planning_qpos_tolerance_
      ));
      motion_dq.push_back(std::clamp(goal_velocity[i], -v_max_allowed_[i], v_max_allowed_[i]));
    }
    new_goal_motion_ = Motion(cycle_begin_time_, motion_q, motion_dq);
    new_goal_ = true;
    new_ltt_ = false;
    new_ltt_processed_ = false;
    // High negative number to make sure we replan the LTT.
    // last_replan_s_ = -1E9;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::newLongTermTrajectory: {}", exc.what());
  }
}

void SafetyShield::setLongTermTrajectory(LongTermTraj& traj) {
  Motion current_motion = getCurrentMotion();
  // Check if robot is at stop
  if (!current_motion.isStopped()) {
    throw RobotMovementException();
  }
  Motion start = traj.getNextMotionAtIndex(0);
  // Check if traj starts at the same position
  if (!current_motion.hasSamePos(&start)) {
    throw TrajectoryException("Given LTT does not start at current robot position.");
  }
  // Check if traj starts at v=0
  if (!current_motion.hasSameVel(&start)) {
    std::stringstream ss;
    ss << "Given LTT does not start with velocity 0.0. Start velocity is [";
    for (size_t i = 0; i < start.getVelocity().size(); ++i) {
      if (i != 0) ss << ",";
      ss << start.getVelocity()[i];
    }
    ss << "].";
    std::string s = ss.str();
    throw TrajectoryException(s);
  }
  // Check if traj ends in stop
  if (!traj.getNextMotionAtIndex(traj.getLength() - 1).isStopped()) {
    throw TrajectoryException("Given LTT does not end in a complete stop of the robot (v = a = j = 0.0)");
  }
  // Replace LTT
  new_long_term_trajectory_ = traj;
  new_ltt_ = true;
  new_ltt_processed_ = true;
}

bool SafetyShield::calculateLongTermTrajectory(const std::vector<double>& start_q, const std::vector<double> start_dq,
                                               const std::vector<double> start_ddq, const std::vector<double>& goal_q,
                                               LongTermTraj& ltt) {
  long_term_planner::Trajectory trajectory;
  bool success = ltp_.planTrajectory(goal_q, start_q, start_dq, start_ddq, trajectory);
  if (!success) return false;
  std::vector<Motion> new_traj(trajectory.length + 1);
  double new_time = path_s_;
  new_traj[0] = Motion(new_time, start_q, start_dq, start_ddq, 0.0);
  new_time += sample_time_;
  std::vector<double> q(nb_joints_);
  std::vector<double> dq(nb_joints_);
  std::vector<double> ddq(nb_joints_);
  std::vector<double> dddq(nb_joints_);
  for (int i = 0; i < trajectory.length; i++) {
    for (int j = 0; j < nb_joints_; j++) {
      q[j] = trajectory.q[j][i];
      dq[j] = trajectory.v[j][i];
      ddq[j] = trajectory.a[j][i];
      dddq[j] = trajectory.j[j][i];
    }
    new_traj[i + 1] = Motion(new_time, q, dq, ddq, dddq);
    new_time += sample_time_;
  }
  if (shield_type_ == ShieldType::OFF || shield_type_ == ShieldType::SSM) {
    ltt = LongTermTraj(
      new_traj, sample_time_, path_s_discrete_, 
      v_max_allowed_, a_max_allowed_, j_max_allowed_,
      sliding_window_k_, alpha_i_max_
    );
  } else {
    ltt = LongTermTraj(
      new_traj, sample_time_, *robot_reach_, path_s_discrete_,
      v_max_allowed_, a_max_allowed_, j_max_allowed_, sliding_window_k_
    );
  }
  return true;
}

void SafetyShield::buildMaxContactEnergies() {
  max_contact_energies_unconstrained_ = {};
  max_contact_energies_constrained_ = {};
  // First, add the link contact energies
  for (int i = 0; i < nb_joints_ - 1; i++) {
    max_contact_energies_unconstrained_.push_back(human_reach_->getMaxContactEnergyFromType(false, ContactType::LINK));
    max_contact_energies_constrained_.push_back(human_reach_->getMaxContactEnergyFromType(true, ContactType::LINK));
  }
  // Then, add the EEF contact energies
  max_contact_energies_unconstrained_.push_back(human_reach_->getMaxContactEnergyFromType(false, eef_contact_type_));
  max_contact_energies_constrained_.push_back(human_reach_->getMaxContactEnergyFromType(true, eef_contact_type_));
}

std::vector<Motion> SafetyShield::getMotionsFromCurrentLTTandPath(const std::vector<double>& time_points) {
  LongTermTraj& ltt = long_term_trajectory_;
  if (new_ltt_) {
    ltt = new_long_term_trajectory_;
  }
  return getMotionsOfAllTimeStepsFromPath(ltt, potential_path_, time_points);
}

std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> SafetyShield::getInertiaMatricesFromCurrentLTTandPath(
  const std::vector<double>& time_points
) {
  LongTermTraj& ltt = long_term_trajectory_;
  if (new_ltt_) {
    ltt = new_long_term_trajectory_;
  }
  return getInertiaMatricesOfAllTimeStepsFromPath(ltt, potential_path_, time_points);
}

}  // namespace safety_shield