#include "safety_shield/safety_shield.h"

namespace safety_shield {

SafetyShield::SafetyShield()
    : max_s_stop_(0),
      v_max_allowed_({0, 0, 0}),
      a_max_allowed_({0, 0, 0}),
      j_max_allowed_({0, 0, 0}),
      a_max_ltt_({0, 0, 0}),
      j_max_ltt_({0, 0, 0}) {
  spdlog::info("Safety shield created.");
}

SafetyShield::SafetyShield(int nb_joints, double sample_time, double max_s_stop,
                           const std::vector<double>& v_max_allowed, const std::vector<double>& a_max_allowed,
                           const std::vector<double>& j_max_allowed, const std::vector<double>& a_max_path,
                           const std::vector<double>& j_max_path, const LongTermTraj& long_term_trajectory,
                           RobotReach* robot_reach, HumanReach* human_reach, Verify* verify, ShieldType shield_type)
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
      shield_type_(shield_type) {
  sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
  std::vector<double> prev_dq;
  for (int i = 0; i < nb_joints_; i++) {
    prev_dq.push_back(0.0);
  }

  if (shield_type_ == ShieldType::OFF) {
    is_safe_ = true;
    Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
  } else if (shield_type_ != ShieldType::SEVERAL_PFL) {
    is_safe_ = false;
    Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
  } else {
    // shield_type == SEVERAL_PFL
    verification_level_ = Verification_level::HEAD;
    computesPotentialTrajectoryForSeveralPfl(verification_level_, prev_dq, nullptr, nullptr);
    next_motion_ = determineNextMotionForSeveralPfl(verification_level_);
  }
  std::vector<double> q_min(nb_joints, -3.141);
  std::vector<double> q_max(nb_joints, -3.141);
  ltp_ =
      long_term_planner::LongTermPlanner(nb_joints, sample_time, q_min, q_max, v_max_allowed, a_max_path, j_max_path);
  spdlog::info("Safety shield created.");
}

SafetyShield::SafetyShield(double sample_time, std::string trajectory_config_file, std::string robot_config_file,
                           std::string mocap_config_file, double init_x, double init_y, double init_z, double init_roll,
                           double init_pitch, double init_yaw, const std::vector<double>& init_qpos,
                           ShieldType shield_type)
    : sample_time_(sample_time), path_s_(0), path_s_discrete_(0), shield_type_(shield_type) {
  ///////////// Build robot reach
  YAML::Node robot_config = YAML::LoadFile(robot_config_file);
  std::string robot_name = robot_config["robot_name"].as<std::string>();
  nb_joints_ = robot_config["nb_joints"].as<int>();
  std::vector<double> transformation_matrices = robot_config["transformation_matrices"].as<std::vector<double>>();
  std::vector<double> enclosures = robot_config["enclosures"].as<std::vector<double>>();
  double secure_radius = robot_config["secure_radius"].as<double>();
  robot_reach_ = new RobotReach(transformation_matrices, nb_joints_, enclosures, init_x, init_y, init_z, init_roll,
                                init_pitch, init_yaw, secure_radius);
  ////////////// Setting trajectory variables
  YAML::Node trajectory_config = YAML::LoadFile(trajectory_config_file);
  max_s_stop_ = trajectory_config["max_s_stop"].as<double>();
  q_min_allowed_ = trajectory_config["q_min_allowed"].as<std::vector<double>>();
  q_max_allowed_ = trajectory_config["q_max_allowed"].as<std::vector<double>>();
  v_max_allowed_ = trajectory_config["v_max_allowed"].as<std::vector<double>>();
  a_max_allowed_ = trajectory_config["a_max_allowed"].as<std::vector<double>>();
  j_max_allowed_ = trajectory_config["j_max_allowed"].as<std::vector<double>>();
  a_max_ltt_ = trajectory_config["a_max_ltt"].as<std::vector<double>>();
  j_max_ltt_ = trajectory_config["j_max_ltt"].as<std::vector<double>>();
  ltp_ = long_term_planner::LongTermPlanner(nb_joints_, sample_time, q_min_allowed_, q_max_allowed_, v_max_allowed_,
                                            a_max_ltt_, j_max_ltt_);
  v_safe_ = trajectory_config["v_safe"].as<double>();
  alpha_i_max_ = trajectory_config["alpha_i_max"].as<double>();
  RobotReach::Velocity_method velocity_method =
      static_cast<RobotReach::Velocity_method>(trajectory_config["velocity_method"].as<int>());
  robot_reach_->setVelocityMethod(velocity_method);
  spdlog::info("velocity_method = {}", velocity_method);
  // Initialize the long term trajectory
  sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
  std::vector<Motion> long_term_traj;
  long_term_traj.push_back(Motion(0.0, init_qpos));
  if (shield_type_ == ShieldType::SSM || shield_type_ == ShieldType::OFF) {
    long_term_trajectory_ =
        LongTermTraj(long_term_traj, sample_time_, path_s_discrete_, sliding_window_k_, alpha_i_max_);
  } else {
    long_term_trajectory_ =
        LongTermTraj(long_term_traj, sample_time_, *robot_reach_, path_s_discrete_, sliding_window_k_);
  }
  //////////// Build human reach
  YAML::Node human_config = YAML::LoadFile(mocap_config_file);
  double measurement_error_pos = human_config["measurement_error_pos"].as<double>();
  double measurement_error_vel = human_config["measurement_error_vel"].as<double>();
  double delay = human_config["delay"].as<double>();

  std::vector<std::string> joint_name_vec = human_config["joint_names"].as<std::vector<std::string>>();
  std::map<std::string, int> joint_names;
  for (std::size_t i = 0; i < joint_name_vec.size(); ++i) {
    joint_names[joint_name_vec[i]] = i;
  }

  std::vector<double> joint_v_max = human_config["joint_v_max"].as<std::vector<double>>();
  std::vector<double> joint_a_max = human_config["joint_a_max"].as<std::vector<double>>();
  // Build bodies
  const YAML::Node& bodies = human_config["bodies"];
  std::map<std::string, reach_lib::jointPair> body_link_joints;
  std::map<std::string, double> thickness;
  for (YAML::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
    const YAML::Node& body = *it;
    body_link_joints[body["name"].as<std::string>()] = reach_lib::jointPair(
        joint_names[body["proximal"].as<std::string>()], joint_names[body["distal"].as<std::string>()]);
    thickness[body["name"].as<std::string>()] = body["thickness"].as<double>();
  }
  // create data struct which maps from body part to its index (in the capsule list) and maximum collision velocity
  int count = 0;
  std::map<std::string, std::pair<int, double>> body_to_index_and_velocity;
  for (const auto& entry : body_link_joints) {
    body_to_index_and_velocity[entry.first] = std::pair<int, double>(count++, 0.0);
  }
  for (YAML::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
    const YAML::Node& body = *it;
    body_to_index_and_velocity[body["name"].as<std::string>()].second = body["maximum_collision_velocity"].as<double>();
  }
  // Build extremities
  const YAML::Node& extremities = human_config["extremities"];
  std::vector<std::string> extremity_base_names;
  std::vector<std::string> extremity_end_names;
  std::vector<double> extremity_length, extremity_thickness;
  for (YAML::const_iterator it = extremities.begin(); it != extremities.end(); ++it) {
    const YAML::Node& extremity = *it;
    extremity_base_names.push_back(extremity["base"].as<std::string>());
    extremity_end_names.push_back(extremity["end"].as<std::string>());
    extremity_length.push_back(extremity["length"].as<double>());
    extremity_thickness.push_back(extremity["thickness"].as<double>());
  }
  human_reach_ = new HumanReach(joint_names.size(), joint_names, body_link_joints, body_to_index_and_velocity, thickness, joint_v_max, joint_a_max,
                                extremity_base_names, extremity_end_names, extremity_length, extremity_thickness,
                                measurement_error_pos, measurement_error_vel, delay);
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
    Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
  } else if (shield_type_ != ShieldType::SEVERAL_PFL) {
    is_safe_ = false;
    Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
  } else {
    // shield_type == SEVERAL_PFL
    verification_level_ = Verification_level::HEAD;
    computesPotentialTrajectoryForSeveralPfl(verification_level_, prev_dq, nullptr, nullptr);
    next_motion_ = determineNextMotionForSeveralPfl(verification_level_);
  }
  spdlog::info("Safety shield created.");
}

void SafetyShield::reset(double init_x, double init_y, double init_z, double init_roll, double init_pitch,
                         double init_yaw, const std::vector<double>& init_qpos, double current_time,
                         ShieldType shield_type) {
  shield_type_ = shield_type;
  robot_reach_->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
  human_reach_->reset();
  std::vector<double> prev_dq;
  for (int i = 0; i < nb_joints_; i++) {
    prev_dq.push_back(0.0);
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
  failsafe_path_head_ = Path();
  failsafe_path_head_2_ = Path();
  failsafe_path_non_head_ = Path();
  failsafe_path_non_head_2_ = Path();
  // Initialize the long term trajectory
  std::vector<Motion> long_term_traj;
  long_term_traj.push_back(Motion(0.0, init_qpos));
  if (shield_type_ == ShieldType::SSM || shield_type_ == ShieldType::OFF) {
    long_term_trajectory_ =
        LongTermTraj(long_term_traj, sample_time_, path_s_discrete_, sliding_window_k_, alpha_i_max_);
  } else {
    long_term_trajectory_ =
        LongTermTraj(long_term_traj, sample_time_, *robot_reach_, path_s_discrete_, sliding_window_k_);
  }
  if (shield_type_ == ShieldType::OFF) {
    is_safe_ = true;
    Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
  } else if (shield_type_ != ShieldType::SEVERAL_PFL) {
    is_safe_ = false;
    Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
  } else {
    // shield_type == SEVERAL_PFL
    verification_level_ = Verification_level::HEAD;
    computesPotentialTrajectoryForSeveralPfl(verification_level_, prev_dq, nullptr, nullptr);
    next_motion_ = determineNextMotionForSeveralPfl(verification_level_);
  }
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
      std::array<double, 6> new_phases = {sample_time_, sample_time_, sample_time_, 0.0, 0.0, 0.0};
      path.setPhases(new_phases);
      return true;
    }
    // Time to reach zero acceleration with maximum jerk
    double t_to_a_0 = roundToTimestep(abs(acc) / j_max);
    // Velocity at zero acceleration
    double v_at_a_0 = vel + acc * t_to_a_0 / 2;

    // If the velocity at zero acceleration is the goal velocity, use t_to_a_0 and max jerk and return
    if (fabs(v_at_a_0 - ve) < epsilon) {
      std::array<double, 6> new_phases = {t_to_a_0, t_to_a_0, t_to_a_0, -acc / t_to_a_0, 0, 0};
      path.setPhases(new_phases);
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
    std::array<double, 6> new_phases = {t01, t01 + t12, t01 + t12 + t23, j01, j12, j23};
    path.setPhases(new_phases);
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
      if (shield_type_ == ShieldType::PFL) {
        // Plan new failsafe path for PFL
        failsafe_2_planning_success = planPFLFailsafe(a_max_manoeuvre, j_max_manoeuvre);
      } else {
        // plan new failsafe path for STP
        failsafe_2_planning_success =
            planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(),
                             recovery_path_.getAcceleration(), 0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
      }
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
    // Fill potential buffer with position and velocity from last failsafe path. This value is not really used.
    double s_d = failsafe_path_.getPosition();
    double ds_d = failsafe_path_.getVelocity();
    double dds_d = failsafe_path_.getAcceleration();
    double ddds_d = failsafe_path_.getJerk();
    // Calculate goal
    potential_path_.getFinalMotion(s_d, ds_d, dds_d);
    Motion goal_motion;
    if (new_ltt_) {
      goal_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_,
                                                          j_max_allowed_);
    } else {
      goal_motion =
          long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
    }
    goal_motion.setTime(potential_path_.getPhase(3));
    return goal_motion;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::computesPotentialTrajectory: {}", exc.what());
    throw exc;
    return Motion();
  }
}

void SafetyShield::computesPotentialTrajectoryForSeveralPfl(Verification_level v, const std::vector<double>& prev_speed,
                                                            Motion* goal_motion_head, Motion* goal_motion_non_head) {
  try {
    // s_int indicates the index of the entire traveled way
    while (path_s_ >= (path_s_discrete_ + 1) * sample_time_) {
      long_term_trajectory_.increasePosition();
      if (new_ltt_) {
        new_long_term_trajectory_.increasePosition();
      }
      path_s_discrete_++;
    }
    // If verified safe, take the recovery path, otherwise, take one of the failsafe paths
    if (v == Verification_level::SAFE && recovery_path_correct_) {
      recovery_path_.setCurrent(true);
      // discard old FailsafePath and replace with new one
      failsafe_path_head_ = failsafe_path_head_2_;
      failsafe_path_non_head_ = failsafe_path_non_head_2_;
      // repair path already incremented
    } else if (v == Verification_level::NON_HEAD) {
      failsafe_path_non_head_.setCurrent(true);
      failsafe_path_non_head_.increment(sample_time_);
      recovery_path_.setCurrent(false);
      failsafe_path_head_.setCurrent(false);
    } else {
      // v == Verification_level::HEAD
      failsafe_path_head_.setCurrent(true);
      failsafe_path_head_.increment(sample_time_);
      recovery_path_.setCurrent(false);
      failsafe_path_non_head_.setCurrent(false);
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
      double pos, vel, acc;
      if(failsafe_path_head_.isCurrent()) {
        pos = failsafe_path_head_.getPosition();
        vel = failsafe_path_head_.getVelocity();
        acc = failsafe_path_head_.getAcceleration();
      } else if(failsafe_path_non_head_.isCurrent()) {
        pos = failsafe_path_non_head_.getPosition();
        vel = failsafe_path_non_head_.getVelocity();
        acc = failsafe_path_non_head_.getAcceleration();
      } else {
        spdlog::error("Error in computesPotentialTrajectoryForSeveralPfl: no failsafe-path is set when planning repair path?");
      }
      recovery_path_correct_ =
          planSafetyShield(pos, vel, acc, 1, a_max_manoeuvre, j_max_manoeuvre, recovery_path_);
    }
    // Only plan new failsafe trajectory if the recovery path planning was successful.
    if (recovery_path_correct_) {
      // advance one step on repair path
      recovery_path_.increment(sample_time_);
      // Plan new failsafe path for SEVERAL_PFL
      bool failsafe_head_2_planning_success = planSeveralPflFailsafe(a_max_manoeuvre, j_max_manoeuvre, v_safe_head_, failsafe_path_head_2_, is_under_v_limit_head_);
      bool failsafe_non_head_2_planning_success = planSeveralPflFailsafe(a_max_manoeuvre, j_max_manoeuvre, v_safe_non_head_, failsafe_path_non_head_2_, is_under_v_limit_non_head_);
      // Check the validity of the planned path
      if (!failsafe_head_2_planning_success || !failsafe_non_head_2_planning_success ||
          (failsafe_path_head_.isCurrent() && recovery_path_.getPosition() < failsafe_path_head_.getPosition()) ||
          (failsafe_path_non_head_.isCurrent() && recovery_path_.getPosition() < failsafe_path_non_head_.getPosition())) {
        recovery_path_correct_ = false;
      }
    }
    // If all planning was correct, use new failsafe path with single recovery step
    if (recovery_path_correct_) {
      potential_path_head_ = failsafe_path_head_2_;
      potential_path_non_head_ = failsafe_path_non_head_2_;
    } else {
      // If planning failed, use previous failsafe path
      potential_path_head_ = failsafe_path_head_;
      potential_path_non_head_ = failsafe_path_non_head_;
    }

    if (goal_motion_head == nullptr) {
      // if its nullptr, we dont need to calculate the goal motions
      // spdlog::info("goal_motion_head is nullptr");
      return;
    }

    //// Calculate start and goal pos of intended motion
    // Fill potential buffer with position and velocity from last failsafe path. This value is not really used.
    double final_head_s_d, final_head_ds_d, final_head_dds_d;
    double final_non_head_s_d, final_non_head_ds_d, final_non_head_dds_d;

    // TODO: we need to know which was the last failsafe path used
    double head_ddds_d = failsafe_path_head_.getJerk();
    double non_head_ddds_d = failsafe_path_non_head_.getJerk();

    potential_path_head_.getFinalMotion(final_head_s_d, final_head_ds_d, final_head_dds_d);
    potential_path_non_head_.getFinalMotion(final_non_head_s_d, final_non_head_ds_d, final_non_head_dds_d);

    if (new_ltt_) {
      *goal_motion_head = new_long_term_trajectory_.interpolate(final_head_s_d, final_head_ds_d, final_head_dds_d,
                                                                head_ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
      *goal_motion_non_head =
          new_long_term_trajectory_.interpolate(final_non_head_s_d, final_non_head_ds_d, final_non_head_dds_d, non_head_ddds_d,
                                                v_max_allowed_, a_max_allowed_, j_max_allowed_);
    } else {
      *goal_motion_head = long_term_trajectory_.interpolate(final_head_s_d, final_head_ds_d, final_head_dds_d, head_ddds_d,
                                                            v_max_allowed_, a_max_allowed_, j_max_allowed_);
      *goal_motion_non_head =
          long_term_trajectory_.interpolate(final_non_head_s_d, final_non_head_ds_d, final_non_head_dds_d, non_head_ddds_d,
                                            v_max_allowed_, a_max_allowed_, j_max_allowed_);
    }
    goal_motion_head->setTime(potential_path_head_.getPhase(3));
    goal_motion_non_head->setTime(potential_path_non_head_.getPhase(3));
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::computesPotentialTrajectory: {}", exc.what());
    throw exc;
  }
}

bool SafetyShield::planPFLFailsafe(double a_max_manoeuvre, double j_max_manoeuvre) {
  // Calculate maximal Cartesian velocity in the short-term plan
  double s_d, ds_d, dds_d;
  // Calculate goal
  bool is_under_iso_velocity = false;
  // v_max is maximum of LTT or STP and vel_s_dot is how much path velocity needs to be scaled to be under v_iso
  // First, plan a failsafe path that ends in a complete stop. This is the longest failsafe path possible.
  bool planning_success =
      planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(),
                       0.0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
  if (!planning_success) {
    return false;
  }
  failsafe_path_2_.getFinalMotion(s_d, ds_d, dds_d);
  double v_max;
  if (!new_ltt_) {
    v_max = long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  } else {
    v_max = new_long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  }
  // Check if the entire long-term trajectory is under iso-velocity
  is_under_iso_velocity = v_max <= v_safe_;
  double v_limit;
  // TODO: if the current velocity is under iso-velocity, we dont need to compute the failsafe-path of the PFL-criterion
  if (!is_under_iso_velocity) {
    v_limit = v_safe_ / v_max;
    if (v_limit > 1 || v_limit < 0) {
      spdlog::error("v_limit not between 0 and 1");
    }
  } else {
    v_limit = recovery_path_.getVelocity();
  }
  // This is the PFL failsafe path that ends in a velocity that is under v_iso.
  planning_success =
      planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(),
                       v_limit, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
  // If the entire long-term trajectory is under iso-velocity, we dont need to check the velocity of the failsafe-path
  if (!is_under_iso_velocity) {
    // Check if the entire failsafe path is under iso-velocity
    // The failsafe path could start at d_s < 1, so the velocity of the failsafe path could be smaller than the LTT
    // velocity.
    double max_d_s = failsafe_path_2_.getMaxVelocity();
    is_under_v_limit_ = max_d_s < v_limit;
  } else {
    is_under_v_limit_ = true;
  }
  return planning_success;
}

bool SafetyShield::planSeveralPflFailsafe(double a_max_manoeuvre, double j_max_manoeuvre, double v_safe, Path& failsafe_path, bool& is_under_v_limit) {
  // Calculate maximal Cartesian velocity in the short-term plan
  double s_d, ds_d, dds_d;
  // Calculate goal
  bool is_under_iso_velocity = false;
  // v_max is maximum of LTT or STP and vel_s_dot is how much path velocity needs to be scaled to be under v_iso
  // First, plan a failsafe path that ends in a complete stop. This is the longest failsafe path possible.
  bool planning_success =
      planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(),
                       0.0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path);
  if (!planning_success) {
    return false;
  }
  failsafe_path.getFinalMotion(s_d, ds_d, dds_d);
  double v_max;
  if (!new_ltt_) {
    v_max = long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  } else {
    v_max = new_long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  }
  is_under_iso_velocity = v_max <= v_safe;
  double v_limit;
  // TODO: if the current velocity is under iso-velocity, we dont need to compute the failsafe-path of the PFL-criterion
  if (!is_under_iso_velocity) {
    v_limit = v_safe / v_max;
    if (v_limit > 1 || v_limit <= 0) {
      spdlog::error("v_limit not between 0 and 1");
    }
  } else {
    v_limit = recovery_path_.getVelocity();
  }
  // This is the PFL failsafe path that ends in a velocity v_head that is under v_iso.
  planning_success =
      planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(),
                       v_limit, a_max_manoeuvre, j_max_manoeuvre, failsafe_path);
  if (!is_under_iso_velocity) {
    double max_d_s = failsafe_path.getMaxVelocity();
    is_under_v_limit = max_d_s < v_limit;
  } else {
    is_under_v_limit = true;
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
      next_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_,
                                                          j_max_allowed_);
    } else {
      next_motion =
          long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
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
        long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
  }
  /// !!! Set s to the new path position !!!
  path_s_ = s_d;
  // Return the calculated next motion
  return next_motion;
}

Motion SafetyShield::determineNextMotionForSeveralPfl(Verification_level verification_level) {
  Motion next_motion;
  double s_d, ds_d, dds_d, ddds_d;
  // TODO: pos of failsafe_path_head and non_head should be the same
  double pos = failsafe_path_head_.getPosition();
  if (verification_level == Verification_level::SAFE) {
    // Fill potential buffer with position and velocity from recovery path
    if (recovery_path_.getPosition() >= pos) {
      s_d = recovery_path_.getPosition();
      ds_d = recovery_path_.getVelocity();
      dds_d = recovery_path_.getAcceleration();
      ddds_d = recovery_path_.getJerk();
    } else {
      // TODO: Problem: wir wissen nicht welche potential path verwendet wird
      potential_path_head_.increment(sample_time_);
      potential_path_non_head_.increment(sample_time_);
      s_d = potential_path_non_head_.getPosition();
      ds_d = potential_path_non_head_.getVelocity();
      dds_d = potential_path_non_head_.getAcceleration();
      ddds_d = potential_path_non_head_.getJerk();
    }

    // Interpolate from new long term buffer
    if (new_ltt_) {
      next_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_,
                                                          j_max_allowed_);
    } else {
      next_motion =
          long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
    }
    // TODO: Problem: wir wissen nicht welche potential path verwendet wird
    // Set potential path as new verified safe path
    safe_path_ = potential_path_non_head_;

  } else if (verification_level == Verification_level::NON_HEAD) {
    // Fill potential buffer with position and velocity from recovery path
    if (recovery_path_.getPosition() >= pos) {
      s_d = recovery_path_.getPosition();
      ds_d = recovery_path_.getVelocity();
      dds_d = recovery_path_.getAcceleration();
      ddds_d = recovery_path_.getJerk();
    } else {
      // TODO: Problem: wir wissen nicht welche potential path verwendet wird
      potential_path_head_.increment(sample_time_);
      potential_path_non_head_.increment(sample_time_);
      s_d = potential_path_head_.getPosition();
      ds_d = potential_path_head_.getVelocity();
      dds_d = potential_path_head_.getAcceleration();
      ddds_d = potential_path_head_.getJerk();
    }

    // Interpolate from new long term buffer
    if (new_ltt_) {
      next_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_,
                                                          j_max_allowed_);
    } else {
      next_motion =
          long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
    }
    // TODO: Problem: wir wissen nicht welche potential path verwendet wird
    // Set potential path as new verified safe path
    safe_path_ = potential_path_head_;
  } else {
    // interpolate from old safe path
    safe_path_.increment(sample_time_);
    s_d = safe_path_.getPosition();
    ds_d = safe_path_.getVelocity();
    dds_d = safe_path_.getAcceleration();
    ddds_d = safe_path_.getJerk();
    next_motion =
        long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
  }
  /// !!! Set s to the new path position !!!
  path_s_ = s_d;
  // Return the calculated next motion
  return next_motion;
}

Motion SafetyShield::step(double cycle_begin_time) {
  if (shield_type_ != ShieldType::SEVERAL_PFL) {
    return standardStep(cycle_begin_time);
  } else {
    return severalPflStep(cycle_begin_time);
  }
}

Motion SafetyShield::standardStep(double cycle_begin_time) {
  cycle_begin_time_ = cycle_begin_time;
  try {
    // Get current motion
    Motion current_motion = getCurrentMotion();
    std::vector<double> alpha_i;
    // If the new LTT was processed at least once and is labeled safe, replace old LTT with new one.
    if (new_ltt_ && new_ltt_processed_) {
      if (is_safe_ || current_motion.isStopped()) {
        long_term_trajectory_ = new_long_term_trajectory_;
        new_ltt_ = false;
        new_goal_ = false;
      }
    }
    // Check if there is a new goal motion
    if (new_goal_) {
      // Check if current motion has acceleration and jerk values that lie in the plannable ranges
      bool is_plannable = checkCurrentMotionForReplanning(current_motion);
      if (is_plannable) {
        // Check if the starting position of the last replanning was very close to the current position
        bool last_close = true;
        if (new_ltt_) {
          for (int i = 0; i < current_motion.getAngle().size(); i++) {
            if (std::abs(current_motion.getAngle()[i] - last_replan_start_motion_.getAngle()[i]) > 0.01) {
              last_close = false;
              break;
            }
          }
        } else {
          last_close = false;
        }
        // Only replan if the current joint position is different from the last.
        bool success = true;
        if (!last_close) {
          success = calculateLongTermTrajectory(current_motion.getAngle(), current_motion.getVelocity(),
                                                current_motion.getAcceleration(), new_goal_motion_.getAngle(),
                                                new_long_term_trajectory_);
          if (success) {
            last_replan_start_motion_ = current_motion;
          }
        }
        if (success) {
          new_ltt_ = true;
          new_ltt_processed_ = false;
        } else {
          new_ltt_ = false;
        }
      } else {
        new_ltt_ = false;
      }
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
    if (shield_type_ != ShieldType::OFF) {
      // Check motion for joint limits (goal motion needed here as it is end of failsafe)
      if (!checkMotionForJointLimits(goal_motion)) {
        is_safe_ = false;
      } else {
        // Compute the robot reachable set for the potential trajectory
        robot_capsules_ =
            robot_reach_->reach(current_motion, goal_motion, (goal_motion.getS() - current_motion.getS()), alpha_i);
        // Compute the human reachable sets for the potential trajectory
        // humanReachabilityAnalysis(t_command, t_brake)
        human_reach_->humanReachabilityAnalysis(cycle_begin_time_, goal_motion.getTime());
        human_capsules_ = human_reach_->getAllCapsules();
        // Verify if the robot and human reachable sets are collision free
        is_safe_ = verify_->verify_human_reach(robot_capsules_, human_capsules_);
        if (shield_type_ == ShieldType::PFL) {
          is_safe_ = is_safe_ || is_under_v_limit_;
        }
      }
    } else {
      is_safe_ = true;
    }

    //print_debug_standard(is_safe_, is_under_v_limit_);

    // Select the next motion based on the verified safety
    next_motion_ = determineNextMotion(is_safe_);
    next_motion_.setTime(cycle_begin_time);
    new_ltt_processed_ = true;
    return next_motion_;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::getNextCycle: {}", exc.what());
    return Motion();
  }
}

Motion SafetyShield::severalPflStep(double cycle_begin_time) {
  cycle_begin_time_ = cycle_begin_time;
  try {
    // Get current motion
    Motion current_motion = getCurrentMotionForSeveralPfl();
    std::vector<double> alpha_i;
    // If the new LTT was processed at least once and is labeled safe, replace old LTT with new one.
    if (new_ltt_ && new_ltt_processed_) {
      if (verification_level_ == Verification_level::SAFE || current_motion.isStopped()) {
        long_term_trajectory_ = new_long_term_trajectory_;
        new_ltt_ = false;
        new_goal_ = false;
      }
    }
    // Check if there is a new goal motion
    if (new_goal_) {
      // Check if current motion has acceleration and jerk values that lie in the plannable ranges
      bool is_plannable = checkCurrentMotionForReplanning(current_motion);
      if (is_plannable) {
        // Check if the starting position of the last replanning was very close to the current position
        bool last_close = true;
        if (new_ltt_) {
          for (int i = 0; i < current_motion.getAngle().size(); i++) {
            if (std::abs(current_motion.getAngle()[i] - last_replan_start_motion_.getAngle()[i]) > 0.01) {
              last_close = false;
              break;
            }
          }
        } else {
          last_close = false;
        }
        // Only replan if the current joint position is different from the last.
        bool success = true;
        if (!last_close) {
          success = calculateLongTermTrajectory(current_motion.getAngle(), current_motion.getVelocity(),
                                                current_motion.getAcceleration(), new_goal_motion_.getAngle(),
                                                new_long_term_trajectory_);
          if (success) {
            last_replan_start_motion_ = current_motion;
          }
        }
        if (success) {
          new_ltt_ = true;
          new_ltt_processed_ = false;
        } else {
          new_ltt_ = false;
        }
      } else {
        new_ltt_ = false;
      }
    }
    if (new_ltt_) {
      alpha_i = new_long_term_trajectory_.getAlphaI();
    } else {
      alpha_i = long_term_trajectory_.getAlphaI();
    }
    // If there is a new long term trajectory (LTT), always override verification_level
    if (new_ltt_ && !new_ltt_processed_) {
      verification_level_ = Verification_level::HEAD;
    }
    // Compute a new potential trajectory
    Motion goal_motion_head;
    Motion goal_motion_non_head;
    computesPotentialTrajectoryForSeveralPfl(verification_level_, next_motion_.getVelocity(), &goal_motion_head,
                                             &goal_motion_non_head);

    // Check motion for joint limits (goal motion needed here as it is end of failsafe)
    if (!(checkMotionForJointLimits(goal_motion_head) && checkMotionForJointLimits(goal_motion_non_head))) {
      verification_level_ = Verification_level::HEAD;
      spdlog::info("joint limits exceeded");
    } else {
      // es werden nur letzte Kapseln geplotted
      // non_head_path darf keine Kollision mit non-Kopf haben
      // Compute the robot reachable set for the potential non-head trajectory
      robot_capsules_ = robot_reach_->reach(current_motion, goal_motion_non_head,
                                            (goal_motion_non_head.getS() - current_motion.getS()), alpha_i);
      // Compute the human reachable sets for the potential non-head trajectory
      human_reach_->humanReachabilityAnalysis(cycle_begin_time_, goal_motion_non_head.getTime());
      // Verify if the robot and human reachable sets are collision free for non-head trajectory
      human_capsules_ = human_reach_->getAllCapsules();
      bool is_safe_non_head = verify_->verify_human_reach_non_head(
                                  robot_capsules_, human_reach_->getArticulatedVelCapsules(),
                                  human_reach_->getArticulatedAccelCapsules(), human_reach_->getBodyLinkJoints(),
                                  human_reach_->getBodyToIndexAndVelocity());
      is_safe_non_head = is_safe_non_head || is_under_v_limit_non_head_;

      // head_path darf keine Kollision mit Kopf haben
      // Compute the robot reachable set for the potential head trajectory
      robot_capsules_ = robot_reach_->reach(current_motion, goal_motion_head,
                                            (goal_motion_head.getS() - current_motion.getS()), alpha_i);
      // Compute the human reachable sets for the potential head trajectory
      human_reach_->humanReachabilityAnalysis(cycle_begin_time_, goal_motion_head.getTime());
      // Verify if the robot and human reachable sets are head-collision free for head trajectory
      human_capsules_ = human_reach_->getAllCapsules();
      bool is_safe_head = verify_->verify_human_reach_head(robot_capsules_, human_reach_->getArticulatedVelCapsules(),
                                                           human_reach_->getArticulatedAccelCapsules(),
                                                           human_reach_->getBodyLinkJoints(),
                                                           human_reach_->getBodyToIndexAndVelocity());
      is_safe_head = is_safe_head || is_under_v_limit_head_;

      // Verification Level bestimmen
      if (is_safe_head && is_safe_non_head) {
        verification_level_ = Verification_level::SAFE;
      } else if (!is_safe_head && is_safe_non_head) {
        verification_level_ = Verification_level::HEAD;
      } else if(is_safe_head && !is_safe_non_head) {
        verification_level_ = Verification_level::NON_HEAD;
      } else {
        // !is_safe_head && !is_safe_non_head
        verification_level_ = Verification_level::HEAD;
      }
      //print_debug_several_pfl(verification_level_, is_safe_head, is_safe_non_head, is_under_v_limit_head_, is_under_v_limit_non_head_);
    }

    // Select the next motion based on the verified safety
    next_motion_ = determineNextMotionForSeveralPfl(verification_level_);
    next_motion_.setTime(cycle_begin_time);
    new_ltt_processed_ = true;
    return next_motion_;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::getNextCycle: {}", exc.what());
    return Motion();
  }
}

Motion SafetyShield::getCurrentMotion() {
  Motion current_pos;
  if (!recovery_path_.isCurrent()) {
    current_pos = long_term_trajectory_.interpolate(failsafe_path_.getPosition(), failsafe_path_.getVelocity(),
                                                    failsafe_path_.getAcceleration(), failsafe_path_.getJerk(),
                                                    v_max_allowed_, a_max_allowed_, j_max_allowed_);
  } else {
    current_pos = long_term_trajectory_.interpolate(recovery_path_.getPosition(), recovery_path_.getVelocity(),
                                                    recovery_path_.getAcceleration(), recovery_path_.getJerk(),
                                                    v_max_allowed_, a_max_allowed_, j_max_allowed_);
  }
  return current_pos;
}

Motion SafetyShield::getCurrentMotionForSeveralPfl() {
  Motion current_pos;
  if (failsafe_path_head_.isCurrent()) {
    current_pos = long_term_trajectory_.interpolate(
        failsafe_path_head_.getPosition(), failsafe_path_head_.getVelocity(), failsafe_path_head_.getAcceleration(),
        failsafe_path_head_.getJerk(), v_max_allowed_, a_max_allowed_, j_max_allowed_);
  } else if (failsafe_path_non_head_.isCurrent()) {
    current_pos =
        long_term_trajectory_.interpolate(failsafe_path_non_head_.getPosition(), failsafe_path_non_head_.getVelocity(),
                                          failsafe_path_non_head_.getAcceleration(), failsafe_path_non_head_.getJerk(),
                                          v_max_allowed_, a_max_allowed_, j_max_allowed_);
  } else if (recovery_path_.isCurrent()) {
    current_pos = long_term_trajectory_.interpolate(recovery_path_.getPosition(), recovery_path_.getVelocity(),
                                                    recovery_path_.getAcceleration(), recovery_path_.getJerk(),
                                                    v_max_allowed_, a_max_allowed_, j_max_allowed_);
  } else {
    spdlog::error("Exception in SafetyShield::getCurrentMotionForSeveralPfl, no path is set");
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
      // TODO: replace with config max min values
      motion_q.push_back(std::clamp(goal_position[i], q_min_allowed_[i], q_max_allowed_[i]));
      motion_dq.push_back(std::clamp(goal_velocity[i], -v_max_allowed_[i], v_max_allowed_[i]));
    }
    new_goal_motion_ = Motion(cycle_begin_time_, motion_q, motion_dq);
    new_goal_ = true;
    new_ltt_ = false;
    new_ltt_processed_ = false;
    std::vector<double> dummy_q;
    for (int i = 0; i < nb_joints_; i++) {
      // fill with high values to guarantee this is not close to current joint position
      dummy_q.push_back(-1234567);
    }
    last_replan_start_motion_ = Motion(cycle_begin_time_, dummy_q);
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
  std::vector<Motion> new_traj(trajectory.length);
  double new_time = path_s_;
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
    new_traj[i] = Motion(new_time, q, dq, ddq, dddq);
    new_time += sample_time_;
  }
  if (shield_type_ == ShieldType::OFF || shield_type_ == ShieldType::SSM) {
    ltt = LongTermTraj(new_traj, sample_time_, path_s_discrete_, sliding_window_k_, alpha_i_max_);
  } else {
    ltt = LongTermTraj(new_traj, sample_time_, *robot_reach_, path_s_discrete_, sliding_window_k_);
  }
  return true;
}

}  // namespace safety_shield