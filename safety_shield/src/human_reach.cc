#include "safety_shield/human_reach.h"

namespace safety_shield {

HumanReach* createHumanReach(
      bool use_single_motion_model,
      bool use_kalman_filter,
      int n_joints_meas,
      std::map<std::string, int> joint_names,
      std::map<std::string, reach_lib::jointPair>& body_link_joints, 
      const std::map<std::string, double>& thickness, 
      std::vector<double>& max_v, 
      std::vector<double>& max_a,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay,
      std::vector<std::string> extremity_base_names = std::vector<std::string>(), 
      std::vector<std::string> extremity_end_names = std::vector<std::string>(), 
      std::vector<double> extremity_length = std::vector<double>(),
      std::vector<double> extremity_thickness = std::vector<double>(),
      double s_w = 2.0e+2,
      double s_v = 1.0e-6,
      double initial_pos_var = 0.0,
      double initial_vel_var = 0.0) {
  if (use_single_motion_model) {
    if (use_kalman_filter) {
      return new HumanReach(n_joints_meas, joint_names, body_link_joints, thickness, max_v, max_a, measurement_error_pos, measurement_error_vel, delay, s_w, s_v, initial_pos_var, initial_vel_var);
    } else {
      return new HumanReach(n_joints_meas, joint_names, body_link_joints, thickness, max_v, max_a, measurement_error_pos, measurement_error_vel, delay);
    }
  } else {
    if (use_kalman_filter) {
      return new HumanReach(n_joints_meas, joint_names, body_link_joints, thickness, max_v, max_a, extremity_base_names, extremity_end_names, extremity_length, extremity_thickness, measurement_error_pos, measurement_error_vel, delay, s_w, s_v, initial_pos_var, initial_vel_var);
    } else {
      return new HumanReach(n_joints_meas, joint_names, body_link_joints, thickness, max_v, max_a, extremity_base_names, extremity_end_names, extremity_length, extremity_thickness, measurement_error_pos, measurement_error_vel, delay);
    }
  }
}

HumanReach::HumanReach(int n_joints_meas,
      std::map<std::string, int> joint_names,
      std::map<std::string, reach_lib::jointPair>& body_link_joints, 
      const std::map<std::string, double>& thickness, 
      std::vector<double>& max_v, 
      std::vector<double>& max_a,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay):
  n_joints_meas_(n_joints_meas),
  body_link_joints_(body_link_joints),
  measurement_error_pos_(measurement_error_pos),
  measurement_error_vel_(measurement_error_vel),
  delay_(delay)
{
  reach_lib::System system(measurement_error_pos, measurement_error_vel, delay);
  human_models_.push_back(
    new reach_lib::ArticulatedCombined(
      system, body_link_joints, thickness, max_v, max_a
  ));
  for (int i = 0; i < n_joints_meas; i++) {
    joint_pos_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
    joint_vel_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
  }
}

HumanReach::HumanReach(int n_joints_meas,
      std::map<std::string, int> joint_names,
      std::map<std::string, reach_lib::jointPair>& body_link_joints, 
      const std::map<std::string, double>& thickness, 
      std::vector<double>& max_v, 
      std::vector<double>& max_a,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay,
      double s_w,
      double s_v,
      double initial_pos_var,
      double initial_vel_var):
      HumanReach(n_joints_meas, joint_names, body_link_joints, thickness, max_v, max_a, measurement_error_pos, measurement_error_vel, delay) {
  use_kalman_filter_ = true;
  measurement_handler_ = new MeasurementHandler(n_joints_meas, s_w, s_v, initial_pos_var, initial_vel_var);
}

HumanReach::HumanReach(int n_joints_meas,
      std::map<std::string, int> joint_names,
      std::map<std::string, reach_lib::jointPair>& body_link_joints, 
      const std::map<std::string, double>& thickness, 
      std::vector<double>& max_v, 
      std::vector<double>& max_a,
      std::vector<std::string>& extremity_base_names, 
      std::vector<std::string>& extremity_end_names, 
      std::vector<double>& extremity_length,
      std::vector<double>& extremity_thickness,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay):
  n_joints_meas_(n_joints_meas),
  body_link_joints_(body_link_joints),
  measurement_error_pos_(measurement_error_pos),
  measurement_error_vel_(measurement_error_vel),
  delay_(delay)
{
  reach_lib::System system(measurement_error_pos, measurement_error_vel, delay);
  // Create extremity map
  std::map<std::string, reach_lib::jointPair> extremity_body_segment_map;
  std::vector<double> extremity_max_v;
  for (int i = 0; i < extremity_base_names.size(); i++) {
    extremity_body_segment_map[extremity_base_names[i]] = reach_lib::jointPair(joint_names.at(extremity_base_names[i]), joint_names.at(extremity_end_names[i]));
    extremity_max_v.push_back(
      std::max(max_v.at(joint_names.at(extremity_base_names[i])), max_v.at(joint_names.at(extremity_end_names[i]))));
  }
  assert(extremity_base_names.size() == extremity_end_names.size());
  human_models_.push_back(new reach_lib::ArticulatedPos(
    system, extremity_body_segment_map, extremity_thickness,
    extremity_max_v, extremity_length
  ));
  human_models_.push_back(new reach_lib::ArticulatedVel(
    system, body_link_joints, thickness, max_v
  ));
  human_models_.push_back(new reach_lib::ArticulatedAccel(
    system, body_link_joints, thickness, max_a
  ));
  
  for (int i = 0; i < n_joints_meas; i++) {
    joint_pos_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
    joint_vel_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
  }
}

HumanReach::HumanReach(int n_joints_meas,
      std::map<std::string, int> joint_names,
      std::map<std::string, reach_lib::jointPair>& body_link_joints, 
      const std::map<std::string, double>& thickness, 
      std::vector<double>& max_v, 
      std::vector<double>& max_a,
      std::vector<std::string>& extremity_base_names, 
      std::vector<std::string>& extremity_end_names, 
      std::vector<double>& extremity_length,
      std::vector<double>& extremity_thickness,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay,
      double s_w,
      double s_v,
      double initial_pos_var,
      double initial_vel_var):
      HumanReach(n_joints_meas, joint_names, body_link_joints, thickness, max_v, max_a, extremity_base_names, extremity_end_names, extremity_length, extremity_thickness, measurement_error_pos, measurement_error_vel, delay) {
  use_kalman_filter_ = true;
  measurement_handler_ = new MeasurementHandler(n_joints_meas, s_w, s_v, initial_pos_var, initial_vel_var);
}

void HumanReach::reset() {
  last_meas_timestep_ = -1;
  for (int i = 0; i < n_joints_meas_; i++) {
    joint_pos_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
    joint_vel_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
  }
}

void HumanReach::measurement(const std::vector<reach_lib::Point>& human_joint_pos, double time) {
  try {
    if (use_kalman_filter_) {
      // Filter measurements
      Observation filtered_measurements = measurement_handler_->filterMeasurements(human_joint_pos, time);
      joint_pos_ = filtered_measurements.position;
      joint_vel_ = filtered_measurements.velocity;
      // TODO make vector
      measurement_error_pos_ = 0;
      for (int i = 0; i < filtered_measurements.pos_variance.size(); i++) {
        measurement_error_pos_ += std::sqrt(filtered_measurements.pos_variance[i]);
      }
      // Three sigma rule
      measurement_error_pos_ = 3.0 * measurement_error_pos_ / filtered_measurements.pos_variance.size();
      last_meas_timestep_ = time;
      return;
    } else {
      if (last_meas_timestep_ != -1) {
        double dt = time - last_meas_timestep_;
        if (dt < 1e-7) {
          safety_shield_logger::warn("HumanReach::measurement: dt is too small. dt = {}", dt);
          joint_pos_ = human_joint_pos;
          last_meas_timestep_ = time;
          return;
        }
        for (int i = 0; i < human_joint_pos.size(); i++) {
          // If more than 1 measurement, calculate velocity
          joint_vel_[i] = (human_joint_pos[i] - joint_pos_[i]) * (1 / dt);
        }
        has_second_meas_ = true;
      }
      joint_pos_ = human_joint_pos;
      last_meas_timestep_ = time;
    }
  } catch (const std::exception& exc) {
    safety_shield_logger::error("Exception in HumanReach::measurement: {}", exc.what());
  }
}

void HumanReach::humanReachabilityAnalysis(double t_command, double t_brake) {
  try {
    // Time between reach command msg and last measurement plus the t_brake time.
    double t_reach = t_command - last_meas_timestep_ + t_brake;
    for (auto& model : human_models_) {
      std::string type = model->get_mode();
      if (type == "ARTICULATED-POS") {
        static_cast<reach_lib::ArticulatedPos*>(model)->update(0.0, t_reach, joint_pos_, joint_vel_);
      } else if (type == "ARTICULATED-VEL") {
        static_cast<reach_lib::ArticulatedVel*>(model)->update(0.0, t_reach, joint_pos_, joint_vel_);
      } else if (type == "ARTICULATED-ACCEL") {
        static_cast<reach_lib::ArticulatedAccel*>(model)->update(0.0, t_reach, joint_pos_, joint_vel_);
      } else if (type == "ARTICULATED-COMBINED") {
        static_cast<reach_lib::ArticulatedCombined*>(model)->update(0.0, t_reach, joint_pos_, joint_vel_);
      } else {
        throw std::runtime_error("HumanReach::humanReachabilityAnalysis: Unknown model type: " + type);
      }
    }
  } catch (const std::exception& exc) {
    safety_shield_logger::error("Exception in HumanReach::humanReachabilityAnalysis: {}", exc.what());
  }
}

}  // namespace safety_shield
