#include "safety_shield/config_utils.h"

namespace safety_shield {

RobotReach* buildRobotReach(
  std::string robot_config_file,
  double init_x,
  double init_y,
  double init_z,
  double init_roll,
  double init_pitch,
  double init_yaw
) {
  YAML::Node robot_config = YAML::LoadFile(robot_config_file);
  std::string robot_name = robot_config["robot_name"].as<std::string>();
  int nb_joints = robot_config["nb_joints"].as<int>();
  std::vector<double> transformation_matrices = robot_config["transformation_matrices"].as<std::vector<double>>();
  std::vector<double> enclosures = robot_config["enclosures"].as<std::vector<double>>();
  double secure_radius = robot_config["secure_radius"].as<double>();
  std::vector<double> link_masses;
  if (robot_config["link_masses"]) {
    link_masses = robot_config["link_masses"].as<std::vector<double>>();
  }
  std::vector<double> inertia_matrices;
  if (robot_config["link_inertias"]) {
    inertia_matrices = robot_config["link_inertias"].as<std::vector<double>>();
  }
  std::vector<double> center_of_masses;
  if (robot_config["link_centers_of_mass"]) {
    center_of_masses = robot_config["link_centers_of_mass"].as<std::vector<double>>();
  }
  return new RobotReach(transformation_matrices, nb_joints, enclosures, 
                        link_masses, inertia_matrices, center_of_masses,
                        init_x, init_y, init_z, init_roll, init_pitch, init_yaw, secure_radius);
}

void readTrajectoryConfig(
  std::string trajectory_config_file,
  double& max_s_stop,
  std::vector<double>& q_min_allowed,
  std::vector<double>& q_max_allowed,
  std::vector<double>& v_max_allowed,
  std::vector<double>& a_max_allowed,
  std::vector<double>& j_max_allowed,
  std::vector<double>& a_max_ltt,
  std::vector<double>& j_max_ltt,
  double& v_safe,
  double& alpha_i_max,
  int& velocity_method,
  int& reachability_set_interval_size
) {
  YAML::Node trajectory_config = YAML::LoadFile(trajectory_config_file);
  max_s_stop = trajectory_config["max_s_stop"].as<double>();
  q_min_allowed = trajectory_config["q_min_allowed"].as<std::vector<double>>();
  q_max_allowed = trajectory_config["q_max_allowed"].as<std::vector<double>>();
  v_max_allowed = trajectory_config["v_max_allowed"].as<std::vector<double>>();
  a_max_allowed = trajectory_config["a_max_allowed"].as<std::vector<double>>();
  j_max_allowed = trajectory_config["j_max_allowed"].as<std::vector<double>>();
  a_max_ltt = trajectory_config["a_max_ltt"].as<std::vector<double>>();
  j_max_ltt = trajectory_config["j_max_ltt"].as<std::vector<double>>();
  v_safe = trajectory_config["v_safe"].as<double>();
  alpha_i_max = trajectory_config["alpha_i_max"].as<double>();
  velocity_method = trajectory_config["velocity_method"].as<int>();
  if(trajectory_config["reachability_set_interval_size"]) {
    reachability_set_interval_size = trajectory_config["reachability_set_interval_size"].as<double>();
  } else {
    reachability_set_interval_size = 10000;
  }
}

HumanReach* buildHumanReach(
  std::string human_config_file
) {
  YAML::Node human_config = YAML::LoadFile(human_config_file);
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
  std::map<std::string, double> max_contact_energy;
  for (YAML::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
    const YAML::Node& body = *it;
    body_link_joints[body["name"].as<std::string>()] = reach_lib::jointPair(
        joint_names[body["proximal"].as<std::string>()], joint_names[body["distal"].as<std::string>()]);
    thickness[body["name"].as<std::string>()] = body["thickness"].as<double>();
    max_contact_energy[body["name"].as<std::string>()] = body["max_contact_energy_unconstrained"].as<double>();
  }

  bool use_combined_model = human_config["use_combined_model"].as<bool>();
  std::vector<std::string> extremity_base_names, extremity_end_names;
  std::vector<double> extremity_length, extremity_thickness, extremity_max_contact_energy;
  if (!use_combined_model) {
    // Build extremities
    const YAML::Node& extremities = human_config["extremities"];
    for (YAML::const_iterator it = extremities.begin(); it != extremities.end(); ++it) {
      const YAML::Node& extremity = *it;
      extremity_base_names.push_back(extremity["base"].as<std::string>());
      extremity_end_names.push_back(extremity["end"].as<std::string>());
      extremity_length.push_back(extremity["length"].as<double>());
      extremity_thickness.push_back(extremity["thickness"].as<double>());
      extremity_max_contact_energy.push_back(extremity["max_contact_energy_unconstrained"].as<double>());
    }
  }

  bool use_kalman_filter = human_config["use_kalman_filter"].as<bool>();
  double s_w, s_v, initial_pos_var, initial_vel_var;
  if (use_kalman_filter) {
    s_w = human_config["s_w"].as<double>();
    s_v = human_config["s_v"].as<double>();
    initial_pos_var = human_config["initial_pos_var"].as<double>();
    initial_vel_var = human_config["initial_vel_var"].as<double>();
  }
  return createHumanReach(
    use_combined_model,
    use_kalman_filter,
    joint_names.size(),
    joint_names,
    body_link_joints, 
    thickness, 
    max_contact_energy,
    joint_v_max, 
    joint_a_max,
    measurement_error_pos, 
    measurement_error_vel, 
    delay,
    extremity_base_names, 
    extremity_end_names, 
    extremity_length,
    extremity_thickness,
    extremity_max_contact_energy,
    s_w,
    s_v,
    initial_pos_var,
    initial_vel_var
  );
}
} // namespace safety_shield