// config_parser.h
#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <iostream>
#include <map>
#include <string>
#include <vector>

#ifdef USE_HEADER_CONFIG
#include "config_safety_shield.h"
#else
#include <yaml-cpp/yaml.h>
#endif

namespace safety_shield {

class ConfigParser {
 public:
  ConfigParser() = default;

#ifdef USE_HEADER_CONFIG
  // config files are never used
  void loadConfig(const std::string& trajectory_config_file, const std::string& robot_config_file,
                  const std::string& mocap_config_file) {
    using namespace config_schunk;

    // Load robot config
    robot_config.robot_name = robot_name;
    robot_config.nb_joints = nb_joints;
    robot_config.transformation_matrices = std::vector<double>(transformation_matrices.begin(), transformation_matrices.end());
    robot_config.enclosures = std::vector<double>(enclosures.begin(), enclosures.end());
    robot_config.secure_radius = secure_radius;

    // Load trajectory config
    trajectory_config.max_s_stop = max_s_stop_;
    trajectory_config.q_min_allowed = std::vector<double>(q_min_allowed.begin(), q_min_allowed.end());
    trajectory_config.q_max_allowed = std::vector<double>(q_max_allowed.begin(), q_max_allowed.end());
    trajectory_config.v_max_allowed = std::vector<double>(v_max_allowed.begin(), v_max_allowed.end());
    trajectory_config.a_max_allowed = std::vector<double>(a_max_allowed.begin(), a_max_allowed.end());
    trajectory_config.j_max_allowed = std::vector<double>(j_max_allowed.begin(), j_max_allowed.end());
    trajectory_config.a_max_ltt = std::vector<double>(a_max_ltt.begin(), a_max_ltt.end());
    trajectory_config.j_max_ltt = std::vector<double>(j_max_ltt.begin(), j_max_ltt.end());
    trajectory_config.v_safe = v_safe;
    trajectory_config.alpha_i_max = alpha_i_max;
    trajectory_config.velocity_method = velocity_method;

    // Load human config
    human_config.measurement_error_pos = measurement_error_pos;
    human_config.measurement_error_vel = measurement_error_vel;
    human_config.delay = delay;
    human_config.joint_names = std::vector<std::string>(joint_names.begin(), joint_names.end());
    human_config.joint_v_max = std::vector<double>(joint_v_max.begin(), joint_v_max.end());
    human_config.joint_a_max = std::vector<double>(joint_a_max.begin(), joint_a_max.end());
    human_config.body_parts = std::vector<std::string>(body_parts.begin(), body_parts.end());
    human_config.body_part_proximal = std::vector<std::string>(body_part_proximal.begin(), body_part_proximal.end());
    human_config.body_part_distal = std::vector<std::string>(body_part_distal.begin(), body_part_distal.end());
    human_config.joint_thickness = std::vector<double>(joint_thickness.begin(), joint_thickness.end());
    human_config.extremity_base_names = std::vector<std::string>(extremities_base.begin(), extremities_base.end());
    human_config.extremity_end_names = std::vector<std::string>(extremities_end.begin(), extremities_end.end());
    human_config.extremity_length = std::vector<double>(extremities_length.begin(), extremities_length.end());
    human_config.extremity_thickness = std::vector<double>(extremities_thickness.begin(), extremities_thickness.end());
    human_config.use_combined_model = use_combined_model;
    human_config.use_kalman_filter = use_kalman_filter;
    human_config.s_w = s_w;
    human_config.s_v = s_v;
    human_config.initial_pos_var = initial_pos_var;
    human_config.initial_vel_var = initial_vel_var;
  }
#else
  void loadConfig(const std::string& trajectory_config_file, const std::string& robot_config_file,
                  const std::string& mocap_config_file) {
    loadTrajectoryConfig(trajectory_config_file);
    loadRobotConfig(robot_config_file);
    loadMocapConfig(mocap_config_file);
  }
#endif

  // Structs to store the configuration data
  struct RobotConfig {
    std::string robot_name;
    int nb_joints;
    std::vector<double> transformation_matrices;
    std::vector<double> enclosures;
    double secure_radius;
  } robot_config;

  struct TrajectoryConfig {
    double max_s_stop;
    std::vector<double> q_min_allowed;
    std::vector<double> q_max_allowed;
    std::vector<double> v_max_allowed;
    std::vector<double> a_max_allowed;
    std::vector<double> j_max_allowed;
    std::vector<double> a_max_ltt;
    std::vector<double> j_max_ltt;
    double v_safe;
    double alpha_i_max;
    int velocity_method;
  } trajectory_config;

  struct HumanConfig {
    double measurement_error_pos;
    double measurement_error_vel;
    double delay;
    std::vector<std::string> joint_names;
    std::vector<double> joint_v_max;
    std::vector<double> joint_a_max;
    std::vector<std::string> body_parts;
    std::vector<std::string> body_part_proximal;
    std::vector<std::string> body_part_distal;
    std::vector<double> joint_thickness;
    std::vector<std::string> extremity_base_names;
    std::vector<std::string> extremity_end_names;
    std::vector<double> extremity_length;
    std::vector<double> extremity_thickness;
    bool use_combined_model;
    bool use_kalman_filter;
    double s_w;
    double s_v;
    double initial_pos_var;
    double initial_vel_var;
  } human_config;

  std::string toString() const {
    std::ostringstream oss;
    // Robot Config
    oss << "\n";
    oss << "Robot Config:\n\n";
    oss << "  Robot Name: " << robot_config.robot_name << "\n";
    oss << "  Number of Joints: " << robot_config.nb_joints << "\n";
    oss << "  Transformation Matrices:\n" << transformationMatrixToString(robot_config.transformation_matrices) << "\n";
    oss << "  Enclosures:\n" << enclosuresToString(robot_config.enclosures) << "\n";
    oss << "  Secure Radius: " << robot_config.secure_radius << "\n\n";

    // Trajectory Config
    oss << "Trajectory Config:\n\n";
    oss << "  Max S Stop: " << trajectory_config.max_s_stop << "\n";
    oss << "  Q Min Allowed: " << vectorToString(trajectory_config.q_min_allowed) << "\n";
    oss << "  Q Max Allowed: " << vectorToString(trajectory_config.q_max_allowed) << "\n";
    oss << "  V Max Allowed: " << vectorToString(trajectory_config.v_max_allowed) << "\n";
    oss << "  A Max Allowed: " << vectorToString(trajectory_config.a_max_allowed) << "\n";
    oss << "  J Max Allowed: " << vectorToString(trajectory_config.j_max_allowed) << "\n";
    oss << "  A Max LTT: " << vectorToString(trajectory_config.a_max_ltt) << "\n";
    oss << "  J Max LTT: " << vectorToString(trajectory_config.j_max_ltt) << "\n";
    oss << "  V Safe: " << trajectory_config.v_safe << "\n";
    oss << "  Alpha I Max: " << trajectory_config.alpha_i_max << "\n";
    oss << "  Velocity Method: " << trajectory_config.velocity_method << "\n\n";

    // Human Config
    oss << "Human Config:\n\n";
    oss << "  Measurement Error Pos: " << human_config.measurement_error_pos << "\n";
    oss << "  Measurement Error Vel: " << human_config.measurement_error_vel << "\n";
    oss << "  Delay: " << human_config.delay << "\n";
    oss << "  Joint Names: " << vectorToString(human_config.joint_names) << "\n";
    oss << "  Joint V Max: " << vectorToString(human_config.joint_v_max) << "\n";
    oss << "  Joint A Max: " << vectorToString(human_config.joint_a_max) << "\n";
    oss << "  Body Parts: " << vectorToString(human_config.body_parts) << "\n";
    oss << "  Body Part Proximal: " << vectorToString(human_config.body_part_proximal) << "\n";
    oss << "  Body Part Distal: " << vectorToString(human_config.body_part_distal) << "\n";
    oss << "  Joint Thickness: " << vectorToString(human_config.joint_thickness) << "\n";
    oss << "  Extremity Base Names: " << vectorToString(human_config.extremity_base_names) << "\n";
    oss << "  Extremity End Names: " << vectorToString(human_config.extremity_end_names) << "\n";
    oss << "  Extremity Length: " << vectorToString(human_config.extremity_length) << "\n";
    oss << "  Extremity Thickness: " << vectorToString(human_config.extremity_thickness) << "\n";
    oss << "  Use Combined Model: " << (human_config.use_combined_model ? "true" : "false") << "\n";
    oss << "  Use Kalman Filter: " << (human_config.use_kalman_filter ? "true" : "false") << "\n";
    oss << "  S W: " << human_config.s_w << "\n";
    oss << "  S V: " << human_config.s_v << "\n";
    oss << "  Initial Pos Var: " << human_config.initial_pos_var << "\n";
    oss << "  Initial Vel Var: " << human_config.initial_vel_var << "\n\n";

    return oss.str();
  }

 private:
#ifndef USE_HEADER_CONFIG
  void loadRobotConfig(const std::string& file) {
    YAML::Node config = YAML::LoadFile(file);
    robot_config.robot_name = config["robot_name"].as<std::string>();
    robot_config.nb_joints = config["nb_joints"].as<int>();
    robot_config.transformation_matrices = config["transformation_matrices"].as<std::vector<double>>();
    robot_config.enclosures = config["enclosures"].as<std::vector<double>>();
    robot_config.secure_radius = config["secure_radius"].as<double>();
  }

  void loadTrajectoryConfig(const std::string& file) {
    YAML::Node config = YAML::LoadFile(file);
    trajectory_config.max_s_stop = config["max_s_stop"].as<double>();
    trajectory_config.q_min_allowed = config["q_min_allowed"].as<std::vector<double>>();
    trajectory_config.q_max_allowed = config["q_max_allowed"].as<std::vector<double>>();
    trajectory_config.v_max_allowed = config["v_max_allowed"].as<std::vector<double>>();
    trajectory_config.a_max_allowed = config["a_max_allowed"].as<std::vector<double>>();
    trajectory_config.j_max_allowed = config["j_max_allowed"].as<std::vector<double>>();
    trajectory_config.a_max_ltt = config["a_max_ltt"].as<std::vector<double>>();
    trajectory_config.j_max_ltt = config["j_max_ltt"].as<std::vector<double>>();
    trajectory_config.v_safe = config["v_safe"].as<double>();
    trajectory_config.alpha_i_max = config["alpha_i_max"].as<double>();
    trajectory_config.velocity_method = config["velocity_method"].as<int>();
  }

  void loadMocapConfig(const std::string& file) {
    YAML::Node config = YAML::LoadFile(file);
    human_config.measurement_error_pos = config["measurement_error_pos"].as<double>();
    human_config.measurement_error_vel = config["measurement_error_vel"].as<double>();
    human_config.delay = config["delay"].as<double>();
    human_config.joint_names = config["joint_names"].as<std::vector<std::string>>();
    human_config.joint_v_max = config["joint_v_max"].as<std::vector<double>>();
    human_config.joint_a_max = config["joint_a_max"].as<std::vector<double>>();

    const YAML::Node& bodies = config["bodies"];
    for (const auto& body : bodies) {
      human_config.body_parts.push_back(body["name"].as<std::string>());
      human_config.body_part_proximal.push_back(body["proximal"].as<std::string>());
      human_config.body_part_distal.push_back(body["distal"].as<std::string>());
      human_config.joint_thickness.push_back(body["thickness"].as<double>());
    }

    const YAML::Node& extremities = config["extremities"];
    for (const auto& extremity : extremities) {
      human_config.extremity_base_names.push_back(extremity["base"].as<std::string>());
      human_config.extremity_end_names.push_back(extremity["end"].as<std::string>());
      human_config.extremity_length.push_back(extremity["length"].as<double>());
      human_config.extremity_thickness.push_back(extremity["thickness"].as<double>());
    }

    human_config.use_combined_model = config["use_combined_model"].as<bool>();
    human_config.use_kalman_filter = config["use_kalman_filter"].as<bool>();
    human_config.s_w = config["s_w"].as<double>();
    human_config.s_v = config["s_v"].as<double>();
    human_config.initial_pos_var = config["initial_pos_var"].as<double>();
    human_config.initial_vel_var = config["initial_vel_var"].as<double>();
  }
#endif

  template <typename T>
  std::string vectorToString(const std::vector<T>& vec) const {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      oss << vec[i];
      if (i != vec.size() - 1) {
        oss << ", ";
      }
    }
    oss << "]";
    return oss.str();
  }

  // Function to format a 4x4 matrix
  std::string transformationMatrixToString(const std::vector<double>& matrix) const {
      std::ostringstream oss;
      size_t index = 0;
      int num_matrices = matrix.size() / 16;
      for (size_t i = 0; i < num_matrices; ++i) {
          for (size_t row = 0; row < 4; ++row) {
              oss << "    ";
              for (size_t col = 0; col < 4; ++col) {
                  oss << matrix[index++] << " ";
              }
              oss << "\n";
          }
          oss << "\n";
      }
      return oss.str();
  }

  // Function to format the enclosures vector
  std::string enclosuresToString(const std::vector<double>& enclosures) const {
      std::ostringstream oss;
      size_t index = 0;
      
      // Define number of rows and columns for formatting
      const size_t num_rows = enclosures.size() / 7;  // Based on your data
      const size_t num_columns = 3;
      
      for (size_t row = 0; row < num_rows; ++row) {
          oss << "    ";
          for (size_t col = 0; col < num_columns; ++col) {
              oss << enclosures[index++] << " ";

          }
          oss << "\n";
          oss << "    "; 
          for (size_t col = 0; col < num_columns; ++col) {
              oss << enclosures[index++] << " ";
              
          }
          oss << "\n";
          oss << "    "; 
          oss << enclosures[index++] << " ";
          oss << "\n";
          oss << "\n";
      }
      
      return oss.str();
  }

};


}  // namespace safety_shield

#endif  // CONFIG_PARSER_H
