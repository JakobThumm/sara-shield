#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "reach_lib.hpp"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

std::vector<reach_lib::Point> human_measurement_far_scenario() {
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[1] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[2] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[3] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[4] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[5] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[6] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[7] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[8] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[9] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[10] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[11] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[12] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[13] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[14] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[15] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[16] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[17] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[18] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[19] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[20] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[21] = reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[22] = reach_lib::Point(10.0, 10.0, 10.0);
  return dummy_human_meas;
}

std::vector<reach_lib::Point> human_measurement_close_scenario() {
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[1] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[2] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[3] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[4] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[5] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[6] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[7] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[8] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[9] =  reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[10] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[11] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[12] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[13] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[14] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[15] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[16] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[17] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[18] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[19] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[20] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[21] = reach_lib::Point(0.0, 0.5, 0.0);
  dummy_human_meas[22] = reach_lib::Point(0.0, 0.5, 0.0);
  return dummy_human_meas;
}

int main() {
  double sample_time = 0.02;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/mujoco_mocap.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.912;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  reach_lib::AABB table = reach_lib::AABB({-0.75, -1.0, 0.82 - 0.05}, {0.75, 1.0, 0.82});
  std::vector<reach_lib::AABB> environment_elements = {table};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::PFL;

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, environment_elements, shield_type);

  std::vector<std::vector<double>> desired_goals = {
    {0.0, 1.5, -M_PI / 2 + 1.0, 0, -M_PI / 2, 0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };
  auto human_measurement_far = human_measurement_far_scenario();
  auto human_measurement_close = human_measurement_close_scenario();

  // n = shield_frequency / RL_frequency = 5 / 250 = 1 / 50
  spdlog::info("Debug started.");
  double t = 0.0;
  double t_max = 10.0;
  double max_iter = (int) t_max / sample_time;
  for (int ep = 0; ep < 10; ep++) {
    for (int i = 0; i < max_iter; i++) {
      t += sample_time/4.0;
      if (i % 4 == 0) {
        shield.humanMeasurement(human_measurement_close, t);
      } else {
        shield.humanMeasurement(human_measurement_far, t);
      }
      t += 3.0/4.0 * sample_time;
      if (i % ((int) max_iter / desired_goals.size()) == 0) {
        int goal_index = (int) i * desired_goals.size() / max_iter;
        std::vector<double> goal = desired_goals[goal_index];
        shield.newLongTermTrajectory(goal, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      }
      safety_shield::Motion next_motion = shield.step(t);
    }
    shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, environment_elements, shield_type);
  }
  spdlog::info("Debug finished.");
  return 0;
}
