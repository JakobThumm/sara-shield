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

std::vector<reach_lib::Point> human_measurement_scc_scenario() {
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] =  reach_lib::Point(2 + 0.9677, 0.2786, 0.5853);
  dummy_human_meas[1] =  reach_lib::Point(2 + 0.8305, 0.2761, 0.5861);
  dummy_human_meas[2] =  reach_lib::Point(2 + 0.8957, 0.2985, 0.7856);
  dummy_human_meas[3] =  reach_lib::Point(2 + 1.02149425, 0.27962167, 0.21236983);
  dummy_human_meas[4] =  reach_lib::Point(2 + 0.77515377, 0.29120222, 0.20588813);
  dummy_human_meas[5] =  reach_lib::Point(2 + 0.90120311, 0.29739679, 0.92079985);
  dummy_human_meas[6] =  reach_lib::Point(2 + 0.98454192,  0.32560526, -0.18388601);
  dummy_human_meas[7] =  reach_lib::Point(2 + 0.81444863,  0.32443212, -0.19174635);
  dummy_human_meas[8] =  reach_lib::Point(2 + 0.90260337, 0.27198951, 0.97369635);
  dummy_human_meas[9] =  reach_lib::Point(2 + 1.01339635,  0.20327359, -0.23127244);
  dummy_human_meas[10] = reach_lib::Point(2 + 0.78264095,  0.19852595, -0.22824111);
  dummy_human_meas[11] = reach_lib::Point(2 + 0.899801  , 0.31482572, 1.18758907);
  dummy_human_meas[12] = reach_lib::Point(2 + 0.98150202, 0.3060102 , 1.09549145);
  dummy_human_meas[13] = reach_lib::Point(2 + 0.82090204, 0.31060957, 1.09258892);
  dummy_human_meas[14] = reach_lib::Point(2 + 0.90500115, 0.26352428, 1.25258792);
  dummy_human_meas[15] = reach_lib::Point(2 + 1.07240476, 0.31491036, 1.12598324);
  dummy_human_meas[16] = reach_lib::Point(2 + 0.72479888, 0.31970966, 1.12507954);
  dummy_human_meas[17] = reach_lib::Point(2 + 1.33200275, 0.34240919, 1.11314008);
  dummy_human_meas[18] = reach_lib::Point(2 + 0.47110089, 0.34110864, 1.11173956);
  dummy_human_meas[19] = reach_lib::Point(2 + 1.5813025 , 0.34350919, 1.12214688);
  dummy_human_meas[20] = reach_lib::Point(2 + 0.21580109, 0.3467087 , 1.1195459 );
  dummy_human_meas[21] = reach_lib::Point(2 + 1.66530385, 0.35850957, 1.11396142);
  dummy_human_meas[22] = reach_lib::Point(2 + 0.13120006, 0.35700906, 1.1133605);
  return dummy_human_meas;
}

int main() {
  double sample_time = 0.004;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/mujoco_mocap.yaml");
  double init_x = 2.0;
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
    {1.5, 1.5, -M_PI, 0, -M_PI, 0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };
  auto human_measurement = human_measurement_scc_scenario();

  // n = shield_frequency / RL_frequency = 5 / 250 = 1 / 50
  spdlog::info("Debug started.");
  double t = 0.0;
  double t_max = 10.0;
  double max_iter = (int) t_max / sample_time;
  for (int ep = 0; ep < 1; ep++) {
    for (int i = 0; i < max_iter; i++) {
      t += sample_time/4.0;
      shield.humanMeasurement(human_measurement, t);
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
