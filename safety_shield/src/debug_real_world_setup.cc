#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>

#include "reach_lib.hpp"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

std::vector<reach_lib::Point> get_human_measurement_far() {
  std::vector<reach_lib::Point> dummy_human_meas(9);
  dummy_human_meas[0] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[1] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[2] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[3] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[4] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[5] =  reach_lib::Point(0.0, 0.0, 2.0);
  dummy_human_meas[6] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[7] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[8] =  reach_lib::Point(10.0, 10.0, 10.0);
  return dummy_human_meas;
}

std::vector<reach_lib::Point> get_human_measurement_close() {
  std::vector<reach_lib::Point> dummy_human_meas(9);
  dummy_human_meas[0] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[1] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[2] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[3] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[4] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[5] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[6] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[7] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[8] =  reach_lib::Point(0.0, 0.0, 0.0);
  return dummy_human_meas;
}

int main() {
  double sample_time = 0.004;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/human_reach_TUM_lab.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.912;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = -M_PI_2;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  reach_lib::AABB table = reach_lib::AABB({-0.75, -1.0, 0.82 - 0.05}, {0.75, 1.0, 0.82});
  std::vector<reach_lib::AABB> environment_elements = {table};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::PFL;

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, environment_elements, shield_type);

  std::vector<std::vector<double>> desired_goals = {
    {1.0, 0.0, 0.0, 0.3, 0.5, 1.0},
    {1.0, 1.4, 0.0, -0.6, 0.3, 0.0},
    {0.0, 0.5, -0.9, 0.0, 0.0, 0.0},
    {0.0, 1.4, 0.0, 0.0, 0.5, 0.0},
    {-1.4, 1.4, 0.0, 0.5, 0.0, 0.5},
    {-2.0, 0.0, 0.0, 1.0, 0.0, 1.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };
  auto human_measurement_close = get_human_measurement_close();
  auto human_measurement_far = get_human_measurement_far();

  // n = shield_frequency / RL_frequency = 5 / 250 = 1 / 50
  spdlog::info("Debug started.");
  int goal_number = 0;
  std::vector<double> goal = desired_goals[goal_number];
  shield.newLongTermTrajectory(goal, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  bool episode_finished = false;
  bool use_close_measurement = false;
  double t = 0.0;
  int human_counter = 0;
  clock_t clkStart;
  clock_t clkFinish;
  std::ofstream myfile;
  myfile.open("step_times.csv");
  
  for (int ep = 0; ep < 4; ep++) {
    episode_finished = false;
    while (!episode_finished) {
      t += sample_time/4.0;
      shield.humanMeasurement(human_measurement_far, t);
      t += 3.0/4.0 * sample_time;
      clkStart = clock();
      safety_shield::Motion next_motion = shield.step(t);
      clkFinish = clock();
      myfile << clkFinish - clkStart << ", ";
      safety_shield::Motion* goal_motion = new safety_shield::Motion(0.0, desired_goals[goal_number]);
      if (next_motion.hasSamePos(goal_motion, 0.03)) {
        goal_number++;
        if (goal_number == desired_goals.size()) {
          episode_finished = true;
          shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, environment_elements, shield_type);
          goal_number = 0;
        }
        std::vector<double> goal = desired_goals[goal_number];
        shield.newLongTermTrajectory(goal, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      }
      delete goal_motion;
    }
    spdlog::info("Episode finished.");
  }
  myfile.close();
  spdlog::info("Debug finished.");
  return 0;
}
