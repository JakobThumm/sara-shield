#include <algorithm>
#include <chrono>
#include <ctime>
#include <functional>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "reach_lib.hpp"
#include "safety_shield/controller.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

std::function<std::string(const std::vector<double>&)> vecToStr2 = [](const std::vector<double>& v) {
  std::stringstream ss;
  ss.precision(4);
  std::copy(v.begin(), v.end(), std::ostream_iterator<double>(ss, v.size() > 1 ? ", " : ""));
  return ss.str();
};

int main() {
  double sample_time = 0.001;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/human_reach_schunk_setup.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::SSM;

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, shield_type);

  // Dummy human measurement
  int measurments = 9;
  std::vector<reach_lib::Point> dummy_human_meas(measurments);
  for (int i = 0; i < measurments; i++) {
    dummy_human_meas[i] = reach_lib::Point(10.0, 10.0, 10.0);
  }
  if (true) {
    safety_shield_logger::info("Debug started.");
    std::vector<double> qpos;
    std::vector<double> qvel;
    auto start = std::chrono::system_clock::now();
    double t = 0.0;
    for (int ep = 0; ep < 1; ep++) {
      t = 0.0;
      for (int i = 0; i < 10000; i++) {  // i < 100; i<10000
        t += 0.001;
        shield.humanMeasurement(dummy_human_meas, t);
        t += 0.003;

        if (i % 10 == 0) {  // % 2
          qpos = {0.0, 0.05*t, 0.0, 0.0, 0.0, 0.0};
          qvel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          shield.newLongTermTrajectory(qpos, qvel);
        }
        
        safety_shield::Motion next_motion = shield.step(t);
        std::vector<double> current_qpos = next_motion.getAngle();
        safety_shield_logger::info("Current time {}", t);
        safety_shield_logger::info("Goal qpos ({}, {}, {}, {}, {}, {})", qpos[0], qpos[1], qpos[2], qpos[3], qpos[4], qpos[5]);
        safety_shield_logger::info("Current qpos ({}, {}, {}, {}, {}, {})", current_qpos[0], current_qpos[1], current_qpos[2], current_qpos[3], current_qpos[4], current_qpos[5]);

        // safety_shield_logger::info("finished step");
      }
      shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, shield_type);
    }

    // Some computation here
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    safety_shield_logger::info("Debug finished, elapsed time: {}", elapsed_seconds.count());
  }
  return 0;
}
