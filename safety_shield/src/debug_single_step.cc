#include <string>
#include <vector>

#include "reach_lib.hpp"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

int main() {
  double sample_time = 0.004;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/cmu_mocap_no_hand.yaml");
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
  std::vector<reach_lib::Point> dummy_human_meas_safe(21);
  std::vector<reach_lib::Point> dummy_human_meas_unsafe(21);
  for (int i = 0; i < 21; i++) {
    dummy_human_meas_safe[i] = reach_lib::Point(10.0, 10.0, 0.0);
    dummy_human_meas_unsafe[i] = reach_lib::Point(0.0, 0.0, 0.0);
  }

  spdlog::info("Debug started.");
  double t = 0.0;
  for (int ep = 0; ep < 5; ep++) {
    spdlog::info("Next motion q[0], dq[0], ddq[0], dddq[0]");
    for (int i = 0; i < 100; i++) {  // i < 100; i<10000
      t += 0.001;
      if (i < 10) {
        shield.humanMeasurement(dummy_human_meas_safe, t);
      } else {
        shield.humanMeasurement(dummy_human_meas_unsafe, t);
        shield.humanMeasurement(dummy_human_meas_unsafe, t + 0.000001);
      }
      t += 0.003;
      if (i % 10 == 0) {  // % 2
        std::vector<double> qpos{1.0, 1.5, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        shield.newLongTermTrajectory(qpos, qvel);
        spdlog::info("new LTT");
      }
      safety_shield::Motion next_motion = shield.step(t);
      spdlog::info("{}, {}, {}, {}", next_motion.getAngle()[0], next_motion.getVelocity()[0],
                   next_motion.getAcceleration()[0], next_motion.getJerk()[0]);
      // spdlog::info("finished step");
    }
    shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, shield_type);
  }
  spdlog::info("Debug finished.");
  return 0;
}