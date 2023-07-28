#include <string>
#include <vector>

#include "reach_lib.hpp"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

std::vector<reach_lib::Point> human_measurement_head_scenario() {
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] = reach_lib::Point(1.0786, -0.0677,  0.3353);
  dummy_human_meas[1] = reach_lib::Point(1.0761, 0.0695, 0.3361);
  dummy_human_meas[2] = reach_lib::Point(1.0985, 0.0043, 0.5356);
  dummy_human_meas[3] = reach_lib::Point(1.08362757, -0.10484137, -0.03962269);
  dummy_human_meas[4] = reach_lib::Point(1.08822514,  0.10909169, -0.04617978);
  dummy_human_meas[5] = reach_lib::Point(1.09739433, -0.0012076 ,  0.67079964);
  dummy_human_meas[6] = reach_lib::Point(1.11107663, -0.08514985, -0.43881868);
  dummy_human_meas[7] = reach_lib::Point(1.10772773,  0.0932326 , -0.44634178);
  dummy_human_meas[8] = reach_lib::Point(1.07197979, -0.00260809,  0.72369264);
  dummy_human_meas[9] = reach_lib::Point(0.98783743, -0.11384181, -0.48389596);
  dummy_human_meas[10] = reach_lib::Point(0.980046  ,  0.1200125 , -0.48063716);
  dummy_human_meas[11] = reach_lib::Point(1.11485195e+00, 1.97000753e-04, 9.37578125e-01);
  dummy_human_meas[12] = reach_lib::Point(1.10602106, -0.08150517,  0.84548301);
  dummy_human_meas[13] = reach_lib::Point(1.11061972, 0.07909477, 0.84257771);
  dummy_human_meas[14] = reach_lib::Point(1.06354908, -0.00500331,  1.00257583);
  dummy_human_meas[15] = reach_lib::Point(1.11492138, -0.17241065,  0.87596658);
  dummy_human_meas[16] = reach_lib::Point(1.11971989, 0.17520109, 0.87505897);
  dummy_human_meas[17] = reach_lib::Point(1.14241904, -0.43200663,  0.86308028);
  dummy_human_meas[18] = reach_lib::Point(1.14111783, 0.42889707, 0.8616792);
  dummy_human_meas[19] = reach_lib::Point(1.14351909, -0.68130614,  0.87209383);
  dummy_human_meas[20] = reach_lib::Point(1.14671798, 0.68419668, 0.86949194);
  dummy_human_meas[21] = reach_lib::Point(1.15851987, -0.76530882,  0.86392285);
  dummy_human_meas[22] = reach_lib::Point(1.15701874, 0.76879873, 0.86332127);
  return dummy_human_meas;
}

std::vector<reach_lib::Point> human_measurement_non_head_scenario() {
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] = reach_lib::Point(1.1286, -0.0677,  0.9853);
  dummy_human_meas[1] = reach_lib::Point(1.1261, 0.0695, 0.9861);
  dummy_human_meas[2] = reach_lib::Point(1.1485, 0.0043, 1.1856);
  dummy_human_meas[3] = reach_lib::Point(1.13309146, -0.10200292,  0.61010016);
  dummy_human_meas[4] = reach_lib::Point(1.13498742, 0.10780121, 0.60359983);
  dummy_human_meas[5] = reach_lib::Point(1.14739433e+00, -1.20759992e-03,  1.32079964e+00);
  dummy_human_meas[6] = reach_lib::Point(1.17676921, -0.08844074,  0.21209643);
  dummy_human_meas[7] = reach_lib::Point(1.1772661 , 0.09204135, 0.20519597);
  dummy_human_meas[8] = reach_lib::Point(1.12197976, -0.00260809,  1.37369264);
  dummy_human_meas[9] = reach_lib::Point(1.05747168, -0.11484298,  0.15629222);
  dummy_human_meas[10] = reach_lib::Point(1.05386821, 0.11744328, 0.15699158);
  dummy_human_meas[11] = reach_lib::Point(1.16485202e+00, 1.96998991e-04, 1.58757810e+00);
  dummy_human_meas[12] = reach_lib::Point(1.15602108, -0.08150517,  1.49548298);
  dummy_human_meas[13] = reach_lib::Point(1.16061975, 0.07909477, 1.49257768);
  dummy_human_meas[14] = reach_lib::Point(1.11354915, -0.00500331,  1.65257581);
  dummy_human_meas[15] = reach_lib::Point(1.16492138, -0.17241066,  1.52596655);
  dummy_human_meas[16] = reach_lib::Point(1.16971991, 0.1752011 , 1.52505892);
  dummy_human_meas[17] = reach_lib::Point(1.19241899, -0.43200663,  1.51308012);
  dummy_human_meas[18] = reach_lib::Point(1.19111782, 0.42889707, 1.51167904);
  dummy_human_meas[19] = reach_lib::Point(1.19351899, -0.68130614,  1.52209368);
  dummy_human_meas[20] = reach_lib::Point(1.19671794, 0.68419668, 1.51949178);
  dummy_human_meas[21] = reach_lib::Point(1.20851975, -0.76530884,  1.51392274);
  dummy_human_meas[22] = reach_lib::Point(1.20701868, 0.76879874, 1.51332116);
  return dummy_human_meas;
}

int main() {
  double sample_time = 0.001;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/cmu_mocap_no_hand.yaml"); //std::string("../config/mujoco_mocap.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::SEVERAL_PFL;

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, shield_type);


  spdlog::info("Debug started.");
  double t = 0.0;
  for (int ep = 0; ep < 5; ep++) {
    for (int i = 0; i < 100; i++) {  // i < 100; i<10000
      t += 0.001;
      shield.humanMeasurement(human_measurement_head_scenario(), t);
      t += 0.003;
      // TODO: goal - pos von hrgym einbauen
      if (i % 10 == 0) {  // % 2
        std::vector<double> qpos{0.2 * t, t, t,
                                 t,       t, std::min(t, 3.1)};  // qpos{0.2*t, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
        std::vector<double> qvel{t + 100, t + 100, t + 100,
                                 t + 100, t + 100, t + 100};  //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        shield.newLongTermTrajectory(qpos, qvel);
        // spdlog::info("new LTT");
      }
      safety_shield::Motion next_motion = shield.step(t);
      // spdlog::info("finished step");
    }
    shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, shield_type);
  }
  spdlog::info("Debug finished.");
  return 0;
}