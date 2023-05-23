#include <string>
#include <vector>

#include "reach_lib.hpp"


#include "safety_shield/safety_shield.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"

// debug file for PFL features
int main () {
    bool activate_shield = true;
    double sample_time = 0.001; 
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

    safety_shield::SafetyShield shield = safety_shield::SafetyShield(activate_shield,
      sample_time, 
      trajectory_config_file,
      robot_config_file,
      mocap_config_file,
      init_x, 
      init_y, 
      init_z, 
      init_roll, 
      init_pitch, 
      init_yaw,
      init_qpos,
      safety_shield::SafetyShield::Safety_method::STP_MAXIMUM, // "STP_MAXIMUM"
      safety_shield::RobotReach::Velocity_method::APPROXIMATE); // true


    // Dummy human measurement
    std::vector<reach_lib::Point> dummy_human_meas(21);
    for (int i=0; i<21; i++) {
      dummy_human_meas[i] = reach_lib::Point(10.0, 10.0, 0.0);
    }

    spdlog::info("Debug started.");
    double t = 0.0;
    for (int ep=0; ep<1; ep++) {
      for (int i=0; i<10000; i++) { //i < 100; i<10000
        t += 0.001;
        shield.humanMeasurement(dummy_human_meas, t);
        t += 0.003;
        if (i % 50 == 0) { // % 2
            std::vector<double> qpos{0.2*t, t, t, t, t, std::min(t, 3.1)};//qpos{0.2*t, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
            std::vector<double> qvel{t+100, t+100, t+100, t+100, t+100, t+100};//{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            shield.newLongTermTrajectory(qpos, qvel);
            //spdlog::info("new LTT");
        }
        safety_shield::Motion next_motion = shield.step(t);
        //spdlog::info("finished step");
      }
      /*
      auto list = shield.getVelocityScaling();
      for(int i = 0; i < list.size(); i++) {
          std::cout << list[i] << std::endl;
      }
       */
      shield.reset(true, init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t);
    }
    spdlog::info("Debug finished.");
    return 0;
}