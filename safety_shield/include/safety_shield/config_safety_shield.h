#ifndef config_safety_shield_H
#define config_safety_shield_H

#include <string>
#include <vector>

namespace config_schunk{
	constexpr char* robot_name = "schunk";
	constexpr int nb_joints = 6;
	constexpr std::array<double, 96> transformation_matrices = {
        1, 0, 0, 0, 
        0, 1, 0, 0, 
        0, 0, 1, 0.054, 
        0, 0, 0, 1,

        1, 0, 0, 0, 
        0, -3.4915e-15, 1, 0.1013, 
        0, -1, -3.4915e-15, 0.1013, 
        0, 0, 0, 1, 

        1, 0, -3.6752e-15, -1.222e-15, 
        0, -1, 1.2832e-29, -0.35, 
        -3.6752e-15, -1.2832e-29, -1, 0, 
        0, 0, 0, 1,  

        1, 0, 0, 0, 
        0, -3.4915e-15, 1, 0.1013, 
        0, -1, -3.4915e-15, 0.1013, 
        0, 0, 0, 1,  

        1, -3.2311e-15, 3.2311e-15, -2.4169e-16, 
        3.2311e-15, -3.4915e-15, -1, 0.0748, 
        3.2311e-15, 1, -3.4915e-15, 0.1999, 
        0, 0, 0, 1,  

        1, 0, 0, 0, 
        0, -3.4915e-15, 1, 0.0748, 
        0, -1, -3.4915e-15, 0.0748, 
        0, 0, 0, 1};


	constexpr std::array<double, 42> enclosures = {
        
        0, -0.015, 0.1013,
        0, -0.015, 0.1013,
        0.09,

        0.00022952, -0.3499, 0.01358,
        -0.00066172, 9.4084e-05, 0.012745,
        0.074021,

        0, -0.015, 0.1013,
        0, -0.015, 0.1013,
        0.09,

        -4.6934e-05, -0.0058834, -0.00049515,
        -2.3657e-06, 0.061878, 0.19827,
        0.07,

        0, 0, 0.0748,
        0, 0, 0.0748,
        0.07,
        
        0.00049324, 0.0033497, 0.024003,
        -0.00049264, 0.0031812, 0.09,
        0.072549
                
	};
	constexpr double secure_radius = 0.02;
    
    constexpr double alpha_i_max = 5.0;
    constexpr double v_safe = 0.05;

    constexpr int velocity_method = 0;

	constexpr double max_s_stop_ = 0.2;
	constexpr std::array<double, 6> q_min_allowed = { -2.9, -1.8, -2.60, -2.9, -1.85, -2.9 };
	constexpr std::array<double, 6> q_max_allowed = { 2.9, 1.8, 2.60, 2.9, 1.85, 2.9 };
	constexpr std::array<double, 6> v_max_allowed = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
	constexpr std::array<double, 6> a_max_allowed = { 10, 10, 10, 10, 10, 10 };
	constexpr std::array<double, 6> j_max_allowed = { 400, 400, 400, 400, 400, 400 };
	constexpr std::array<double, 6> a_max_ltt = { 2, 2, 2, 2, 2, 2 };
	constexpr std::array<double, 6> j_max_ltt = { 15, 15, 15, 15, 15, 15 };

	//////////// Build human reach
	constexpr double measurement_error_pos = 0.0;
	constexpr double measurement_error_vel = 0.1;
	constexpr double delay = 0.0;

    // use extrimities as well
    constexpr bool use_combined_model = true;

    // use kalman filter
    constexpr bool use_kalman_filter = false;
    constexpr double s_w = 2.0e+2;
    constexpr double s_v = 1.0e-6;
    constexpr double initial_pos_var = 0.003;
    constexpr double initial_vel_var = 0.5;


	//std::vector<std::string> name_of_joints = { "lShoulder", "lElbow", "lHand", "rShoulder", "rElbow", "rHand", "Collar", "Torso", "Head" };
    constexpr std::array<char*, 9> joint_names = { "rShoulder", "lHand", "lShoulder", "Torso", "rElbow", "rHand", "Collar", "Head", "lElbow" };

	constexpr std::array<double, 9> joint_v_max = { 2, 2, 2, 2, 2, 2, 2, 2, 2 };
	constexpr std::array<double, 9> joint_a_max = { 50, 50, 50, 50, 50, 50, 20, 20, 50 };
	
    // Build bodies
	constexpr std::array<char*, 8> body_parts = {"lUpperArm", "lLowerArm", "lHand", "rUpperArm", "rLowerArm", "rHand", "Torso", "Head" };
	constexpr std::array<char*, 8> body_part_proximal = { "lShoulder", "lElbow", "lHand", "rShoulder", "rElbow", "rHand", "Collar", "Head" };
	constexpr std::array<char*, 8> body_part_distal = { "lElbow", "lHand", "lHand", "rElbow", "rHand", "rHand", "Torso", "Head" };
	constexpr std::array<double, 8> joint_thickness = { 0.108, 0.068, 0.208, 0.108, 0.068, 0.208, 0.193, 0.3 };

    constexpr std::array<char*, 2> extremities_base = {"lShoulder", "rShoulder"};
    constexpr std::array<char*, 2> extremities_end = {"lHand", "rHand"};
    constexpr std::array<double, 2> extremities_length = {1.0, 1.0};
    constexpr std::array<double, 2> extremities_thickness = {0.01, 0.01};

    /*
    std::vector<std::string> body_parts = {"rHand"};
      std::vector<std::string> body_part_proximal = {"rHand"};
      std::vector<std::string> body_part_distal = {"rHand"};
      std::vector<double> joint_thickness = {0.208};

      std::vector<std::string> extremities_base = {"rHand"};
      std::vector<std::string> extremities_end = {"rHand"};
      std::vector<double> extremities_length = {1.0};
      std::vector<double> extremities_thickness = {0.01};
    */
}

#endif  // config_safety_shield_H
