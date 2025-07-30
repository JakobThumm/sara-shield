// -*- lsst-c++ -*/
/**
 * @file safety_shield.h
 * @brief Defines the online verification class
 * @version 0.1
 * @copyright This file is part of SaRA-Shield.
 * SaRA-Shield is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 * SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with SaRA-Shield.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdexcept>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "long_term_planner/long_term_planner.h"
#include "reach_lib.hpp"
#include "safety_shield/exceptions.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/path.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/verify_iso.h"
#include "safety_shield/trajectory_utils.h"
#include "safety_shield/config_utils.h"
#include "spdlog/spdlog.h"

#ifndef safety_shield_H
#define safety_shield_H

namespace safety_shield {

/**
 * @brief The type of shield to use.
 *
 * @details
 *  OFF: No shield is used.
 *  SSM: Speed and Separation Monitoring: The robot stops for the human.
 *  PFL: Power and Force Limiting: The robot slows down to a safe contact speed for the human.
 */
enum class ShieldType { OFF, SSM, PFL };

/**
 * @brief Computes the failsafe trajectory
 */
class SafetyShield {
 private:
  /**
   * @brief The type of shield to use.
   *
   * @details
   *  OFF: No shield is used.
   *  SSM: Speed and Separation Monitoring: The robot stops for the human.
   *  PFL: Power and Force Limiting: The robot slows down to a safe contact speed for the human.
   */
  ShieldType shield_type_;

  /**
   * @brief Robot reachable set calculation object
   *
   */
  RobotReach* robot_reach_;

  /**
   * @brief Human reachable set calcualtion object
   *
   */
  HumanReach* human_reach_;

  /**
   * @brief Verifier object
   *
   * Takes the robot and human capsules as input and checks them for collision.
   */
  VerifyISO* verify_;

  /**
   * @brief The visualization of reachable sets
   *
   * TODO: Write a visualization for mujoco
   */
  // RvizMarker* rviz_;

  /**
   * @brief path to go back to the long term plan
   */
  Path intended_path_;

  /**
   * @brief fail-safe path of the current path
   */
  Path failsafe_path_;

  /**
   * @brief fail-safe path of the repair path
   */
  Path failsafe_path_2_;

  /**
   * @brief verified safe path
   */
  Path verified_path_;

  /**
   * @brief the constructed intended step + failsafe path
   */
  Path monitored_path_;

  /**
   * @brief Number of joints of the robot
   */
  int nb_joints_;

  /**
   * @brief sampling time
   */
  double sample_time_;

  /**
   * @brief the number of samples since start
   */
  int path_s_discrete_;

  /**
   * @brief Time since start
   */
  double path_s_;

  /**
   * @brief Was the last timestep safe
   */
  bool is_safe_;

  /**
   * @brief Whether or not the monitored path is under the safe velocity the entire time.
   *
   */
  bool is_under_v_limit_ = false;

  /**
   * @brief Indicates if the last replanning was successful or not.
   *
   * Indicates problems in the following statements:
   * - It is not strictly guaranteed that the manoeuvres generated maintain 0 ≤ ṡ ≤ 1. In
   * practice, this is only a problem when s̈ ˙ max or s̈ max change rapidly from one timestep to
   * the next, causing the trajectory of s̈ to “overshoot” 0 or 1. Since at all times we have a
   * failsafe trajectory available, verified in advance, which brings the robot to a safe state, if a
   * proposed short-term plan were to overshoot at any point during the plan, this short-term
   * plan is verified as unsafe and the failsafe trajectory is chosen.
   * - Again, when s̈ ˙ m or s̈ m change rapidly between timesteps, it may occur that |s̈| > s̈ m at
   * the start of a proposed short-term plan. Again, if this occurs, that particular short-term
   * plan is verified as unsafe and the failsafe trajectory is chosen.
   */
  bool intended_path_correct_ = false;

  /**
   * @brief The last published motion
   */
  Motion next_motion_;

  /**
   * @brief The new long term goal
   */
  Motion new_goal_motion_;

  /**
   * @brief the maximum time to stop
   */
  double max_s_stop_;

  /**
   * @brief the maximum time to stop in timesteps (discrete)
   */
  int sliding_window_k_;

  /**
   * @brief Minimum angle (absolute)
   */
  std::vector<double> q_min_allowed_;

  /**
   * @brief Maximum angle (absolute)
   */
  std::vector<double> q_max_allowed_;

  /**
   * @brief maximum joint velocity allowed
   */
  std::vector<double> v_max_allowed_;

  /**
   * @brief maximum joint acceleration allowed
   */
  std::vector<double> a_max_allowed_;

  /**
   * @brief maximum joint jerk allowed
   */
  std::vector<double> j_max_allowed_;

  /**
   * @brief maximum acceleration along the long term plan
   */
  std::vector<double> a_max_ltt_;

  /**
   * @brief maximum jerk along the long term plan
   */
  std::vector<double> j_max_ltt_;

  /**
   * @brief maximum cartesian acceleration of robot joints
   *
   * @details Might not be used, depending on the LTT constructor used.
   */
  double alpha_i_max_;

  /**
   * @brief the stored long_term_trajectory
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief new LTT that wants to override the current LTT
   */
  LongTermTraj new_long_term_trajectory_;

  /**
   * @brief indicates that there is a potential new LTT
   */
  bool new_ltt_ = false;

  /**
   * @brief indicates that there is a new goal to compute a new LTT.
   *
   * We need a differentation between new goal and new LTT because an LTT to a new goal can only be calculated if the
   * accerlation and jerk values are within the LTT planning bounds.
   */
  bool new_goal_ = false;

  /**
   * @brief indicates that the new LTT was passed to the safety verification at least once.
   */
  bool new_ltt_processed_ = false;

  /**
   * @brief Last s value when replanning happend
   *
   * If the last starting position of the replanning is very close to this position, we can skip the replanning and use
   * the previously planned trajectory.
   */
  double last_replan_s_ = -1.0;

  /**
   * @brief the time when the loop begins
   */
  double cycle_begin_time_;

  //////// Reachable sets of human and robot //////
  /**
   * @brief robot reachable set capsules for every time interval (gets updated in every step())
   * first index is time interval; second index is robot capsule
   */
  std::vector<std::vector<reach_lib::Capsule>> robot_capsules_time_intervals_;

  /**
   * @brief Vector of robot reachable set capsules of last time interval (for visualization in hrgym)
   */
  std::vector<reach_lib::Capsule> robot_capsules_;

  /**
   * @brief human reachable sets for every time interval (gets updated in every step())
   */
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> human_capsules_time_intervals_;

  /**
   * @brief Vector of human reachable set capsules of last time interval (for visualization in hrgym)
   */
  std::vector<std::vector<reach_lib::Capsule>> human_capsules_;

  /**
   * @brief The worst-case contact type on the end effector.
   */
  ContactType eef_contact_type_;

  /**
   * @brief Maximum contact energies for (un)constrained contacts.
   * @details The format is [robot_link_index][model_index][body_index].
   * @details The robot links and the end effector have different allowed contact energies,
   *    as the end-effector can have sharp edges.
   */
  std::vector<std::vector<std::vector<double>>> max_contact_energies_unconstrained_;
  std::vector<std::vector<std::vector<double>>> max_contact_energies_constrained_;

  //////// For replanning new trajectory //////
  /**
   * @brief Trajecory planner
   */
  long_term_planner::LongTermPlanner ltp_;

  /**
   * @brief Axis-aligned bounding boxes of the environment elements.
   */
  std::vector<reach_lib::AABB> environment_elements_;

  /**
   * @brief maximum cartesian velocity allowed at collision in m/s
   */
  double v_safe_ = 0.10;

  /**
   * @brief Defines the length of a time interval used for human and robot reachability analysis.
   * Each time interval consists of this many shield time steps.
   * A good value is 5
   */
  int reachability_set_interval_size_ = 10000;

  /**
   * @brief duration is interval size * sampling time of safety shield
   */
  double reachability_set_duration_;

  /**
   * @brief This value is substracted from the maximal allowed q pos values before planning the LTT.
   * 
   */
  double planning_qpos_tolerance_ = 0.01;

 protected:
  /**
   * @brief Calculate max acceleration and jerk based on previous velocity
   * @details Mathematical explanation in http://mediatum.ub.tum.de/doc/1443612/652879.pdf eq. 2.6a and b (P.20).
   *
   * @param[in] prev_speed vector of previous joint velocities
   * @param[in] a_max_part max acceleration for this part of the LTT
   * @param[in] j_max_part max jerk for this part of the LTT
   * @param[out] a_max_manoeuvre Maximum path acceleration
   * @param[out] j_max_manoeuvre Maximum path jerk
   */
  void calculateMaxAccJerk(const std::vector<double>& prev_speed, const std::vector<double>& a_max_part,
                           const std::vector<double>& j_max_part, double& a_max_manoeuvre, double& j_max_manoeuvre);

  /**
   * @brief Computes the fail-safe path
   *
   * @param[in] pos,vel,acc the starting point caracteristics
   * @param[in] ve the desired final velocity
   * @param[in] a_max the maximum acceleration allowed
   * @param[in] j_max the maximum jerk allowed
   * @param[out] path the new path
   * @return Whether the planning was successful or not
   */
  bool planSafetyShield(double pos, double vel, double acc, double ve, double a_max, double j_max, Path& path);

  /**
   * @brief Computes the fail-safe path for PFL mode.
   *
   * @details Sets the failsafe_path_2_ variable.
   *
   * @param[in] a_max_manoeuvre Maximum path acceleration
   * @param[in] j_max_manoeuvre Maximum path jerk
   *
   * @return true planning was successful
   * @return false planning failed
   */
  bool planPFLFailsafe(double a_max_manoeuvre, double j_max_manoeuvre);

  /**
   * @brief Update the safe path to the monitored path if the verification is successful (safe) or increment the existing safe path otherwise.
   * 
   * @param is_safe Whether or not the verification was successful.
   */
  inline void updateSafePath(bool is_safe) {
    if (is_safe) {
      verified_path_ = monitored_path_;
    } else {
      verified_path_.increment(sample_time_);
    }
    // Set s to the new path position
    path_s_ = verified_path_.getPosition();
  }

  /**
   * @brief Check a given motion if it exceeds the joint limits.
   *
   * @param motion Motion to check
   * @return true if path does NOT exceed joint limits
   * @return false if path exceeds joint limits
   */
  bool checkMotionForJointLimits(Motion& motion);

  /**
   * @brief round a continuous time to a timestep
   * @param t continuous time
   * @return timestep
   */
  inline double roundToTimestep(double t) {
    return ceil(t / sample_time_) * sample_time_;
  }

  /**
   * @brief Determines if the current motion is in the acceleration bounds for replanning
   *
   * @param current_motion current motion
   * @returns bool: if the current motion lies in the bounds for replanning
   */
  bool checkCurrentMotionForReplanning(Motion& current_motion);

  /**
   * @brief Calculates a new trajectory from current joint state to desired goal state.
   * @param start_q The current joint angles
   * @param start_dq The current joint velocities
   * @param start_ddq The current joint accelerations
   * @param goal_q The desired joint angles
   * @param ltt The calculated long-term trajectory
   * @return True if success, false otherwise
   */
  bool calculateLongTermTrajectory(const std::vector<double>& start_q, const std::vector<double> start_dq,
                                   const std::vector<double> start_ddq, const std::vector<double>& goal_q,
                                   LongTermTraj& ltt);

  /**
   * @brief Convert a capsule to a vector containing [p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, r]
   * p1: Center point of half sphere 1
   * p2: Center point of half sphere 2
   * r: Radius of half spheres and cylinder
   *
   * @param cap Capsule
   * @return std::vector<double>
   */
  inline std::vector<double> convertCapsule(const reach_lib::Capsule& cap) {
    std::vector<double> capsule(7);
    capsule[0] = cap.p1_.x;
    capsule[1] = cap.p1_.y;
    capsule[2] = cap.p1_.z;
    capsule[3] = cap.p2_.x;
    capsule[4] = cap.p2_.y;
    capsule[5] = cap.p2_.z;
    capsule[6] = cap.r_;
    return capsule;
  }

 public:
  /**
   * @brief Default constructor
   *
   */
  SafetyShield();

  /**
   * @brief Construct a new Safety Shield object
   *
   * @param nb_joints Number of joints of the robot
   * @param sample_time Sample time of safety shield
   * @param max_s_stop Maximal path length to stop the robot
   * @param v_max_allowed Maximal allowed joint speed
   * @param a_max_allowed Maximal allowed joint acceleration
   * @param j_max_allowed Maximal allowed joint jerk
   * @param a_max_path Maximal allowed relative path acceleration
   * @param j_max_path Maximal allowed relative path jerk
   * @param long_term_trajectory Fixed trajectory to execute (will be overwritten by new intended goals)
   *    This also defines the initial qpos.
   * @param robot_reach Robot reachable set calculation object
   * @param human_reach Human reachable set calculation object
   * @param verify Verification of reachable sets object
   * @param environment_elements Elements of the environment (as AABB)
   * @param shield_type What type of safety shield to use, select from `OFF`, `SSM`, or `PFL`
   * @param eef_contact_type The worst-case contact type on the end effector.
   */
  SafetyShield(int nb_joints, double sample_time, double max_s_stop, const std::vector<double>& v_max_allowed,
               const std::vector<double>& a_max_allowed, const std::vector<double>& j_max_allowed,
               const std::vector<double>& a_max_path, const std::vector<double>& j_max_path,
               const LongTermTraj& long_term_trajectory, RobotReach* robot_reach, HumanReach* human_reach,
               VerifyISO* verify, 
               const std::vector<reach_lib::AABB> &environment_elements,
               ShieldType shield_type = ShieldType::SSM,
               ContactType eef_contact_type = ContactType::EDGE);

  /**
   * @brief Construct a new Safety Shield object from config files.
   *
   * @param sample_time Sample time of shield
   * @param trajectory_config_file Path to config file defining the trajectory parameters
   * @param robot_config_file Path to config file defining the robot transformation matrices and capsules
   * @param mocap_config_file Path to config file defining the human motion capture
   * @param init_x Base x pos
   * @param init_y Base y pos
   * @param init_z Base z pos
   * @param init_roll Base roll
   * @param init_pitch Base pitch
   * @param init_yaw Base yaw
   * @param init_qpos Initial joint position of the robot
   * @param environment_elements Elements of the environment (as AABB)
   * @param shield_type What type of safety shield to use, select from `OFF`, `SSM`, or `PFL`
   * @param eef_contact_type The worst-case contact type on the end effector.
   */
  SafetyShield(double sample_time, std::string trajectory_config_file, std::string robot_config_file,
               std::string mocap_config_file, double init_x, double init_y, double init_z, double init_roll,
               double init_pitch, double init_yaw, const std::vector<double>& init_qpos,
               const std::vector<reach_lib::AABB> &environment_elements,
               ShieldType shield_type = ShieldType::SSM,
               ContactType eef_contact_type = ContactType::EDGE);

  /**
   * @brief A SafetyShield destructor
   */
  ~SafetyShield(){
    delete robot_reach_;
    delete human_reach_;
    delete verify_;
  };

  /**
   * @brief Resets the safety shield completely.
   *
   * @param init_x Base x pos
   * @param init_y Base y pos
   * @param init_z Base z pos
   * @param init_roll Base roll
   * @param init_pitch Base pitch
   * @param init_yaw Base yaw
   * @param init_qpos Initial joint position of the robot
   * @param current_time Initial time
   * @param environment_elements Elements of the environment (as AABB)
   * @param shield_type What type of safety shield to use, select from `OFF`, `SSM`, or `PFL`
   * @param eef_contact_type The worst-case contact type on the end effector.
   */
  void reset(double init_x, double init_y, double init_z, double init_roll, double init_pitch, double init_yaw,
             const std::vector<double>& init_qpos, double current_time, 
             const std::vector<reach_lib::AABB> &environment_elements, ShieldType shield_type = ShieldType::SSM,
             ContactType eef_contact_type = ContactType::EDGE);

  /**
   * @brief Computes the new trajectory depending on dq and if the previous path is safe and publishes it
   * @param[in] v is the previous path safe
   * @param[in] prev_speed the velocity of the previous point
   * @returns goal_motion: goal position, velocity, acceleration and time of the computed trajectory to execute.
   */
  Motion computesPotentialTrajectory(bool v, const std::vector<double>& prev_speed);

  /**
   * @brief Gets the information that the next simulation cycle (sample time) has started
   * @param cycle_begin_time timestep of begin of current cycle in seconds.
   *
   * @return next motion to be executed
   */
  Motion step(double cycle_begin_time);

  /**
   * @brief Calculates and returns the current motion
   */
  Motion getCurrentMotion();

  /**
   * @brief Evaluate if the new long term trajectory is processed and safe to use.
   * @details Should be called in the `step()` function.
   *  This function sets the following internal attributes:
   *    - long_term_trajectory_: Will be overridden by the new long term trajectory if it is safe.
   *    - new_ltt_: Will be set to false if the new LTT is safe.
   *    - new_ltt_processed_: Will be set to false if the new LTT is safe.
   * @param current_motion current motion we are in.
   */
  void evaluateNewLTTProcessed(Motion& current_motion);

  /**
   * @brief Evaluate if replanning a new LTT is necessary and plan one if needed.
   * @details Should be called in the `step()` function if the new_goal_ flag is set, i.e.,
   *  the user requested a new goal to move to.
   *  This function sets the following internal attributes:
   *    - new_long_term_trajectory_: The newly calculated LTT
   *    - new_ltt_: Flag that indicates that a new LTT is available
   *    - new_ltt_processed_: Flag that indicates that the new LTT was passed to the safety verification at least once.
   * 
   * @param current_motion current motion we are in.
   */
  void newGoalPlanning(Motion& current_motion);

  /**
   * @brief verify the safety of the movement from the current motion to the goal motion.
   * 
   * @details If the shield mode is OFF, the function will always return true.
   * This function sets robot_capsules_ and human_capsules_ needed for plotting.
   * 
   * @param[in] current_motion Current motion the robot is in.
   * @param[in] goal_motion Goal motion the robot wants to move to.
   * @param[in] alpha_i Maximum cartesian acceleration of robot joints
   * @return true if safe
   * @return false if unsafe
   */
  bool verifySafety(Motion& current_motion, Motion& goal_motion, const std::vector<double>& alpha_i);

  /**
   * @brief verify if the contact energy constraint is satisfied
   * 
   * @param[in] time_points Time points that define the edges of the time intervals.
   * @param[in] interval_edges_motions Motions at the edges of the time intervals.
   * @param[in] collision_index Index of the first collision in the time intervals, -1 if no collision.
   * @return true if safe
   * @return false if unsafe
   */
  bool verifyContactEnergySafety(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index);

  /**
   * @brief verify if the contact energy constraint is satisfied.
   * 
   * @details First classifies the contact type and then checks if the contact energy constraint is satisfied.
   * 
   * @param[in] time_points Time points that define the edges of the time intervals.
   * @param[in] interval_edges_motions Motions at the edges of the time intervals.
   * @param[in] collision_index Index of the first collision in the time intervals, -1 if no collision.
   * @return true if safe
   * @return false if unsafe
   */
  bool verifyContactEnergySafetyByContactType(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index);

  /**
   * @brief Build the robot capsule velocity pointers for the given interval edges motions.
   * 
   * @param[in] interval_edges_motions The motions at the edges of the time intervals.
   * @param[out] vel_cap_start The pointer to the capsule velocity at the start of the interval.
   * @param[out] vel_cap_end The pointer to the capsule velocity at the end of the interval.
   */
  void buildRobotVelocityPointers(
    const std::vector<Motion>& interval_edges_motions,
    std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator>& vel_cap_start,
    std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator>& vel_cap_end);

  /**
   * @brief verify if the contact velocity constraint is satisfied.
   * 
   * @details This function is not fully implemented and therefore not tested!!!
   * @details This function is not in use.
   * 
   * @param[in] time_points Time points that define the edges of the time intervals.
   * @param[in] interval_edges_motions Motions at the edges of the time intervals.
   * @param[in] collision_index Index of the first collision in the time intervals, -1 if no collision.
   * @return true if safe
   * @return false if unsafe
   */
  bool verifyContactVelocitySafety(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index);

  /**
   * @brief verify if the constrained contact constraint (clamping) is satisfied
   * 
   * @param[in] time_points Time points that define the edges of the time intervals.
   * @param[in] interval_edges_motions Motions at the edges of the time intervals.
   * @param[in] collision_index Index of the first collision in the time intervals, -1 if no collision.
   * @return true if safe
   * @return false if unsafe
   */
  bool verifyConstrainedContactSafety(std::vector<double> time_points, std::vector<Motion> interval_edges_motions, int& collision_index);

  /**
   * @brief Calculates a new trajectory from current joint state to desired goal state.
   * Sets new trajectory as desired new long term trajectory.
   * @param goal_position Desired joint angles to move to
   * @param goal_velocity Desired joint velocities at the goal position
   */
  void newLongTermTrajectory(const std::vector<double>& goal_position, const std::vector<double>& goal_velocity);

  /**
   * @brief Overrides the current long-term trajectory.
   * @details Requires the robot to be at a complete stop, i.e. v=a=j=0.0 for all joints
   *    Requires the LTT to end in a complete stop.
   *    Requires the LTT to start in the same position as the robot.
   *    Requires the LTT to start with v=0
   *
   * @param traj New long-term trajectory
   *
   * @throws RobotMovementException Robot is not v=a=j=0
   * @throws TrajectoryException Incorrect LTT
   */
  void setLongTermTrajectory(LongTermTraj& traj);

  /**
   * @brief Set the worst-case contact type on the end effector.
   * @details Use this function if your end effector tool changes.
   * @details Calls buildMaxContactEnergies() to update the contact energies.
   * 
   * @param contact_type choose between BLUNT, EDGE, WEDGE, and SHEET
   */
  inline void setEEFContactType(ContactType contact_type) {
    eef_contact_type_ = contact_type;
    buildMaxContactEnergies();
  }

  /**
   * @brief Build the max_contact_energies_(un)constrained_ objects.
   * @details The HumanReach object has to be initialized before calling this function.
   */
  void buildMaxContactEnergies();

  /**
   * @brief Calculate the list of motions on the LTT based on the current path and the given time points.
   * 
   * @param time_points Time points to return the list of motions at.
   * @return std::vector<Motion> 
   */
  std::vector<Motion> getMotionsFromCurrentLTTandPath(const std::vector<double>& time_points);

  /**
   * @brief Calculate the list of link inertia matrices on the LTT based on the current path and the given time points.
   * 
   * @param time_points Time points to return the list of motions at.
   * @return std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>>
   */
  std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> getInertiaMatricesFromCurrentLTTandPath(const std::vector<double>& time_points);

  /**
   * @brief Receive a new human measurement
   * @param[in] human_measurement A vector of human joint measurements (list of reach_lib::Points)
   * @param[in] time The timestep of the measurement in seconds.
   */
  inline void humanMeasurement(const std::vector<reach_lib::Point> human_measurement, double time) {
    human_reach_->measurement(human_measurement, time);
  }

  /**
   * @brief Receive a new human measurement.
   * Calls humanMeasurement(const std::vector<reach_lib::Point> human_measurement, double time).
   * @param[in] human_measurement A vector of human joint measurements (list of list of doubles [x, y, z])
   * @param[in] time The timestep of the measurement in seconds.
   */
  inline void humanMeasurement(const std::vector<std::vector<double>> human_measurement, double time) {
    std::vector<reach_lib::Point> converted_vec;
    for (int i = 0; i < human_measurement.size(); i++) {
      converted_vec.push_back(
          reach_lib::Point(human_measurement[i][0], human_measurement[i][1], human_measurement[i][2]));
    }
    humanMeasurement(converted_vec, time);
  }

  /**
   * @brief Get the Robot Reach Capsules as a vector of [p1[0:3], p2[0:3], r]
   * p1: Center point of half sphere 1
   * p2: Center point of half sphere 2
   * r: Radius of half spheres and cylinder
   *
   * @return std::vector<std::vector<double>> Capsules
   */
  inline std::vector<std::vector<double>> getRobotReachCapsules() {
    std::vector<std::vector<double>> capsules(robot_capsules_.size(), std::vector<double>(7));
    for (int i = 0; i < robot_capsules_.size(); i++) {
      capsules[i] = convertCapsule(robot_capsules_[i]);
    }
    return capsules;
  }

  /**
   * @brief Get the Human Reach Capsules as a vector of [p1[0:3], p2[0:3], r]
   * p1: Center point of half sphere 1
   * p2: Center point of half sphere 2
   * r: Radius of half spheres and cylinder
   *
   * @param type Type of capsule. 
   *  If in combined mode: select 0 for combined model
   *  If not in combined mode: select 0 for POS, 1 for VEL, and 2 for ACCEL
   *
   * @return std::vector<std::vector<double>> Capsules
   */
  inline std::vector<std::vector<double>> getHumanReachCapsules(int type = 0) {
    if (type < 0 || type >= human_capsules_.size()) {
      throw std::invalid_argument("Invalid type of human reach capsules requested. Please select a value between 0 and " +
                                  std::to_string(human_capsules_.size() - 1));
    }
    std::vector<std::vector<double>> capsules(human_capsules_[type].size(), std::vector<double>(7));
    for (int i = 0; i < human_capsules_[type].size(); i++) {
      capsules[i] = convertCapsule(human_capsules_[type][i]);
    }
    return capsules;
  }

  inline bool getSafety() {
    return is_safe_;
  }

  inline ShieldType getShieldType() {
    return shield_type_;
  }

  inline double getSampleTime() const {
    return sample_time_;
  }

  inline ContactType getEEFContactType() const {
    return eef_contact_type_;
  }

};
}  // namespace safety_shield

#endif  // safety_shield_H