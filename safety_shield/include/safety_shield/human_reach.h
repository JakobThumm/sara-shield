// -*- lsst-c++ -*/
/**
 * @file human_reach.h
 * @brief Defines the human reach class
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

#include <assert.h>

#include <exception>
#include <map>
#include <string>
#include <vector>

#include "reach_lib.hpp"
#include "spdlog/spdlog.h"

#ifndef HUMAN_REACH_H
#define HUMAN_REACH_H

namespace safety_shield {

/**
 * @brief Class handling the reachability analysis of the human.
 *
 * This class holds all three articulated models (pos, vel, accel) of the human,
 * and handels incoming measurements.
 */
class HumanReach {
 private:
  /**
   * @brief Last measured timestep.
   */
  double last_meas_timestep_ = -1;

  /**
   * @brief Number of joint measurements.
   */
  int n_joints_meas_;

  /**
   * @brief Joint position measurements
   */
  std::vector<reach_lib::Point> joint_pos_;

  /**
   * @brief Calculated velocities
   */
  std::vector<reach_lib::Point> joint_vel_;

  /**
   * @brief Map the entries of body links to their proximal and distal joint.
   */
  std::map<std::string, reach_lib::jointPair> body_link_joints_;

  /**
   * @brief Map entries of body to index (in capsule list) and maximum allowed collision velocity
   */
  std::map<std::string, std::pair<int, double>> body_to_index_and_velocity_;

  /**
   * @brief The object for calculating the position based reachable set.
   */
  reach_lib::ArticulatedPos human_p_;

  /**
   * @brief The object for calculating the velocity based reachable set.
   */
  reach_lib::ArticulatedVel human_v_;

  /**
   * @brief The object for calculating the acceleration based reachable set.
   */
  reach_lib::ArticulatedAccel human_a_;

  /**
   * @brief We need two measurements for velocity calculation.
   */
  bool has_second_meas_ = false;

  /**
   * @brief Maximal positional measurement error
   *
   */
  double measurement_error_pos_;

  /**
   * @brief Maximal velocity measurement error
   *
   */
  double measurement_error_vel_;

  /**
   * @brief Delay in measurement processing pipeline
   *
   */
  double delay_;

 public:
  /**
   * @brief Empty constructor
   */
  HumanReach() {}

  /**
   * @brief HumanReach constructor
   * @param[in] n_joints_meas Number of joints in the measurement
   * @param[in] joint_names Maps the joint name to the joint index (key: Joint name, value: Joint index)
   * @param[in] measurement_error_pos Maximal positional measurement error
   * @param[in] measurement_error_vel Maximal velocity measurement error
   * @param[in] delay Delay in measurement processing pipeline
   * @param[in] joint_pair_map Maps the proximal and distal joint to a body part identified by a string (key: Name of
   * body part, value: Proximal and distal joint index)
   * @param[in] thickness Defines the thickness of the body parts (key: Name of body part, value: Thickness of body
   * part)
   * @param[in] max_v The maximum velocity of the joints
   * @param[in] max_a The maximum acceleration of the joints
   * @param[in] extremity_base_names The base joints of extremities, e.g., right / left shoulder, right / left hip
   * socket
   * @param[in] extremity_end_names The end joints of extremities, e.g., right / left hand, right / left foot --> Is
   * used for thickness of extremities
   * @param[in] extremity_length The max length of the extremities (related to extremity_base_names)
   * @param[in] extremity_thickness The thickness of the extremities (usually zero, as it is already considered in
   * extremity_length)
   * @param[in] wrist_names The name identifiers of the two hands
   */
  HumanReach(int n_joints_meas,
             std::map<std::string, int> joint_names,
             std::map<std::string, reach_lib::jointPair>& body_link_joints,
             const std::map<std::string, double>& thickness,
             std::vector<double>& max_v,
             std::vector<double>& max_a,
             std::vector<std::string>& extremity_base_names,
             std::vector<std::string>& extremity_end_names,
             std::vector<double>& extremity_length,
             std::vector<double>& extremity_thickness,
             double measurement_error_pos,
             double measurement_error_vel,
             double delay);

  /**
   * @brief HumanReach constructor
   * @param[in] n_joints_meas Number of joints in the measurement
   * @param[in] joint_names Maps the joint name to the joint index (key: Joint name, value: Joint index)
   * @param[in] measurement_error_pos Maximal positional measurement error
   * @param[in] measurement_error_vel Maximal velocity measurement error
   * @param[in] delay Delay in measurement processing pipeline
   * @param[in] joint_pair_map Maps the proximal and distal joint to a body part identified by a string (key: Name of
   * body part, value: Proximal and distal joint index)
   * @param[in] thickness Defines the thickness of the body parts (key: Name of body part, value: Thickness of body
   * part)
   * @param[in] max_v The maximum velocity of the joints
   * @param[in] max_a The maximum acceleration of the joints
   * @param[in] extremity_base_names The base joints of extremities, e.g., right / left shoulder, right / left hip
   * socket
   * @param[in] extremity_end_names The end joints of extremities, e.g., right / left hand, right / left foot --> Is
   * used for thickness of extremities
   * @param[in] extremity_length The max length of the extremities (related to extremity_base_names)
   * @param[in] extremity_thickness The thickness of the extremities (usually zero, as it is already considered in
   *extremity_length)
   * @param[in] wrist_names The name identifiers of the two hands
   **/
  HumanReach(int n_joints_meas,
             std::map<std::string, int> joint_names,
             std::map<std::string, reach_lib::jointPair>& body_link_joints,
             std::map<std::string, std::pair<int, double>>& body_to_index_and_velocity,
             const std::map<std::string, double>& thickness,
             std::vector<double>& max_v,
             std::vector<double>& max_a,
             std::vector<std::string>& extremity_base_names,
             std::vector<std::string>& extremity_end_names,
             std::vector<double>& extremity_length,
             std::vector<double>& extremity_thickness,
             double measurement_error_pos,
             double measurement_error_vel,
             double delay);

  /**
   * @brief Destructor
   */
  ~HumanReach() {}

  /**
   * @brief Reset the human reach object.
   *
   */
  void reset();

  /**
   * @brief Update the joint measurements.
   * @param[in] human_joint_pos The positions of the human joints.
   * @param[in] time The simulation time.
   */
  void measurement(const std::vector<reach_lib::Point>& human_joint_pos, double time);

  /**
   * @brief Calculate reachability analysis for given breaking time.
   *
   * Updates the values in human_p_, human_v_, human_a_.
   * Get the values afterwards with the getter functions!
   *
   * @param[in] t_command Current time
   * @param[in] t_brake Time horizon of reachability analysis
   */
  void humanReachabilityAnalysis(double t_command, double t_brake);

  /**
   * @brief Get the Articulated Pos capsules
   *
   * @return reach_lib::ArticulatedPos capsules
   */
  inline std::vector<reach_lib::Capsule> getArticulatedPosCapsules() {
    return reach_lib::get_capsules(human_p_);
  }

  /**
   * @brief Get the Articulated Vel capsules
   *
   * @return reach_lib::ArticulatedVel capsules
   */
  inline std::vector<reach_lib::Capsule> getArticulatedVelCapsules() {
    return reach_lib::get_capsules(human_v_);
  }

  /**
   * @brief Get the Articulated Accel capsules
   *
   * @return reach_lib::ArticulatedAccel capsules
   */
  inline std::vector<reach_lib::Capsule> getArticulatedAccelCapsules() {
    return reach_lib::get_capsules(human_a_);
  }

  /**
   * @brief Get the All Capsules of pos, vel, and accel
   *
   * @return std::vector<std::vector<reach_lib::Capsule>>
   */
  inline std::vector<std::vector<reach_lib::Capsule>> getAllCapsules() {
    return {getArticulatedPosCapsules(), getArticulatedVelCapsules(), getArticulatedAccelCapsules()};
  }

  /**
   * @brief Get the Last Meas Timestep object
   *
   * @return double
   */
  inline double getLastMeasTimestep() {
    return last_meas_timestep_;
  }

  /**
   * @brief Get the Joint Pos object
   *
   * @return std::vector<reach_lib::Point>
   */
  inline std::vector<reach_lib::Point> getJointPos() {
    return joint_pos_;
  }

  /**
   * @brief Get the Joint Vel object
   *
   * @return std::vector<reach_lib::Point>
   */
  inline std::vector<reach_lib::Point> getJointVel() {
    return joint_vel_;
  }

  /**
   * @brief Get the Body Link Joints object
   *
   * @return std::map<std::string, reach_lib::jointPair>
   */
  inline std::map<std::string, reach_lib::jointPair> getBodyLinkJoints() {
    return body_link_joints_;
  }

  /**
   * @brief get the Body To Index and Velocity map
   *
   * @return std::map<std::string, std::pair<int, double>>
   */
  inline std::map<std::string, std::pair<int, double>> getBodyToIndexAndVelocity() {
    return body_to_index_and_velocity_;
  };

  /**
   * @brief Get the Measurement Error Pos object
   *
   * @return double
   */
  inline double getMeasurementErrorPos() {
    return measurement_error_pos_;
  }

  /**
   * @brief Get the Measurement Error Vel object
   *
   * @return double
   */
  inline double getMeasurementErrorVel() {
    return measurement_error_vel_;
  }

  /**
   * @brief Get the Delay object
   *
   * @return double
   */
  inline double getDelay() {
    return delay_;
  }
};
}  // namespace safety_shield
#endif  // HUMAN_REACH_H
