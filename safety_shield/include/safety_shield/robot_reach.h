// -*- lsst-c++ -*-
/**
 * @file robot_reach.h
 * @brief Define the class for robot reachability analysis
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

#include <Eigen/Dense>
#include <algorithm>
#include <exception>
#include <vector>

#include "reach_lib.hpp"
#include "safety_shield/motion.h"
#include "spdlog/spdlog.h"

#ifndef ROBOT_REACH_H
#define ROBOT_REACH_H

namespace safety_shield {

/**
 * @brief Class that calculates the robot reachable sets.
 */
class RobotReach {
 public:
  enum Velocity_method {
    APPROXIMATE,
    EXACT,
  };

  /**
   * @brief Velocity in SE3, where the first element is the linear velocity and the second the angular velocity
   */
  typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> SE3Vel;

  /**
   * @brief Veloctiy in SE3 of the two points that define a capsule
   */
  typedef std::pair<SE3Vel, SE3Vel> CapsuleVelocity;

 private:
  /**
   * @brief the number of joints of the robot
   */
  int nb_joints_;

  /**
   * @brief List of transforamtion matrices from joint to joint (fixed description, not including joint movements)
   */
  std::vector<Eigen::Matrix4d> transformation_matrices_;

  /**
   * @brief The enclosing capsules
   */
  std::vector<reach_lib::Capsule> robot_capsules_;

  /**
   * @brief Expands the radius of the robot capsules by this amount to
   *  account for measurement and modelling errors.
   */
  double secure_radius_;

  /**
   * @brief if maximum cartesian velocities should be approximative or exact
   */
  Velocity_method velocity_method_ = APPROXIMATE;

  /**
   * @brief enclosing capsules for velocity calculation
   */
  std::vector<reach_lib::Capsule> robot_capsules_for_velocity_;

  /**
   * @brief list of z-vectors (third column of transformation matrix) for velocity calculation
   */
  std::vector<Eigen::Vector3d> z_vectors_;

 public:
  /**
   * @brief A robot empty constructor
   */
  RobotReach() {}

  /**
   * @brief A robot basic constructor
   *
   * @param transformation_matrices the transformation matrices
   * @param nb_joints the number of joints of the robot
   * @param geom_param the robot occupancy matrix
   * @param x initial x position of base
   * @param y initial y position of base
   * @param z initial z position of base
   * @param roll initial roll of base
   * @param pitch initial pitch of base
   * @param yaw initial yaw of base
   * @param secure_radius Expand the radius of the robot capsules by this amount to
   *  account for measurement and modelling errors.
   */
  RobotReach(std::vector<double> transformation_matrices, int nb_joints, std::vector<double> geom_par, double x = 0,
             double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0, double secure_radius = 0);

  /**
   *  @brief A robot destructor
   */
  ~RobotReach() {}

  /**
   * @brief Reset the robot reach object.
   *
   * @param x initial x position of base
   * @param y initial y position of base
   * @param z initial z position of base
   * @param roll initial roll of base
   * @param pitch initial pitch of base
   * @param yaw initial yaw of base
   */
  void reset(double x, double y, double z, double roll, double pitch, double yaw);

  /**
   * @brief Computes the global transformation matrix of a given joint.
   *
   * @param q The joint angle
   * @param n_joint The number of joint
   * @param T The current transformation matrix (Start with Identity). T will be modified in this function.
   */
  inline void forwardKinematic(const double& q, const int& n_joint, Eigen::Matrix4d& T) {
    // Transform T to new joint coordinate system
    T = T * transformation_matrices_[n_joint + 1];
    Eigen::Matrix4d Rz;
    Rz << cos(q), -sin(q), 0, 0, sin(q), cos(q), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    T = T * Rz;
  }

  inline Eigen::Vector4d pointToVector(const reach_lib::Point& p) {
    Eigen::Vector4d vec;
    vec << p.x, p.y, p.z, 1.0;
    return vec;
  }

  inline Eigen::Vector3d pointTo3dVector(const reach_lib::Point& p) {
    Eigen::Vector3d vec;
    vec << p.x, p.y, p.z;
    return vec;
  }

  inline reach_lib::Point vectorToPoint(const Eigen::Vector4d& vec) {
    return reach_lib::Point(vec(0), vec(1), vec(2));
  }

  /**
   * @brief Transform the capsule of joint n by the transformation matrix T.
   * @return the transformed capsule
   */
  reach_lib::Capsule transformCapsule(const int& n_joint, const Eigen::Matrix4d& T);

  /**
   * @brief Calculates the reachable set from the new desired start and goal joint position.
   *
   * Computes the reachable occupancy capsules of the robot.
   * For a detailed proof of formality, please see: http://mediatum.ub.tum.de/doc/1443612/652879.pdf Chapter 3.4
   *
   * @param[in] start_config The configuration of the robot in the beginning of the trajectory
   * @param[in] goal_config The configuration of the robot in the end of the trajectory
   * @param[in] s_diff The difference in the trajectory time parameter s for the given path
   * @param[in] alpha_i The maximum acceleration of each capsule point with resprect to the time parameter s for the
   * given path
   *
   * @returns Array of capsules
   */
  std::vector<reach_lib::Capsule> reach(Motion& start_config, Motion& goal_config, double s_diff,
                                        std::vector<double> alpha_i);

  /**
   * @brief sets velocity_method
   */
  inline void setVelocityMethod(Velocity_method velocity_method) {
    velocity_method_ = velocity_method;
  }

  /**
   * @brief calculates maximum cartesian velocity for a specific robot configuration
   * @param motion configuration of robot
   * @return maximum cartesian velocity of motion
   */
  double maxVelocityOfMotion(const Motion& motion);

  /**
   * @brief Calculate the cartesian velocity of both defining point of a specific capsule
   * @assumption calculateAlltransformationMatricesAndCapsules() was called before
   * @param capsule which capsule
   * @param q_dot velocity configuration of robot
   * @return Velocity in SE3 of both Capusle points
   */
  CapsuleVelocity getVelocityOfCapsule(const int capsule, std::vector<double> q_dot);

  /**
   * @brief Calculate the maximum norm of the cartesian velocity for a specific capsule
   * @assumption calculateAlltransformationMatricesAndCapsules() was called before
   * @param capsule which capsule
   * @param q_dot velocity configuration of robot
   * @return maximum cartesian velocity of capsule
   */
  double getMaxVelocityOfCapsule(const int capsule, std::vector<double> q_dot);

  /**
   * @brief calculates all transformation matrices and capsules for a specific robot configuration and sets them
   * @param q vector of robot angles
   */
  void calculateAllTransformationMatricesAndCapsules(const std::vector<double>& q);

  /**
   * @brief Returns Jacobian of a point attached to the i-th joint.
   * @details You often take robot_capsules_for_velocity_[joint].p1_ or robot_capsules_for_velocity_[joint].p2_ as
   * point.
   * @assumption calculateAlltransformationMatricesAndCapsules() was called before
   * @param joint Jacobian for which joint
   * @param point Jacobian for which point
   * @return Jacobian 6 x nb_joints
   */
  Eigen::Matrix<double, 6, Eigen::Dynamic> getJacobian(const int joint, const reach_lib::Point& point);

  /**
   * @brief computes approximate maximum cartesian velocity of a specific capsule
   * @assumption calculateAlltransformationMatricesAndCapsules() was called before
   * @param capsule
   * @param v linear velocity at joint
   * @param omega angular velocity at joint
   * @return approximate maximum cartesian velocity of capsule
   */
  double approximateVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega);

  /**
   * @brief computes approximate maximum cartesian velocity of a specific capsule
   * @assumption calculateAlltransformationMatricesAndCapsules() was called before
   * @param capsule
   * @param v linear velocity at joint
   * @param omega angular velocity at joint
   * @return approximate maximum cartesian velocity of capsule
   */
  double exactVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega);

  /**
   * @brief computes cross product as skew-symmetric matrix
   * @param vec
   * @return matrix
   */
  inline Eigen::Matrix3d getCrossProductAsMatrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d cross;
    cross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return cross;
  }

  /**
   * @brief returns robot capsules for velocity
   * @return robot capsules for velocity
   */
  inline std::vector<occupancy_containers::capsule::Capsule>& getRobotCapsulesForVelocity() {
    return robot_capsules_for_velocity_;
  }
};
}  // namespace safety_shield

#endif  // ROBOT_REACH_H
