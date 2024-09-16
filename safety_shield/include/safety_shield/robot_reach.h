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
  enum VelocityMethod {
    APPROXIMATE,
    EXACT,
  };

  /**
   * @brief Velocity in SE3, where the first element is the linear velocity and the second the angular velocity
   */
  struct SE3Vel {
    SE3Vel() {}
    SE3Vel(const Eigen::Vector3d& v, const Eigen::Vector3d& w) : v(v), w(w) {}
    /**
     * @brief Linear velocity.
     */
    Eigen::Vector3d v;
    /**
     * @brief Angular velocity.
     */
    Eigen::Vector3d w;
  };

  /**
   * @brief Veloctiy in SE3 of the two points that define a capsule
   */
  struct CapsuleVelocity {
    CapsuleVelocity() {}
    CapsuleVelocity(const SE3Vel& v1, const SE3Vel& v2) : v1(v1), v2(v2) {}
    /**
     * @brief SE3 velocity of the first point.
     */
    SE3Vel v1;
    /**
     * @brief SE3 velocity of the second point.
     */
    SE3Vel v2;
  };

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
   * @brief List of link lengths.
   */
  std::vector<double> link_lengths_;

  /**
   * @brief Masses of the links.
   */
  std::vector<double> link_masses_;

  /**
   * @brief Inertia matrices of the individual links.
   */
  std::vector<Eigen::Matrix3d> link_inertias_;

  /**
   * @brief Transformation matrix from the link coordinate system to the center of mass of the link.
   */
  std::vector<Eigen::Matrix4d> link_center_of_masses_;

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
  VelocityMethod velocity_method_ = APPROXIMATE;

  /**
   * @brief enclosing capsules for velocity calculation
   */
  std::vector<reach_lib::Capsule> robot_capsules_for_velocity_;

  /**
   * @brief list of z-vectors (third column of transformation matrix) for velocity calculation
   */
  std::vector<Eigen::Vector3d> z_vectors_;

  /**
   * @brief current transformation matrices
   */
  std::vector<Eigen::Matrix4d> current_transformation_matrices_;

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
   * @param link_masses the masses of the links
   * @param link_inertias the inertias of the links
   * @param link_center_of_masses the center of masses of the links
   * @param x initial x position of base
   * @param y initial y position of base
   * @param z initial z position of base
   * @param roll initial roll of base
   * @param pitch initial pitch of base
   * @param yaw initial yaw of base
   * @param secure_radius Expand the radius of the robot capsules by this amount to
   *  account for measurement and modelling errors.
   */
  RobotReach(std::vector<double> transformation_matrices, int nb_joints, std::vector<double> geom_par,
             std::vector<double> link_masses, std::vector<double> link_inertias, std::vector<double> link_center_of_masses,
             double x = 0, double y = 0, double z = 0,
             double roll = 0, double pitch = 0, double yaw = 0, double secure_radius = 0);

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
  inline void forwardKinematic(const double& q, const int& n_joint, Eigen::Matrix4d& T) const {
    // Transform T to new joint coordinate system
    T = T * transformation_matrices_[n_joint + 1];
    Eigen::Matrix4d Rz;
    Rz << cos(q), -sin(q), 0, 0, sin(q), cos(q), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    T = T * Rz;
  }

  inline Eigen::Vector4d pointToVector(const reach_lib::Point& p) const {
    Eigen::Vector4d vec;
    vec << p.x, p.y, p.z, 1.0;
    return vec;
  }

  inline Eigen::Vector3d pointTo3dVector(const reach_lib::Point& p) const {
    Eigen::Vector3d vec;
    vec << p.x, p.y, p.z;
    return vec;
  }

  inline reach_lib::Point vectorToPoint(const Eigen::Vector4d& vec) const {
    return reach_lib::Point(vec(0), vec(1), vec(2));
  }

  inline Eigen::Matrix4d xyzrpy2transformationMatrix(double x, double y, double z, double roll, double pitch, double yaw) const {
    Eigen::Matrix4d T;
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    T << cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy, x, sr * cp,
        sr * sp * sy + cr * cy, sr * sp * cy - cr * sy, y, -sp, cp * sy, cp * cy, z, 0, 0, 0, 1;
    return T;
  }

  /**
   * @brief Transform the capsule of joint n by the transformation matrix T.
   * @return the transformed capsule
   */
  reach_lib::Capsule transformCapsule(const int& n_joint, const Eigen::Matrix4d& T) const;

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
                                        std::vector<double> alpha_i) const;

  /**
   * @brief Calculate the reachable sets for each time interval motion[i] to motion[i+1]
   * 
   * @param motions The list of motions
   * @param alpha_i The maximum acceleration of each capsule point with resprect to the time parameter s for the
   * given path
   * @return list of reachable sets
   */
  std::vector<std::vector<reach_lib::Capsule>> reachTimeIntervals(std::vector<Motion> motions, std::vector<double> alpha_i) const;

  /**
   * @brief calculates maximum cartesian velocity for a specific robot configuration
   * @details calls calculateAllTransformationMatricesAndCapsules() before
   * @param motion configuration of robot
   * @return maximum cartesian velocity of motion
   */
  double maxVelocityOfMotion(const Motion& motion);

  /**
   * @brief Calculate the maximal error in the Cartesian velocity of all robot links.
   * Return the maximum error between two measurement points with temporal distance dt.
   * 
   * @param dt time difference between measurement points
   * @param dq_max maximum joint velocity
   * @param ddq_max maximum joint acceleration
   * @param dddq_max maximum joint jerk
   * @return std::vector<double> upper bound of the velocity error for each joint
   */
  std::vector<double> calculateMaxVelErrors(double dt,
   const std::vector<double>& dq_max, const std::vector<double>& ddq_max, const std::vector<double>& dddq_max) const;

  /**
   * @brief Calculate the maximal error in the Cartesian velocity of a single robot link.
   * Return the maximum error between two measurement points with temporal distance dt.
   * 
   * @param link_index index of the robot link
   * @param dt time difference between measurement points
   * @param dq_max maximum joint velocity
   * @param ddq_max maximum joint acceleration
   * @param dddq_max maximum joint jerk
   * @return double upper bound of the velocity error for each joint
   */
  double calculateMaxVelError(int link_index, double dt,
   const std::vector<double>& dq_max, const std::vector<double>& ddq_max, const std::vector<double>& dddq_max) const;

  /**
   * @brief Calculate all velocities for a specific robot configuration.
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @param[in] q_dot Joint velocities
   * @return std::vector<CapsuleVelocity> Vector of velocities in SE3 of both points of a capsule
   */
  std::vector<CapsuleVelocity> calculateAllCapsuleVelocities(const std::vector<double> q_dot) const;

  /**
   * @brief Calculate all inertia matrices for a specific robot configuration.
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @return std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> Inertia matrices of the links.
   */
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> calculateAllInertiaMatrices() const;

  /**
   * @brief Calculate all inverse translational mass matrices for a specific robot configuration.
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @return std::vector<Eigen::Matrix<double, 3, 3>> The inverse translational mass matrices of the links.
   */
  std::vector<Eigen::Matrix<double, 3, 3>> calculateAllInvMassMatrices() const;

  /**
   * @brief Calculate all maximum reflected masses for a specific robot configuration.
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @return std::vector<double> Maximum reflected masses of the links.
   */
  std::vector<double> calculateAllMaxReflectedMasses() const;

  /**
   * @brief Calculate the cartesian velocity of both defining point of a specific capsule
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @param capsule which capsule
   * @param q_dot velocity configuration of robot
   * @return Velocity in SE3 of both Capusle points
   */
  CapsuleVelocity calculateVelocityOfCapsule(const int capsule, std::vector<double> q_dot) const;

  /**
   * @brief Calculate the cartesian velocity of both defining point of a specific capsule given a pre-computed jacobian
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @param[in] capsule which capsule
   * @param[in] q_dot velocity configuration of robot
   * @param[in] jacobian Jacobian of the capsule
   * @return Velocity in SE3 of both Capusle points
   */
  CapsuleVelocity calculateVelocityOfCapsuleWithJacobian(const int capsule, std::vector<double> q_dot, const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian) const;

  /**
   * @brief Calculate the inertia matrix of a specific link
   * 
   * @param[in] i link index
   * @param[in] link_jacobians Jacobians of the robot links up until link i.
   * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia matrix of link i.
   */
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> calculateInertiaMatrix(const int i, const std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>>& link_jacobians) const;

  /**
   * @brief Calculate the inverse translational mass matrix of a specific link
   * 
   * @details Currently only works for the last link!
   * 
   * @param[in] link_jacobian Jacobians of the robot link i.
   * @param[in] inertia_matrix Inertia matrix of the robot link i.
   * @return Eigen::Matrix<double, 3, 3> inverse translational mass matrix of link i.
   */
  Eigen::Matrix<double, 3, 3> calculateInvMassMatrix(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& link_jacobian,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia_matrix
  ) const;

  /**
   * @brief Calculate the local inertia matrix of a specific link
   * @details B = m * J_P^T * J_P + J_O^T R I R^T J_O
   * 
   * @param J jacobian of the center of mass of the given link
   * @param m mass of the given link
   * @param R rotation matrix from the base system to the link system
   * @param I inertia tensor of the link
   * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> 
   */
  inline Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> calculateLocalIntertiaMatrix(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& J,
    double m,
    const Eigen::Matrix<double, 3, 3>& R,
    const Eigen::Matrix<double, 3, 3>& I
  ) const {
    return m * J.topRows(3).transpose() * J.topRows(3) +
      J.bottomRows(3).transpose() * R * I * R.transpose() * J.bottomRows(3);
  }

  /**
   * @brief Calculate the maximum norm of the cartesian velocity for a specific capsule
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @param capsule which capsule
   * @param q_dot velocity configuration of robot
   * @return maximum cartesian velocity of capsule
   */
  double calculateMaxVelocityOfCapsule(const int capsule, std::vector<double> q_dot) const;

  /**
   * @brief calculates all transformation matrices and capsules for a specific robot configuration and sets them
   * @param q vector of robot angles
   */
  void calculateAllTransformationMatricesAndCapsules(const std::vector<double>& q);

  /**
   * @brief Returns Jacobian of a point attached to the i-th joint.
   * @details You often take robot_capsules_for_velocity_[joint].p1_ or robot_capsules_for_velocity_[joint].p2_ as
   * point.
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @param joint Jacobian for which joint
   * @param point Jacobian for which point
   * @return Jacobian 6 x nb_joints
   */
  Eigen::Matrix<double, 6, Eigen::Dynamic> calculateJacobian(const int joint, const reach_lib::Point& point) const;

  /**
   * @brief computes approximate maximum cartesian velocity of a specific capsule
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @param capsule
   * @param v linear velocity at joint
   * @param omega angular velocity at joint
   * @return approximate maximum cartesian velocity of capsule
   */
  double approximateVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega) const;

  /**
   * @brief computes approximate maximum cartesian velocity of a specific capsule
   * @assumption calculateAllTransformationMatricesAndCapsules() was called before
   * @param capsule
   * @param v linear velocity at joint
   * @param omega angular velocity at joint
   * @return approximate maximum cartesian velocity of capsule
   */
  double exactVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega) const;

  /**
   * @brief Calculate the reflected masses of the robot links for each time interval.
   * 
   * @param robot_motions motions of the time intervals
   * @return std::vector<std::vector<double>> reflected masses of the robot links for each time interval
   */
  std::vector<std::vector<double>> calculateRobotLinkReflectedMassesPerTimeInterval(
    const std::vector<Motion>& robot_motions
  ) const;

   /**
   * @brief Calculate the reflected masses of the robot links
   * 
   * @param robot_motions motion of the robot
   * @return std::vector<double> reflected masses of the robot links
   */
  std::vector<double> calculateRobotLinkReflectedMasses(
    const Motion& robot_motion
  ) const;

  /**
   * @brief computes cross product as skew-symmetric matrix
   * @param vec
   * @return matrix
   */
  inline Eigen::Matrix3d getCrossProductAsMatrix(const Eigen::Vector3d& vec) const {
    Eigen::Matrix3d cross;
    cross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return cross;
  }

  /**
   * @brief returns robot capsules for velocity
   * @return robot capsules for velocity
   */
  inline std::vector<occupancy_containers::capsule::Capsule> getRobotCapsulesForVelocity() const {
    return robot_capsules_for_velocity_;
  }

  /**
   * @brief Get the Nb Joints object
   * 
   * @return int 
   */
  inline int getNbJoints() const {
    return nb_joints_;
  }

  /**
   * @brief sets velocity_method
   */
  inline void setVelocityMethod(VelocityMethod velocity_method) {
    velocity_method_ = velocity_method;
  }
};
}  // namespace safety_shield

#endif  // ROBOT_REACH_H
