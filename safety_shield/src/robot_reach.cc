#include "safety_shield/robot_reach.h"

namespace safety_shield {

RobotReach::RobotReach(std::vector<double> transformation_matrices, int nb_joints, std::vector<double> geom_par,
                       std::vector<double> link_masses, std::vector<double> link_inertias, std::vector<double> link_center_of_masses,
                       double x, double y, double z, double roll, double pitch, double yaw, double secure_radius)
    : nb_joints_(nb_joints), link_masses_(link_masses), secure_radius_(secure_radius) {
  Eigen::Matrix4d transformation_matrix_first = xyzrpy2transformationMatrix(x, y, z, roll, pitch, yaw);
  transformation_matrices_.push_back(transformation_matrix_first);
  for (int joint = 0; joint < nb_joints; joint++) {
    // Fill transformation matrix
    Eigen::Matrix4d transformation_matrix;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        transformation_matrix(i, j) = transformation_matrices[16 * joint + 4 * i + j];
      }
    }
    transformation_matrices_.push_back(transformation_matrix);
    link_lengths_.push_back(transformation_matrix.block<3,1>(0, 3).norm());
    // Fill capsules
    Eigen::Vector4d p1;
    Eigen::Vector4d p2;
    for (int i = 0; i < 3; i++) {
      p1(i) = geom_par[7 * joint + i];
      p2(i) = geom_par[7 * joint + 3 + i];
    }
    double radius = geom_par[7 * joint + 6];
    reach_lib::Capsule capsule(vectorToPoint(p1), vectorToPoint(p2), radius);
    robot_capsules_.push_back(capsule);
    // Build inertia matrices
    // link_inertias have form ixx_0, ixy_0, ixz_0, iyy_0, iyz_0, izz_0, ...
    Eigen::Matrix<double, 3, 3> inertia_matrix;
    inertia_matrix << link_inertias[6 * joint], link_inertias[6 * joint + 1], link_inertias[6 * joint + 2],
                      link_inertias[6 * joint + 1], link_inertias[6 * joint + 3], link_inertias[6 * joint + 4],
                      link_inertias[6 * joint + 2], link_inertias[6 * joint + 4], link_inertias[6 * joint + 5];
    link_inertias_.push_back(inertia_matrix);
    // Build center of masses
    // CoM have the form x_0, y_0, z_0, roll_0, pitch_0, yaw_0, ...
    link_center_of_masses_.push_back(xyzrpy2transformationMatrix(
      link_center_of_masses[6 * joint], link_center_of_masses[6 * joint + 1], link_center_of_masses[6 * joint + 2],
      link_center_of_masses[6 * joint + 3], link_center_of_masses[6 * joint + 4], link_center_of_masses[6 * joint + 5]
    ));
  }
}

void RobotReach::reset(double x, double y, double z, double roll, double pitch, double yaw) {
  transformation_matrices_[0] = xyzrpy2transformationMatrix(x, y, z, roll, pitch, yaw);
}

reach_lib::Capsule RobotReach::transformCapsule(const int& n_joint, const Eigen::Matrix4d& T) const {
  Eigen::Vector4d p1 = T * pointToVector(robot_capsules_[n_joint].p1_);
  Eigen::Vector4d p2 = T * pointToVector(robot_capsules_[n_joint].p2_);
  reach_lib::Capsule c(vectorToPoint(p1), vectorToPoint(p2), robot_capsules_[n_joint].r_);
  return c;
}

std::vector<reach_lib::Capsule> RobotReach::reach(Motion& start_config, Motion& goal_config, double s_diff,
                                                  std::vector<double> alpha_i) const {
  assert(alpha_i.size() == nb_joints_);
  Eigen::Matrix4d T_before = transformation_matrices_[0];
  Eigen::Matrix4d T_after = transformation_matrices_[0];
  std::vector<reach_lib::Capsule> reach_capsules;
  std::vector<double> q1 = start_config.getAngle();
  std::vector<double> q2 = goal_config.getAngle();
  for (int i = 0; i < nb_joints_; i++) {
    // build capsule before
    forwardKinematic(q1[i], i, T_before);
    reach_lib::Capsule before = transformCapsule(i, T_before);
    // build capsule after
    forwardKinematic(q2[i], i, T_after);
    reach_lib::Capsule after = transformCapsule(i, T_after);

    // Calculate center of ball enclosing point p1 before and after
    reach_lib::Point p_1_k = (before.p1_ + after.p1_) * 0.5;
    // Calculate center of ball enclosing point p2 before and after
    reach_lib::Point p_2_k = (before.p2_ + after.p2_) * 0.5;
    // Calculate radius of ball enclosing point p1 before and after
    double r_1 = reach_lib::Point::norm(before.p1_ - after.p1_) / 2.0 + alpha_i[i] * s_diff * s_diff / 8.0 +
                  robot_capsules_[i].r_;
    // Calculate radius of ball enclosing point p2 before and after
    double r_2 = reach_lib::Point::norm(before.p2_ - after.p2_) / 2.0 + alpha_i[i] * s_diff * s_diff / 8.0 +
                  robot_capsules_[i].r_;
    // Final radius is maximum of r_1 and r_2 plus the radius expansion for modelling errors.
    double radius = std::max(r_1, r_2) + secure_radius_;
    // Enclosure capsule radius is max of ball around p1 and ball around p2
    reach_capsules.push_back(reach_lib::Capsule(p_1_k, p_2_k, radius));
  }
  return reach_capsules;
}

/// get for each robot motion the corresponding reachable set and collect in list
std::vector<std::vector<reach_lib::Capsule>> RobotReach::reachTimeIntervals(std::vector<Motion> motions, std::vector<double> alpha_i) const {
  std::vector<std::vector<reach_lib::Capsule>> robot_reachable_sets;
  for(int i = 0; i < motions.size() - 1; i++) {
    std::vector<reach_lib::Capsule> timestep_i = reach(motions[i], motions[i+1], motions[i+1].getS() - motions[i].getS(), alpha_i);
    robot_reachable_sets.push_back(timestep_i);
  }
  return robot_reachable_sets;
}

double RobotReach::maxVelocityOfMotion(const Motion& motion) {
  calculateAllTransformationMatricesAndCapsules(motion.getAngleRef());
  double max_capsule_vel = 0;
  for (int i = 0; i < nb_joints_; ++i) {
    double capsule_vel = calculateMaxVelocityOfCapsule(i, motion.getVelocityRef());
    max_capsule_vel = std::max(max_capsule_vel, capsule_vel);
  }
  return max_capsule_vel;
}

std::vector<double> RobotReach::calculateMaxVelErrors(
  double dt, const std::vector<double>& dq_max,
  const std::vector<double>& ddq_max, const std::vector<double>& dddq_max) const {
  std::vector<double> max_vel_errors;
  for (int i = 0; i < nb_joints_; i++) {
    max_vel_errors.push_back(calculateMaxVelError(i, dt, dq_max, ddq_max, dddq_max));
  }
  return max_vel_errors;
}

double RobotReach::calculateMaxVelError(int link_index, double dt, const std::vector<double>& dq_max,
  const std::vector<double>& ddq_max, const std::vector<double>& dddq_max) const {
  double max_vel_error = 0;
  int N = link_index + 1;
  for (int k = 0; k < N; k++) {
    for (int i = k; i < N; i++) {
      // sum up the vector dq_max from 0 to i-1
      double dq_sum = 0;
      for (int j = 0; j <= i; j++) {
        dq_sum += dq_max[j];
      }
      double ddq_sum = 0;
      for (int j = 0; j <= i; j++) {
        ddq_sum += ddq_max[j];
      }
      max_vel_error += link_lengths_[i] * (dddq_max[k] + ddq_max[k] * dq_sum + dq_max[k] * (ddq_sum + dq_sum * dq_sum));
    }
  }
  max_vel_error *= dt*dt/8.0;
  return max_vel_error;
}

void RobotReach::calculateAllTransformationMatricesAndCapsules(const std::vector<double>& q) {
  robot_capsules_for_velocity_.clear();
  z_vectors_.clear();
  current_transformation_matrices_.clear();
  Eigen::Matrix4d T_capsule = transformation_matrices_[0];
  current_transformation_matrices_.push_back(T_capsule);
  // push z-vector of transformation matrix
  z_vectors_.emplace_back(T_capsule.block(0, 2, 3, 1));
  for (int i = 0; i < nb_joints_; ++i) {
    forwardKinematic(q[i], i, T_capsule);
    reach_lib::Capsule capsule = transformCapsule(i, T_capsule);
    // push robot capsule and z-vector
    robot_capsules_for_velocity_.push_back(capsule);
    z_vectors_.emplace_back(T_capsule.block(0, 2, 3, 1));
    Eigen::Matrix4d deep_copy_T = T_capsule;
    current_transformation_matrices_.push_back(deep_copy_T);
  }
}

std::vector<RobotReach::CapsuleVelocity> RobotReach::calculateAllCapsuleVelocities(const std::vector<double> q_dot) const {
  assert(q_dot.size() == nb_joints_);  // Number of joints should match
  std::vector<CapsuleVelocity> velocities;
  for (int i = 0; i < nb_joints_; i++) {
    velocities.push_back(calculateVelocityOfCapsule(i, q_dot));
  }
  return velocities;
}

std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> RobotReach::calculateAllInertiaMatrices(const std::vector<double> q_dot) const {
  assert(q_dot.size() == nb_joints_);  // Number of joints should match
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> inertia_matrices;
  std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> link_jacobians(nb_joints_);
  for (int i = 0; i < nb_joints_; i++) {
    reach_lib::Point CoM = robot_capsules_for_velocity_[i].p1_ + vectorToPoint((current_transformation_matrices_[i] * link_center_of_masses_[i]).col(3));
    link_jacobians[i] = calculateJacobian(i, CoM);
    inertia_matrices.push_back(calculateInertiaMatrix(i, link_jacobians));
  }
  return inertia_matrices;
}

RobotReach::CapsuleVelocity RobotReach::calculateVelocityOfCapsule(const int capsule, std::vector<double> q_dot) const {
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = calculateJacobian(capsule, robot_capsules_for_velocity_[capsule].p1_);
  return calculateVelocityOfCapsuleWithJacobian(capsule, q_dot, jacobian);
}

RobotReach::CapsuleVelocity RobotReach::calculateVelocityOfCapsuleWithJacobian(const int capsule, std::vector<double> q_dot, const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian) const {
  Eigen::VectorXd velocity = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_dot.data(), q_dot.size());
  Eigen::Vector<double, 6> result1 = jacobian * velocity;
  RobotReach::SE3Vel vel1(result1.segment(0, 3), result1.segment(3, 3));
  // Point 2: v_2 = v_1 + \omega_1 \times (p_2 - p_1), \omega_2 = \omega_1
  RobotReach::SE3Vel vel2(
    vel1.v + vel1.w.cross(pointTo3dVector(
      robot_capsules_for_velocity_[capsule].p2_ - robot_capsules_for_velocity_[capsule].p1_
    )),
    vel1.w);
  return RobotReach::CapsuleVelocity(vel1, vel2);
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RobotReach::calculateInertiaMatrix(const int i, const std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>>& link_jacobians) const {
  assert(i < nb_joints_);
  assert(link_jacobians.size() > i);  // Jacobians should be calculated up to the i-th link.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia_matrix;
  // J_P = link_jacobians[i](Eigen::seq(0,3), Eigen::all)
  // J_O = link_jacobians[i](Eigen::seq(3,6), Eigen::all)
  // R = current_transformation_matrices_[i].block(0, 0, 3, 3)
  inertia_matrix = calculateLocalIntertiaMatrix(link_jacobians[0], link_masses_[0], current_transformation_matrices_[0].block(0, 0, 3, 3), link_inertias_[0]);
  for (int j = 1; j <= nb_joints_; j++) {
    auto J = (j < i) ? link_jacobians[j] : link_jacobians[i];
    inertia_matrix = inertia_matrix + calculateLocalIntertiaMatrix(J, link_masses_[j], current_transformation_matrices_[j].block(0, 0, 3, 3), link_inertias_[j]);
  }
  return inertia_matrix;
}

double RobotReach::calculateMaxVelocityOfCapsule(const int capsule, std::vector<double> q_dot) const {
  RobotReach::CapsuleVelocity capsule_velocity = calculateVelocityOfCapsule(capsule, q_dot);
  if (velocity_method_ == APPROXIMATE) {
    return approximateVelOfCapsule(capsule, capsule_velocity.v2.v, capsule_velocity.v2.w);
  } else {
    // velocity_method_ == EXACT
    return exactVelOfCapsule(capsule, capsule_velocity.v2.v, capsule_velocity.v2.w);
  }
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotReach::calculateJacobian(const int joint, const reach_lib::Point& point) const {
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
  jacobian.setZero(6, nb_joints_);
  Eigen::Vector3d p_e = pointTo3dVector(point);
  for (int i = 0; i < joint + 1; ++i) {
    Eigen::Vector3d z_i = z_vectors_[i + 1];
    Eigen::Vector3d p_i = pointTo3dVector(robot_capsules_for_velocity_[i].p1_);
    Eigen::Vector3d upper = z_i.cross(p_e - p_i);
    Eigen::Vector<double, 6> column;
    column << upper, z_i;
    jacobian.col(i) = column;
  }
  return jacobian;
}

double RobotReach::approximateVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega) const {
  Eigen::Vector3d p1 = pointTo3dVector(robot_capsules_for_velocity_[capsule].p1_);
  Eigen::Vector3d p2 = pointTo3dVector(robot_capsules_for_velocity_[capsule].p2_);
  Eigen::Vector3d link = p1 - p2;
  // radius of capsule
  double r = robot_capsules_for_velocity_[capsule].r_;
  // maximum velocity of capsule surface at second endpoint
  double q2 = v.norm() + omega.norm() * r;
  // maximum velocity of capsule surface at first endpoint
  double q1 = (v + omega.cross(link)).norm() + omega.norm() * r;
  // take maximum of both
  return std::max(q1, q2);
}

double RobotReach::exactVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega) const {
  Eigen::Vector3d n = omega.normalized();
  double scalar_v = v.transpose() * n;
  Eigen::Vector3d p1 = pointTo3dVector(robot_capsules_for_velocity_[capsule].p1_);
  Eigen::Vector3d p2 = pointTo3dVector(robot_capsules_for_velocity_[capsule].p2_);
  //  LSE solving to get offset o of screw axis: Ax = b, b = v - scalar_v * n, A = S(omega), o = -(x - p2)
  Eigen::Matrix3d A = getCrossProductAsMatrix(omega);
  Eigen::Vector3d b = v - (v.transpose() * n) * n;
  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  Eigen::Vector3d o = p2 - x;
  // perpendicular distance between pi and screw axis
  Eigen::Vector3d p1_cross = n.cross(p1 - o);
  Eigen::Vector3d p2_cross = n.cross(p2 - o);
  // radius of capsule
  double r = robot_capsules_for_velocity_[capsule].r_;
  // get maximum distance + radius of capsule
  double d_perp = std::max(p1_cross.norm(), p2_cross.norm()) + r;
  double right_current = std::abs(omega.norm()) * d_perp;
  // norm of angular and linear velocity
  return std::sqrt(scalar_v * scalar_v + right_current * right_current);
}

std::vector<std::vector<double>> RobotReach::calculateRobotLinkReflectedMassesPerTimeInterval(
  const std::vector<Motion>& robot_motions
) const {
  std::vector<std::vector<double>> masses;
  for (int i = 0; i < robot_motions.size(); i++) {
    masses.push_back(calculateRobotLinkReflectedMasses(robot_motions[i]));
  }
  return masses;
}

std::vector<double> RobotReach::calculateRobotLinkReflectedMasses(
  const Motion& robot_motion
) const {
  // TODO implement this function.
  double m = 10;
  std::vector<double> masses;
  for (int i = 0; i < nb_joints_; i++) {
    masses.push_back(m);
  }
  return masses;
}

}  // namespace safety_shield