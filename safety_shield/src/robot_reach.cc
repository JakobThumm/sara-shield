#include "safety_shield/robot_reach.h"

namespace safety_shield {

RobotReach::RobotReach(std::vector<double> transformation_matrices, int nb_joints, std::vector<double> geom_par,
                       double x, double y, double z, double roll, double pitch, double yaw,
                       double secure_radius)
    : nb_joints_(nb_joints), secure_radius_(secure_radius) {
  Eigen::Matrix4d transformation_matrix_first;
  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);
  transformation_matrix_first << cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy, x, sr * cp,
      sr * sp * sy + cr * cy, sr * sp * cy - cr * sy, y, -sp, cp * sy, cp * cy, z, 0, 0, 0, 1;
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
  }
}

void RobotReach::reset(double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Matrix4d transformation_matrix;
  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);
  transformation_matrix << cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy, x, sr * cp, sr * sp * sy + cr * cy,
      sr * sp * cy - cr * sy, y, -sp, cp * sy, cp * cy, z, 0, 0, 0, 1;
  transformation_matrices_[0] = transformation_matrix;
}

reach_lib::Capsule RobotReach::transformCapsule(const int& n_joint, const Eigen::Matrix4d& T) {
  Eigen::Vector4d p1 = T * pointToVector(robot_capsules_[n_joint].p1_);
  Eigen::Vector4d p2 = T * pointToVector(robot_capsules_[n_joint].p2_);
  reach_lib::Capsule c(vectorToPoint(p1), vectorToPoint(p2), robot_capsules_[n_joint].r_);
  return c;
}

std::vector<reach_lib::Capsule> RobotReach::reach(Motion& start_config, Motion& goal_config, double s_diff,
                                                  std::vector<double> alpha_i) {
  try {
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
      double r_1 =
          reach_lib::Point::norm(before.p1_ - after.p1_) / 2.0 + alpha_i[i] * s_diff * s_diff / 8.0 + robot_capsules_[i].r_;
      // Calculate radius of ball enclosing point p2 before and after
      double r_2 = reach_lib::Point::norm(before.p2_ - after.p2_) / 2.0 + alpha_i[i] * s_diff * s_diff / 8.0 +
                   robot_capsules_[i].r_;
      // Final radius is maximum of r_1 and r_2 plus the radius expansion for modelling errors.
      double radius = std::max(r_1, r_2) + secure_radius_;
      // Enclosure capsule radius is max of ball around p1 and ball around p2
      reach_capsules.push_back(reach_lib::Capsule(p_1_k, p_2_k, radius));
    }
    return reach_capsules;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in RobotReach::reach: {}", exc.what());
    return {};
  }
}

double RobotReach::maxVelocityOfMotion(const Motion& motion) {
  calculateAllTransformationMatricesAndCapsules(motion.getAngleRef());
  double max_capsule_vel = 0;
  for (int i = 0; i < nb_joints_; ++i) {
    double capsule_vel = getMaxVelocityOfCapsule(i, motion.getVelocityRef());
    max_capsule_vel = std::max(max_capsule_vel, capsule_vel);
  }
  return max_capsule_vel;
}

void RobotReach::calculateAllTransformationMatricesAndCapsules(const std::vector<double>& q) {
  robot_capsules_for_velocity_.clear();
  z_vectors_.clear();
  Eigen::Matrix4d T_capsule = transformation_matrices_[0];
  // push z-vector of transformation matrix
  z_vectors_.emplace_back(T_capsule.block(0, 2, 3, 1));
  for (int i = 0; i < nb_joints_; ++i) {
    forwardKinematic(q[i], i, T_capsule);
    reach_lib::Capsule capsule = transformCapsule(i, T_capsule);
    // push robot capsule and z-vector
    robot_capsules_for_velocity_.push_back(capsule);
    z_vectors_.emplace_back(T_capsule.block(0, 2, 3, 1));
  }
}

RobotReach::CapsuleVelocity RobotReach::getVelocityOfCapsule(const int capsule, std::vector<double> q_dot){
  Eigen::VectorXd velocity = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_dot.data(), q_dot.size());
  // Point 1
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian1 = getJacobian(capsule, robot_capsules_for_velocity_[capsule].p1_);
  Eigen::Vector<double, 6> result1 = jacobian1 * velocity.segment(0, capsule + 1);
  RobotReach::SE3Vel vel1(result1.segment(0, 3), result1.segment(3, 3));
  // Point 2: v_2 = v_1 + \omega_1 \times (p_2 - p_1), \omega_2 = \omega_1
  RobotReach::SE3Vel vel2(
    vel1.first + vel1.second.cross(pointTo3dVector(
      robot_capsules_for_velocity_[capsule].p2_ - robot_capsules_for_velocity_[capsule].p1_
    )),
    vel1.second);
  return RobotReach::CapsuleVelocity(vel1, vel2);
}

double RobotReach::getMaxVelocityOfCapsule(const int capsule, std::vector<double> q_dot) {
  RobotReach::CapsuleVelocity capsule_velocity = getVelocityOfCapsule(capsule, q_dot);
  if (velocity_method_ == APPROXIMATE) {
    return approximateVelOfCapsule(capsule, capsule_velocity.second.first, capsule_velocity.second.second);
  } else {
    // velocity_method_ == EXACT
    return exactVelOfCapsule(capsule, capsule_velocity.second.first, capsule_velocity.second.second);
  }
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotReach::getJacobian(const int joint, const reach_lib::Point& point) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
  jacobian.setZero(6, joint + 1);
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

double RobotReach::approximateVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega) {
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

double RobotReach::exactVelOfCapsule(const int capsule, const Eigen::Vector3d& v, const Eigen::Vector3d& omega) {
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

}  // namespace safety_shield