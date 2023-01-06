#include "safety_shield/long_term_traj.h"

namespace safety_shield {

Motion LongTermTraj::interpolate(double s, double ds, double dds, double ddds, 
        std::vector<double>& v_max_allowed, std::vector<double>& a_max_allowed, std::vector<double>& j_max_allowed) {
  // Example: s=2.465, sample_time = 0.004 --> ind = 616.25
  assert(sample_time_ != 0);
  double ind = s/sample_time_;
  double intpart;
  // Example: intpart = 616.0, ind_mod = 0.25
  double ind_mod = modf(ind, &intpart);
  // floor(s/sample_time) + 1 ==> lower index
  int ind1 = static_cast<int>(intpart);
  // ceil(s/sample_time) + 1 ==> upper index
  int ind2 = static_cast<int>(ceil(ind));
  // time from first index to interpolation point
  double dt = ind_mod * sample_time_;
  std::vector<double> q1 = getNextMotionAtIndex(ind1).getAngle();
  std::vector<double> dq1 = getNextMotionAtIndex(ind1).getVelocity();
  std::vector<double> ddq1 = getNextMotionAtIndex(ind1).getAcceleration();
  std::vector<double> dddq1 = getNextMotionAtIndex(ind1).getJerk();
  std::vector<double> q(q1.size());
  std::vector<double> dq(q1.size());
  std::vector<double> ddq(q1.size());
  std::vector<double> dddq(q1.size());
  for (int i = 0 ; i < q1.size(); i++) {
      // Linearly interpolate between lower and upper index of position
      q[i] = q1[i] + dt * dq1[i] + 1.0/2 *dt*dt * ddq1[i] + 1.0/6 * dt*dt*dt * dddq1[i];
      // Calculate LTT velocity
      double v_max_int = dq1[i] + dt * ddq1[i] + 1.0/2 * dt*dt * dddq1[i];
      double v_int = v_max_int * ds;
      dq[i] = std::clamp(v_int, -v_max_allowed[i], v_max_allowed[i]);
      // Calculate Acceleration
      double a_max_int = ddq1[i] + dt * dddq1[i];
      double a_int = v_max_int * dds + ds * ds * a_max_int;
      ddq[i] = std::clamp(a_int, -a_max_allowed[i], a_max_allowed[i]);
      dddq[i] = dddq1[i] * ds * ds * ds + 3.0 * a_max_int * dds * ds + v_max_int * ddds;
  }
  return Motion(0.0, q, dq, ddq, dddq, s);
}

void LongTermTraj::calculate_max_acc_jerk_window(std::vector<Motion> &long_term_traj, int k) {
  int traj_length = long_term_traj.size();
  // It must be k <= trajectory length.
  k = std::min(traj_length, k);
  int n_joints = long_term_traj[0].getAngle().size();
  max_acceleration_window_.reserve(traj_length);
  max_jerk_window_.reserve(traj_length);

  // We use method 3 of https://www.geeksforgeeks.org/sliding-window-maximum-maximum-of-all-subarrays-of-size-k/
  std::vector<std::deque<int>> max_queue_acc;
  std::vector<std::deque<int>> max_queue_jerk;
  for (int j=0; j<n_joints; ++j) {
    max_queue_acc.push_back(std::deque<int>() );
    max_queue_jerk.push_back(std::deque<int>() );
  }
  /* Process first k (or first window)
    elements of array */
  int i;
  for (i = 0; i < k; ++i) {
    for (int j=0; j<n_joints; ++j) {
      // ACCELERATION
      // For every element, the previous smaller elements are useless so remove them from queue
      while (!max_queue_acc[j].empty() && long_term_traj[i].getAcceleration()[j] >= 
          long_term_traj[max_queue_acc[j].back()].getAcceleration()[j]) {
        // Remove from rear
        max_queue_acc[j].pop_back();
      }
      // Add new element at rear of queue
      max_queue_acc[j].push_back(i);

      // JERK
      while ((!max_queue_jerk[j].empty()) && long_term_traj[i].getJerk()[j] >= 
          long_term_traj[max_queue_jerk[j].back()].getJerk()[j]) {
        max_queue_jerk[j].pop_back();
      }
      max_queue_jerk[j].push_back(i);
    }
    
  }
  // Process rest of the elements,
  // i.e., from arr[k] to arr[n-1]
  for (; i < traj_length+k; ++i) {
    std::vector<double> max_acc; std::vector<double> max_jerk;
    max_acc.reserve(n_joints); max_jerk.reserve(n_joints);
    for (int j=0; j<n_joints; ++j) {
      // ACCELERATION
      // The element at the front of the queue is the largest element of previous window
      max_acc.push_back(long_term_traj[max_queue_acc[j].front()].getAcceleration()[j]);
      // Remove the elements which are out of this window
      while ((!max_queue_acc[j].empty()) && max_queue_acc[j].front() <= i - k) {
        // Remove from front of queue
        max_queue_acc[j].pop_front();
      }
      if (i<traj_length) {
        // Remove all elements smaller than the currently being added element (remove useless elements)
        while ((!max_queue_acc[j].empty()) && long_term_traj[i].getAcceleration()[j] >= long_term_traj[max_queue_acc[j].back()].getAcceleration()[j]) {
          max_queue_acc[j].pop_back();
        }
        // Add current element at the rear of Qi
        max_queue_acc[j].push_back(i);
      }

      // JERK
      max_jerk.push_back(long_term_traj[max_queue_jerk[j].front()].getJerk()[j]);
      while ((!max_queue_jerk[j].empty()) && max_queue_jerk[j].front() <= i - k) {
        max_queue_jerk[j].pop_front();
      }
      if (i<traj_length) {
        while ((!max_queue_jerk[j].empty()) && long_term_traj[i].getJerk()[j] >= long_term_traj[max_queue_jerk[j].back()].getJerk()[j]) {
          max_queue_jerk[j].pop_back();
        }
        max_queue_jerk[j].push_back(i);
      }
    }
    max_acceleration_window_.push_back(max_acc);
    max_jerk_window_.push_back(max_jerk);
  }
}

double LongTermTraj::getMaxofMaximumCartesianVelocityWithLength(unsigned long index, unsigned long length) {
    unsigned long last = index + length + 1;
    double max = getMotion(index).getMaximumCartesianVelocity();
    for(unsigned long i = index + 1; i < last; i++) {
        max = std::max(max, getMotion(i).getMaximumCartesianVelocity());
    }
    return max;
}

double LongTermTraj::getMaxofMaximumCartesianVelocityWithS(double s) {
    unsigned long i = getCurrentPos();
    double max = getMotion(i).getMaximumCartesianVelocity();
    while(true) {
        max = std::max(max, getMotion(i).getMaximumCartesianVelocity());
        if(getMotion(i).getTime() > s) {
            return max;
        }
        ++i;
    }
}

Eigen::Matrix<double, 6, Eigen::Dynamic> LongTermTraj::getJacobian(pinocchio::JointIndex joint) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
    jacobian.resize(6, long_term_traj_[0].getNbModules());
    jacobian.setZero();
    pinocchio::getJointJacobian(model_, data_, joint, pinocchio::WORLD, jacobian);
    return jacobian;
}

Eigen::Matrix3d LongTermTraj::getCrossProductAsMatrix(Eigen::Vector3d vec) {
    Eigen::Matrix3d cross;
    cross <<    0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
    return cross;
}

// TODO: dauert zu lange
void LongTermTraj::calculateExactMaxCartesianVelocity(RobotReach& robot_reach) {
    if(!velocityCalculation_) {
        return;
    }
    std::vector<reach_lib::Capsule> robot_capsules;
    double epsilon = 1e-6;
    for (unsigned long i = 0; i < getLength(); i++) {
        Motion& motion = long_term_traj_[i];
        // TODO: forward kinematics with pinocchio --> get cartesian coordinates of Link Points + radius of capsule
        robot_capsules = robot_reach.reach(motion, motion, 0, alpha_i_);
        Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getAngle().data(),
                                                                          motion.getAngle().size());
        Eigen::VectorXd q_dot = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getVelocity().data(),
                                                                              motion.getVelocity().size());
        pinocchio::forwardKinematics(model_, data_, q, q_dot);
        pinocchio::computeJointJacobians(model_, data_, q);
        double max = 0;
        for (unsigned long j = 0; j <= motion.getAngle().size(); j++) {
            auto jacobian = getJacobian(j);
            Eigen::VectorXd velocity = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getVelocity().data(),
                                                                                         motion.getVelocity().size());
            Eigen::Vector<double, 6> result = jacobian * velocity;
            Eigen::Vector3d v = result.segment(0, 3);
            Eigen::Vector3d w = result.segment(3, 3);
            double scalar_w = w.norm();
            if(scalar_w < epsilon) {
                // no angular velocity
                max = std::max(v.norm(), max);
            } else {
                // with angular velocity
                Eigen::Vector3d n = w / scalar_w;
                double scalar_v = v.transpose() * n;
                Eigen::Vector3d p1(robot_capsules[j].p1_.x, robot_capsules[j].p1_.y, robot_capsules[j].p1_.z);
                Eigen::Vector3d p2(robot_capsules[j].p2_.x, robot_capsules[j].p2_.y, robot_capsules[j].p2_.z);
                // Ax = b
                // b = v - scalar_v * n
                // A = S(w)
                // o = -(x - p2)
                Eigen::Matrix3d A = getCrossProductAsMatrix(w);
                Eigen::Vector3d b = v - scalar_v * n;
                Eigen::Vector3d x = A.partialPivLu().solve(b); //colPivHouseholderQr()
                Eigen::Vector3d o = -(x - p2);
                Eigen::Vector3d p1_cross = getCrossProductAsMatrix(n) * (p1 - o);
                Eigen::Vector3d p2_cross = getCrossProductAsMatrix(n) * (p2 - o);
                double r = robot_capsules[j].r_ + robot_reach.getSecureRadius();
                double d_perp = std::max(p1_cross.norm(), p2_cross.norm()) + r;
                double right_current = std::abs(scalar_w) * d_perp;
                double current = std::sqrt(scalar_v * scalar_v + right_current * right_current);
                max = std::max(max, current);
            }
        }
        motion.setMaximumCartesianVelocity(max);
    }
}

// TODO: dauert auch zu lange
void LongTermTraj::calculateApproximateMaxCartesianVelocity(RobotReach& robot_reach) {
    if(!velocityCalculation_) {
        return;
    }
    std::vector<reach_lib::Capsule> robot_capsules;
    double epsilon = 1e-6;
    for (unsigned long i = 0; i < getLength(); i++) {
        Motion& motion = long_term_traj_[i];
        robot_capsules = robot_reach.reach(motion, motion, 0, alpha_i_);
        Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getAngle().data(), motion.getAngle().size());
        Eigen::VectorXd q_dot = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getVelocity().data(), motion.getVelocity().size());
        pinocchio::forwardKinematics(model_, data_, q, q_dot);
        pinocchio::computeJointJacobians(model_, data_, q);
        double max = 0;
        for (unsigned long j = 0; j <= motion.getNbModules(); j++) {
            auto jacobian = getJacobian(j);
            Eigen::VectorXd velocity = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getVelocity().data(),
                                                                                     motion.getVelocity().size());
            Eigen::Vector<double, 6> result = jacobian * velocity;
            Eigen::Vector3d v = result.segment(0, 3);
            Eigen::Vector3d w = result.segment(3, 3);
            double scalar_w = w.norm();
            //std::cout << jacobian << std::endl;
            if(scalar_w < epsilon) {
                // no angular velocity
                max = std::max(max, v.norm());
            } else {
                // with angular velocity
                // vector of Link: r1 = p2 - p1
                Eigen::Vector3d p1(robot_capsules[j].p1_.x, robot_capsules[j].p1_.y, robot_capsules[j].p1_.z); //data_.oMi[j]; model_.jointPlacements;
                Eigen::Vector3d p2(robot_capsules[j].p2_.x, robot_capsules[j].p2_.y, robot_capsules[j].p2_.z); //data.oMi[j+1];
                Eigen::Vector3d link = p2 - p1;
                double r = robot_capsules[j].r_ + robot_reach.getSecureRadius();
                double q1 = v.norm() + w.norm() * r;
                double q2 = (v + getCrossProductAsMatrix(w) * link).norm() + scalar_w * r;
                max = std::max(max, q1);
                max = std::max(max, q2);
            }
        }
        motion.setMaximumCartesianVelocity(max);
    }
}

// TODO: iterative allKinematics function
void LongTermTraj::allKinematics(RobotReach& robotReach, Motion& motion, std::vector<Eigen::Matrix4d> transformation_matrices_q, std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobians) {
    Eigen::Matrix4d T = robotReach.getTransformationMatrices()[0];
    transformation_matrices_q.push_back(T);
    std::vector<double> q = motion.getAngle();
    for(int i = 0; i < motion.getAngle().size(); i++) {
        robotReach.forwardKinematic(q[i], i, T);
        transformation_matrices_q.push_back(T);
        // TODO: segfault, maybe cant save this type of list --> save current jacobi matrix + transformation_matrices in a list
        Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
        jacobian.setZero(6, i+1);
        //std::cout << "jacobian" << std::endl;
        //std::cout << jacobian << std::endl;
        for(int j = 0; j <= i; j++) {
            // z_j x (p_j+1 - p_j) over z_j
            Eigen::Vector3d z_j = transformation_matrices_q[j].block(0,2,3,1);
            Eigen::Vector3d p_j = transformation_matrices_q[j].block(0,3,3,1);
            Eigen::Vector3d p_j_next = transformation_matrices_q[j+1].block(0,3,3,1);
            Eigen::Vector3d over = getCrossProductAsMatrix(z_j) * (p_j_next - p_j);
            Eigen::Vector<double, 6> column;
            column << over, z_j;
            // TODO: does not allow resize?
            //std::cout << "column-vector" << std::endl;
            //std::cout << column << std::endl;
            jacobian.col(j) = column;
        }
    }
}

void LongTermTraj::myCalculateApproximateMaxCartesianVelocity(RobotReach& robot_reach) {
    if(!velocityCalculation_) {
        return;
    }
    double epsilon = 1e-6;
    for (unsigned long i = 0; i < getLength(); i++) {
        Motion& motion = long_term_traj_[i];
        std::vector<Eigen::Matrix4d> transformation_matrices_q;
        Eigen::Matrix4d T = robot_reach.getTransformationMatrices()[0];
        transformation_matrices_q.push_back(T);
        Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getAngle().data(), motion.getAngle().size());
        Eigen::VectorXd q_dot = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getVelocity().data(), motion.getVelocity().size());
        double max = 0;
        for (unsigned long j = 0; j < q.size(); j++) {
            robot_reach.forwardKinematic(q[j], j, T);
            transformation_matrices_q.push_back(T);
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
            jacobian.setZero(6, j+1);
            for(int k = 0; k <= j; k++) {
                // z_k x (p_k+1 - p_k) over k_j
                Eigen::Vector3d z_k = transformation_matrices_q[k].block(0,2,3,1);
                Eigen::Vector3d p_k = transformation_matrices_q[k].block(0,3,3,1);
                Eigen::Vector3d p_k_next = transformation_matrices_q[k+1].block(0,3,3,1);
                Eigen::Vector3d over = getCrossProductAsMatrix(z_k) * (p_k_next - p_k);
                Eigen::Vector<double, 6> column;
                column << over, z_k;
                jacobian.col(k) = column;
            }
            // jacobian j, transformation_matrices j - 1
            Eigen::VectorXd velocity = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motion.getVelocity().data(),
                                                                                     motion.getVelocity().size());
            Eigen::Vector<double, 6> result = jacobian * velocity.segment(0, j+1);
            Eigen::Vector3d v = result.segment(0, 3);
            Eigen::Vector3d w = result.segment(3, 3);
            double scalar_w = w.norm();
            if(scalar_w < epsilon) {
                // no angular velocity
                max = std::max(max, v.norm());
            } else {
                // with angular velocity
                // vector of Link: r1 = p2 - p1, p2 from transformation_matrix_j and p1 from transformation_matrix_j-1
                Eigen::Vector3d p1 = transformation_matrices_q[j].block(0,3,3,1);
                Eigen::Vector3d p2 = transformation_matrices_q[j+1].block(0,3,3,1);;
                Eigen::Vector3d link = p2 - p1;
                double r = robot_reach.getRobotCapsules()[j].r_ + robot_reach.getSecureRadius();
                double q1 = v.norm() + w.norm() * r;
                double q2 = (v + getCrossProductAsMatrix(w) * link).norm() + scalar_w * r;
                max = std::max(max, q1);
                max = std::max(max, q2);
            }
        }
        motion.setMaximumCartesianVelocity(max);
    }
}

} // namespace safety_shield