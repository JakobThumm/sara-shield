#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>


namespace robotParams {

inline void schunk(pinocchio::Model &model) {
  // Joint 0
  Eigen::Matrix3d rotation_matrix0;
  rotation_matrix0 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d translation_vector0(0.0, 0.0, 0.0);
  pinocchio::SE3 joint_placement0(rotation_matrix0, translation_vector0);
  pinocchio::JointModelRZ joint0;
  auto joint0_id = model.addJoint(0, joint0, joint_placement0, "Link_0_jnt");

  // Append Body to Link
  Eigen::Matrix3d rotation_matrix_link0;
  rotation_matrix_link0 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d translation_vector_link0(0.0, 0.0193, 0.0833);
  Eigen::Matrix3d inertia_matrix0;
  inertia_matrix0 << 0.0153118, 0.0, 0.0, 0.0, 0.0097334, 0.0057564, 0.0, 0.0057564, 0.0115774;
  pinocchio::Inertia inertia0(3.9, Eigen::Vector3d(0.0, 0.0193, 0.0833), inertia_matrix0);
  model.appendBodyToJoint(joint0_id, inertia0, pinocchio::SE3::Identity());

  // Joint 1
  Eigen::Matrix3d rotation_matrix1;
  rotation_matrix1 << 1.0, 0.0, 0.0, 0.0, -3.205103454691839e-09, 1.0, 0.0, -1.0, -3.205103454691839e-09;
  Eigen::Vector3d translation_vector1(0.0, 0.1013, 0.1013);
  pinocchio::SE3 joint_placement1(rotation_matrix1, translation_vector1);
  pinocchio::JointModelRZ joint1;
  auto joint1_id = model.addJoint(1, joint1, joint_placement1, "Link_1_jnt");

  // Append Body to Link
  Eigen::Matrix3d rotation_matrix_link1;
  rotation_matrix_link1 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d translation_vector_link1(0.0, -0.175, 0.0132);
  Eigen::Matrix3d inertia_matrix1;
  inertia_matrix1 << 0.02444323, 1e-06, -6e-06, 1e-06, 0.01684973, -2e-07, -6e-06, -2e-07, 0.0247255;
  pinocchio::Inertia inertia1(1.62, Eigen::Vector3d(0.0, -0.175, 0.0132), inertia_matrix1);
  model.appendBodyToJoint(joint1_id, inertia1, pinocchio::SE3::Identity());

  // Joint 2
  Eigen::Matrix3d rotation_matrix2;
  rotation_matrix2 << 1.0, 0.0, 0.0, 0.0, -1.0, 3.5897930298416118e-09, 0.0, -3.5897930298416118e-09, -1.0;
  Eigen::Vector3d translation_vector2(-0.0, -0.35, 0.0);
  pinocchio::SE3 joint_placement2(rotation_matrix2, translation_vector2);
  pinocchio::JointModelRZ joint2;
  auto joint2_id = model.addJoint(2, joint2, joint_placement2, "Link_2_jnt");

  // Append Body to Link
  Eigen::Matrix3d rotation_matrix_link2;
  rotation_matrix_link2 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d translation_vector_link2(0.0, 0.0193, 0.0833);
  Eigen::Matrix3d inertia_matrix2;
  inertia_matrix2 << 0.0153118, 0.0, 0.0, 0.0, 0.0097334, 0.0057564, 0.0, 0.0057564, 0.0115774;
  pinocchio::Inertia inertia2(3.9, Eigen::Vector3d(0.0, 0.0193, 0.0833), inertia_matrix2);
  model.appendBodyToJoint(joint2_id, inertia2, pinocchio::SE3::Identity());

  // Joint 3
  Eigen::Matrix3d rotation_matrix3;
  rotation_matrix3 << 1.0, 0.0, 0.0, 0.0, -3.205103454691839e-09, 1.0, 0.0, -1.0, -3.205103454691839e-09;
  Eigen::Vector3d translation_vector3(0.0, 0.1013, 0.1013);
  pinocchio::SE3 joint_placement3(rotation_matrix3, translation_vector3);
  pinocchio::JointModelRZ joint3;
  auto joint3_id = model.addJoint(3, joint3, joint_placement3, "Link_3_jnt");

  // Append Body to Link
  Eigen::Matrix3d rotation_matrix_link3;
  rotation_matrix_link3 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d translation_vector_link3(-0.00018, 0.0477, 0.071);
  Eigen::Matrix3d inertia_matrix3;
  inertia_matrix3 << 0.00659271, -5.59e-06, -6.78e-06, -5.59e-06, 0.00585297, -0.0018243, -6.78e-06, -0.0018243,
      0.00161768;
  pinocchio::Inertia inertia3(1.0, Eigen::Vector3d(-0.00018, 0.0477, 0.071), inertia_matrix3);
  model.appendBodyToJoint(joint3_id, inertia3, pinocchio::SE3::Identity());

  // Joint 4
  Eigen::Matrix3d rotation_matrix4;
  rotation_matrix4 << 1.0, 0.0, 0.0, 0.0, -3.205103454691839e-09, -1.0, 0.0, 1.0, -3.205103454691839e-09;
  Eigen::Vector3d translation_vector4(-0.0, 0.0748, 0.1999);
  pinocchio::SE3 joint_placement4(rotation_matrix4, translation_vector4);
  pinocchio::JointModelRZ joint4;
  auto joint4_id = model.addJoint(4, joint4, joint_placement4, "Link_4_jnt");

  // Append Body to Link
  Eigen::Matrix3d rotation_matrix_link4;
  rotation_matrix_link4 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d translation_vector_link4(0.0, 0.0118, 0.0628);
  Eigen::Matrix3d inertia_matrix4;
  inertia_matrix4 << 0.0054166, 0.0, 0.0, 0.0, 0.0029408, 0.0013608, 0.0, 0.0013608, 0.0040558;
  pinocchio::Inertia inertia4(1.8, Eigen::Vector3d(0.0, 0.0118, 0.0628), inertia_matrix4);
  model.appendBodyToJoint(joint4_id, inertia4, pinocchio::SE3::Identity());

  // Joint 5
  Eigen::Matrix3d rotation_matrix5;
  rotation_matrix5 << 1.0, 0.0, 0.0, 0.0, -3.205103454691839e-09, 1.0, 0.0, -1.0, -3.205103454691839e-09;
  Eigen::Vector3d translation_vector5(0.0, 0.0748, 0.0748);
  pinocchio::SE3 joint_placement5(rotation_matrix5, translation_vector5);
  pinocchio::JointModelRZ joint5;
  auto joint5_id = model.addJoint(5, joint5, joint_placement5, "Link_5_jnt");

  // Append Body to Link
  Eigen::Matrix3d rotation_matrix_link5;
  rotation_matrix_link5 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d translation_vector_link5(0.0, 0.0, 0.038);
  Eigen::Matrix3d inertia_matrix5;
  inertia_matrix5 << 0.0018647, 0.0, 0.0, 0.0, 0.0017647, 0.0, 0.0, 0.0, 0.0006331;
  pinocchio::Inertia inertia5(1.5, Eigen::Vector3d(0.0, 0.0, 0.038), inertia_matrix5);
  model.appendBodyToJoint(joint5_id, inertia5, pinocchio::SE3::Identity());
}
}

#endif  // ROBOT_MODEL_H
