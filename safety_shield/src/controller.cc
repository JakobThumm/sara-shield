#include "safety_shield/controller.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <iterator>
#include <pinocchio/algorithm/rnea.hpp>
#include <sstream>

#include "pinocchio/algorithm/crba.hpp"

std::function<std::string(const std::vector<double>&)> vecToStr = [](const std::vector<double>& v) {
  std::stringstream ss;
  ss.precision(4);
  std::copy(v.begin(), v.end(), std::ostream_iterator<double>(ss, v.size() > 1 ? ", " : ""));
  return ss.str();
};

static std::string toString(const Eigen::MatrixXd& mat) {
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

Controller::Controller(const std::string& urdf_file, const std::vector<double>& armature,
                       const std::vector<double>& jbv, const std::vector<double>& jbc, const std::string& srdf_file) {
  this->buildModel(urdf_file, model);

  // Initialize friction parameters
  this->jbv = Eigen::VectorXd::Map(jbv.data(), jbv.size());
  this->jbc = Eigen::VectorXd::Map(jbc.data(), jbc.size());

  if (!armature.empty()) {
    this->armature = Eigen::VectorXd::Map(&armature[0], armature.size());
    this->with_armature = true;
  } else {
    this->with_armature = false;
  }

  // Initialize the joint state vectors
  q_m = Eigen::VectorXd::Zero(model.nq);
  dq_m = Eigen::VectorXd::Zero(model.nv);
  ddq_m = Eigen::VectorXd::Zero(model.nv);
  q_d = Eigen::VectorXd::Zero(model.nv);
  dq_d = Eigen::VectorXd::Zero(model.nv);
  ddq_d = Eigen::VectorXd::Zero(model.nv);
}

Eigen::MatrixXd Controller::calculateMassMatrix() {
  pinocchio::Data data(model);
  pinocchio::crba(model, data, this->q_m);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd massMatrix = data.M;

  for (int i = 0; i < massMatrix.rows(); ++i) {
    massMatrix(i, i) = massMatrix(i, i) + armature[i];
  }
  return massMatrix;
}

std::vector<std::vector<double>> Controller::getMassMatrix() {
  Eigen::MatrixXd massMatrix = calculateMassMatrix();

  // Convert Eigen::MatrixXd to std::vector<std::vector<double>>
  std::vector<std::vector<double>> massMatrixVec(massMatrix.rows(), std::vector<double>(massMatrix.cols()));
  for (int i = 0; i < massMatrix.rows(); ++i) {
    for (int j = 0; j < massMatrix.cols(); ++j) {
      massMatrixVec[i][j] = massMatrix(i, j);
    }
  }
  return massMatrixVec;
}

std::vector<double> Controller::getGravityVector() {
  pinocchio::Data data(model);
  // Compute the gravity term using the current state
  pinocchio::computeGeneralizedGravity(model, data, q_m);

  // Convert Eigen::VectorXd to std::vector<double>
  const Eigen::VectorXd& g = data.g;
  std::vector<double> gravityVector(g.data(), g.data() + g.size());
  return gravityVector;
}

std::vector<std::vector<double>> Controller::getCoriolisMatrix() {
  pinocchio::Data data(model);
  // Compute the Coriolis matrix using the current state
  pinocchio::computeCoriolisMatrix(model, data, q_m, dq_m);

  // Convert Eigen::MatrixXd to std::vector<std::vector<double>>
  const Eigen::MatrixXd& C = data.C;
  std::vector<std::vector<double>> coriolisMatrixVec(C.rows(), std::vector<double>(C.cols()));
  for (int i = 0; i < C.rows(); ++i) {
    for (int j = 0; j < C.cols(); ++j) {
      coriolisMatrixVec[i][j] = C(i, j);
    }
  }
  return coriolisMatrixVec;
}

std::vector<double> Controller::getNonLinearEffects() {
  // Compute the non-linear effects (Coriolis, centrifugal, and gravitational forces)
  // Assuming 'q' is your current joint configuration and 'v' is the current joint velocities
  pinocchio::Data data(model);
  pinocchio::nonLinearEffects(model, data, this->q_m, this->dq_m);
  Eigen::VectorXd nonLinearEffects = data.nle;
  std::vector<double> nonLinearEffectsVec(nonLinearEffects.data(),
                                          nonLinearEffects.data() + nonLinearEffects.rows() * nonLinearEffects.cols());
  return nonLinearEffectsVec;
}

Eigen::VectorXd Controller::calculateFrictionTorque(const Eigen::VectorXd& dq) {
  Eigen::VectorXd tau_f = Eigen::VectorXd::Zero(dq.size());
  for (int i = 0; i < dq.size(); ++i) {
    tau_f(i) = jbv(i) * dq(i) + jbc(i) * sin(atan(100 * dq(i)));
  }
  return tau_f;
}

ImpedanceController::ImpedanceController(const std::string& urdf_file, const std::vector<double>& K, const std::vector<double>& D,
                                         const std::vector<double>& armature, const std::vector<double>& jbv,
                                         const std::vector<double>& jbc, const std::string& srdf_file)
    : Controller(urdf_file, armature, jbv, jbc, srdf_file) {
  
  this->K = Eigen::MatrixXd::Zero(model.nv, model.nv);
  this->D = Eigen::MatrixXd::Zero(model.nv, model.nv);

  for (int i = 0; i < model.nv; ++i) {
    this->K(i, i) = K[i];
    this->D(i, i) = D[i];
  }
  
}

Eigen::VectorXd ImpedanceController::calculateFeedForwardTorque() {
  pinocchio::Data data(model);
  pinocchio::nonLinearEffects(model, data, this->q_m, this->dq_m);
  return data.nle;
}

Eigen::VectorXd ImpedanceController::calculateClosedLoopTorque() {
  pinocchio::Data data(model);
  pinocchio::crba(model, data, this->q_m);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  return K * (q_d - q_m) + D * (dq_d - dq_m) + data.M * ddq_d;
}

void Controller::setEigenVector(Eigen::VectorXd& eig_vec, const std::vector<double>& vec) {
  for (int i = 0; i < this->model.nv; i++) eig_vec(i) = vec.at(i);
}

std::vector<double> ImpedanceController::calculateOutputTorque(
    const std::vector<double>& q_m, const std::vector<double>& dq_m, const std::vector<double>& ddq_m,
    const std::vector<double>& q_d, const std::vector<double>& dq_d, const std::vector<double>& ddq_d) {
  setEigenVector(this->q_m, q_m);
  setEigenVector(this->dq_m, dq_m);
  setEigenVector(this->ddq_m, ddq_m);
  setEigenVector(this->q_d, q_d);
  setEigenVector(this->dq_d, dq_d);
  setEigenVector(this->ddq_d, ddq_d);

  Eigen::VectorXd result = calculateFeedForwardTorque() + calculateClosedLoopTorque();
  return std::vector<double>(result.data(), result.data() + result.rows() * result.cols());
}

PassivityController::PassivityController(const std::string& urdf_file, const std::vector<double>& K, const std::vector<double>& D,
                                         const std::vector<double>& armature, const std::vector<double>& jbv,
                                         const std::vector<double>& jbc, const std::string& srdf_file)
    : Controller(urdf_file, armature, jbv, jbc, srdf_file){

  this->K = Eigen::MatrixXd::Zero(model.nv, model.nv);
  this->D = Eigen::MatrixXd::Zero(model.nv, model.nv);

  for (int i = 0; i < model.nv; ++i) {
    this->K(i, i) = K[i];
    this->D(i, i) = D[i];
  }

}

std::vector<double> PassivityController::calculateOutputTorque(
    const std::vector<double>& q_m, const std::vector<double>& dq_m, const std::vector<double>& ddq_m,
    const std::vector<double>& q_d, const std::vector<double>& dq_d, const std::vector<double>& ddq_d) {
  pinocchio::Data data(model);

  setEigenVector(this->q_m, q_m);
  setEigenVector(this->dq_m, dq_m);
  setEigenVector(this->ddq_m, ddq_m);
  setEigenVector(this->q_d, q_d);
  setEigenVector(this->dq_d, dq_d);
  setEigenVector(this->ddq_d, ddq_d);

  // Calculate errors
  Eigen::VectorXd e = this->q_d - this->q_m;
  Eigen::VectorXd de = this->dq_d - this->dq_m;

  // Auxiliary velocities and accelerations
  Eigen::VectorXd dq_a = this->dq_d + K * e;
  Eigen::VectorXd ddq_a = this->ddq_d + D * de;

  // Control input nu
  Eigen::VectorXd nu = K * (de + D * e);

  Eigen::MatrixXd massMatrix = calculateMassMatrix();

  Eigen::VectorXd tau_mass_matrix = massMatrix * ddq_a;
  Eigen::VectorXd tau_coriolis = pinocchio::computeCoriolisMatrix(model, data, this->q_m, this->dq_m) * dq_a;

  // Compute inverse dynamics (Robot dynamics considering desired accelerations)
  Eigen::VectorXd tau = tau_mass_matrix + tau_coriolis + pinocchio::computeGeneralizedGravity(model, data, this->q_m);

  // Add control input nu and friction torques
  tau += nu + calculateFrictionTorque(this->dq_d);

  return std::vector<double>(tau.data(), tau.data() + tau.rows() * tau.cols());
}
