#ifndef CONTROLLER_H
#define CONTROLLER_H

// #define BUILD_FROM_URDF

#include <Eigen/Dense>
#include <string>
#include <vector>

#ifdef BUILD_FROM_URDF
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#else
#include "safety_shield/robot_params.h"
#endif

class Controller {
 protected:
  pinocchio::Model model;
  Eigen::VectorXd q_m;    // Measured joint positions
  Eigen::VectorXd dq_m;   // Measured joint velocities
  Eigen::VectorXd ddq_m;  // Measured joint accelerations
  Eigen::VectorXd q_d;    // Desired joint positions
  Eigen::VectorXd dq_d;   // Desired joint velocities
  Eigen::VectorXd ddq_d;  // Desired joint accelerations
  Eigen::VectorXd jbv;    // Joint-based viscous friction coefficients
  Eigen::VectorXd jbc;    // Joint-based Coulomb friction coefficients

  Eigen::VectorXd armature;  // armature values

  bool with_armature;

  virtual void buildModel(const std::string& urdf_file, pinocchio::Model& model, const std::string& srdf_file = "") {
#ifdef BUILD_FROM_URDF
    pinocchio::urdf::buildModel(urdf_file, model, false);

    if (!srdf_file.empty()) {
      pinocchio::srdf::loadRotorParameters(model, srdf_file, false);  // Loading rotor parameters
    }

#else
    robotParams::schunk(model);
#endif
  }

 public:
  explicit Controller(const std::string& urdf_file, const std::vector<double>& armature = std::vector<double>(),
                      const std::vector<double>& jbv = std::vector<double>(),
                      const std::vector<double>& jbc = std::vector<double>(), const std::string& srdf_file = "");
  virtual ~Controller() {};
  virtual void setEigenVector(Eigen::VectorXd& eig_vec, const std::vector<double>& vec);
  virtual Eigen::MatrixXd calculateMassMatrix();
  virtual std::vector<std::vector<double>> getMassMatrix();
  virtual std::vector<double> getGravityVector();
  virtual std::vector<std::vector<double>> getCoriolisMatrix();
  virtual std::vector<double> getNonLinearEffects();
  virtual std::vector<double> calculateOutputTorque(const std::vector<double>& q_m, const std::vector<double>& dq_m,
                                                    const std::vector<double>& ddq_m, const std::vector<double>& q_d,
                                                    const std::vector<double>& dq_d,
                                                    const std::vector<double>& ddq_d) = 0;
  virtual Eigen::VectorXd calculateFrictionTorque(const Eigen::VectorXd& dq);
};

class ImpedanceController : public Controller {
 private:
  Eigen::MatrixXd K;  // Stiffness matrix
  Eigen::MatrixXd D;  // Damping matrix
  Eigen::VectorXd calculateFeedForwardTorque();
  Eigen::VectorXd calculateClosedLoopTorque();

 public:
  explicit ImpedanceController(const std::string& urdf_file, const std::vector<double>& K, const std::vector<double>& D,
                               const std::vector<double>& armature = std::vector<double>(),
                               const std::vector<double>& jbv = std::vector<double>(),
                               const std::vector<double>& jbc = std::vector<double>(),
                               const std::string& srdf_file = "");
  ~ImpedanceController() {};
  std::vector<double> calculateOutputTorque(const std::vector<double>& q_m, const std::vector<double>& dq_m,
                                            const std::vector<double>& ddq_m, const std::vector<double>& q_d,
                                            const std::vector<double>& dq_d, const std::vector<double>& ddq_d) override;
  std::vector<double> getFrictionTorque() {
    Eigen::VectorXd tau_f = calculateFrictionTorque(dq_d);
    return std::vector<double>(tau_f.data(), tau_f.data() + tau_f.size());
  }
  void changeKD(const std::vector<double>& K, const std::vector<double>& D) {
    for (int i = 0; i < model.nv; ++i) {
      this->K(i, i) = K[i];
      this->D(i, i) = D[i];
    }
  }
};

class PassivityController : public Controller {
 private:
  Eigen::MatrixXd K;  // Proportional gain for the control input nu
  Eigen::MatrixXd D;  // Derivative gain for the control input nu

 public:
  PassivityController(const std::string& urdf_file, const std::vector<double>& K, const std::vector<double>& D,
                      const std::vector<double>& armature = std::vector<double>(),
                      const std::vector<double>& jbv = std::vector<double>(),
                      const std::vector<double>& jbc = std::vector<double>(), const std::string& srdf_file = "");
  std::vector<double> calculateOutputTorque(const std::vector<double>& q_m, const std::vector<double>& dq_m,
                                            const std::vector<double>& ddq_m, const std::vector<double>& q_d,
                                            const std::vector<double>& dq_d, const std::vector<double>& ddq_d);
  void changeKD(const std::vector<double>& K, const std::vector<double>& D) {
    for (int i = 0; i < model.nv; ++i) {
      this->K(i, i) = K[i];
      this->D(i, i) = D[i];
    }
  }
};

#endif  // CONTROLLER_H
