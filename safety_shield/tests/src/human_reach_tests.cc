#include <gtest/gtest.h>
#include <math.h>

#include "human_reach_fixture.h"
#include "safety_shield/human_reach.h"
#include "spdlog/spdlog.h"

namespace safety_shield {

TEST_F(HumanReachTestKF, InitializationKFTest) {
  EXPECT_DOUBLE_EQ(0, 0);
  human_reach_kf_->reset();
  delete human_reach_kf_;
  delete human_reach_cm_kf_;
}

TEST_F(HumanReachTestKF, MeasurementTest) {
  std::vector<reach_lib::Point> human_joint_pos_0 = {reach_lib::Point(0, 0, 0)};
  double t_meas_0 = 0.0;
  human_reach_kf_->measurement(human_joint_pos_0, t_meas_0);
  human_reach_cm_kf_->measurement(human_joint_pos_0, t_meas_0);
  human_reach_kf_->humanReachabilityAnalysis(0.0, 0.1);
  human_reach_cm_kf_->humanReachabilityAnalysis(0.0, 0.1);
  std::vector<reach_lib::Capsule> v_cap = human_reach_kf_->getArticulatedVelCapsules();
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.x, 0.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.y, 0.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.z, 0.0);
  std::vector<reach_lib::Capsule> v_cap_cm = human_reach_cm_kf_->getArticulatedCombinedCapsules();
  EXPECT_DOUBLE_EQ(v_cap_cm[0].p1_.x, 0.0);
  EXPECT_DOUBLE_EQ(v_cap_cm[0].p1_.y, 0.0);
  EXPECT_DOUBLE_EQ(v_cap_cm[0].p1_.z, 0.0);
  std::vector<reach_lib::Point> human_joint_pos_1 = {reach_lib::Point(1.0, 0.0, 0.0)};
  double t_meas_1 = 1.0;
  human_reach_kf_->measurement(human_joint_pos_1, t_meas_1);
  human_reach_cm_kf_->measurement(human_joint_pos_1, t_meas_1);
  human_reach_kf_->humanReachabilityAnalysis(1.0, 0.1);
  human_reach_cm_kf_->humanReachabilityAnalysis(1.0, 0.1);
  v_cap = human_reach_kf_->getArticulatedVelCapsules();
  EXPECT_TRUE(v_cap[0].p1_.x < 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.y, 0.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.z, 0.0);
  v_cap_cm = human_reach_cm_kf_->getArticulatedCombinedCapsules();
  EXPECT_TRUE(v_cap[0].p1_.x < 1.0);
  EXPECT_DOUBLE_EQ(v_cap_cm[0].p1_.y, 0.0);
  EXPECT_DOUBLE_EQ(v_cap_cm[0].p1_.z, 0.0);
}

TEST_F(HumanReachCombinedOnlyTest, InitializationCombinedOnlyTest) {
  EXPECT_DOUBLE_EQ(0, 0);
  delete human_reach_;
}

TEST_F(HumanReachCombinedOnlyTest, ErrorHandlingTests) {
  EXPECT_THROW(human_reach_->getArticulatedPosCapsules(), HumanModelNotFoundException);
  EXPECT_THROW(human_reach_->getArticulatedVelCapsules(), HumanModelNotFoundException);
  EXPECT_THROW(human_reach_->getArticulatedAccelCapsules(), HumanModelNotFoundException);
}

TEST_F(HumanReachCombinedOnlyTest, HumanReachMeasCombinedOnlyTest) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachCombinedOnlyTest, HumanReachMeasVel0ombinedOnlyTest) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(1, 2, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachCombinedOnlyTest, HumanReachAnalysisCombinedTest) {
  reach_lib::Point p(0.0, 0.0, 0.0);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(0.5, 0.0, 0.0);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 2.01;
  double t_break = 0.04;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double a_max = 10;
  double thickness = 0.1;
  double radius = thickness / 2.0;
  double v_0 = 0.5;
  reach_lib::Point dy_0(1.0, 0.0, 0.0);
  double v_max = 1;
  // v_max exactly reached
  double r = radius + human_reach_->getMeasurementErrorPos() + human_reach_->getMeasurementErrorVel() * t +
             0.5 * a_max * pow(t, 2.0);
  reach_lib::Point next_pos = q + dy_0 * v_0 * t;
  std::vector<std::vector<reach_lib::Capsule>> caps = human_reach_->getAllCapsules();
  std::vector<reach_lib::Capsule> c_cap = caps[0];
  EXPECT_DOUBLE_EQ(c_cap[0].p1_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(c_cap[0].p1_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(c_cap[0].p1_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(c_cap[0].p2_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(c_cap[0].p2_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(c_cap[0].p2_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(c_cap[0].r_, r);
}

TEST_F(HumanReachCombinedOnlyArmTest, GetHumanContactEnergies) {
  auto max_contact_energy = human_reach_->getMaxContactEnergy();
  ASSERT_EQ(max_contact_energy.size(), 1);
  ASSERT_EQ(max_contact_energy[0].size(), 3);
  EXPECT_DOUBLE_EQ(max_contact_energy[0][2], 0.75);
  EXPECT_DOUBLE_EQ(max_contact_energy[0][1], 1.0);
  EXPECT_DOUBLE_EQ(max_contact_energy[0][0], 1.0);
}

TEST_F(HumanReachCombinedOnlyArmTest, HumanReachAnalysisCombinedTest) {
  reach_lib::Point p1(0.0, 0.0, 0.0);
  reach_lib::Point p2(0.5, 0.0, 0.0);
  reach_lib::Point p3(1.0, 0.0, 0.0);
  std::vector<reach_lib::Point> human_joint_pos = {p1, p2, p3};
  double t_meas = 0.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  p1 = reach_lib::Point(0.0, 0.01, 0.0);
  p2 = reach_lib::Point(0.5, 0.01, 0.0);
  p3 = reach_lib::Point(1.0, 0.01, 0.0);
  human_joint_pos = {p1, p2, p3};
  t_meas = 0.01;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 0.02;
  double t_break = 0.03;
  // Delay = 0.01
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double a_max = 50;
  double thickness_1 = 0.1;
  double thickness_2 = 0.1;
  double thickness_3 = 0.208;
  double v_0 = 1.0;
  reach_lib::Point dy_0(0.0, 1.0, 0.0);
  double v_max = 2.0;
  double t_up = std::max(std::min((v_max - v_0) / a_max, t), 0.0);
  double t_down = std::max(std::min((v_max + v_0) / a_max, t), 0.0);
  double t_bar = std::max(std::min(v_max / a_max, t), 0.0);
  double base_r = a_max * std::sqrt(std::pow(0.5 * (t*(t_up+t_down) - 0.5 * (std::pow(t_up, 2.0) + std::pow(t_down, 2.0))), 2.0) + 
      std::pow(t*t_bar - 0.5 * std::pow(t_bar, 2.0), 2.0)) + human_reach_->getMeasurementErrorPos() + human_reach_->getMeasurementErrorVel() * t;
  double r_1 = base_r + thickness_1/2.0;
  double r_2 = base_r + thickness_2/2.0;
  double r_3 = base_r + thickness_3/2.0;
  double shift = (v_0 * t + 0.5 * a_max * (t * (t_up - t_down) - 0.5 * (std::pow(t_up, 2.0) - std::pow(t_down, 2.0))));
  reach_lib::Point next_pos_1 = p1 + dy_0 * shift;
  reach_lib::Point next_pos_2 = p2 + dy_0 * shift;
  reach_lib::Point next_pos_3 = p3 + dy_0 * shift;
  std::vector<std::vector<reach_lib::Capsule>> caps = human_reach_->getAllCapsules();
  std::vector<reach_lib::Capsule> c_cap = caps[0];
  EXPECT_EQ(c_cap.size(), 3);
  EXPECT_DOUBLE_EQ(c_cap[2].p1_.x, next_pos_1.x);
  EXPECT_DOUBLE_EQ(c_cap[2].p1_.y, next_pos_1.y);
  EXPECT_DOUBLE_EQ(c_cap[2].p1_.z, next_pos_1.z);
  EXPECT_DOUBLE_EQ(c_cap[2].p2_.x, next_pos_2.x);
  EXPECT_DOUBLE_EQ(c_cap[2].p2_.y, next_pos_2.y);
  EXPECT_DOUBLE_EQ(c_cap[2].p2_.z, next_pos_2.z);
  EXPECT_DOUBLE_EQ(c_cap[2].r_, r_1);
  EXPECT_DOUBLE_EQ(c_cap[1].p1_.x, next_pos_2.x);
  EXPECT_DOUBLE_EQ(c_cap[1].p1_.y, next_pos_2.y);
  EXPECT_DOUBLE_EQ(c_cap[1].p1_.z, next_pos_2.z);
  EXPECT_DOUBLE_EQ(c_cap[1].p2_.x, next_pos_3.x);
  EXPECT_DOUBLE_EQ(c_cap[1].p2_.y, next_pos_3.y);
  EXPECT_DOUBLE_EQ(c_cap[1].p2_.z, next_pos_3.z);
  EXPECT_DOUBLE_EQ(c_cap[1].r_, r_2);
  EXPECT_DOUBLE_EQ(c_cap[0].p1_.x, next_pos_3.x);
  EXPECT_DOUBLE_EQ(c_cap[0].p1_.y, next_pos_3.y);
  EXPECT_DOUBLE_EQ(c_cap[0].p1_.z, next_pos_3.z);
  EXPECT_DOUBLE_EQ(c_cap[0].p2_.x, next_pos_3.x);
  EXPECT_DOUBLE_EQ(c_cap[0].p2_.y, next_pos_3.y);
  EXPECT_DOUBLE_EQ(c_cap[0].p2_.z, next_pos_3.z);
  EXPECT_DOUBLE_EQ(c_cap[0].r_, r_3);
}

TEST_F(HumanReachTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0);
  EXPECT_THROW(human_reach_->getArticulatedCombinedCapsules(), HumanModelNotFoundException);
}

TEST_F(HumanReachTest, HumanReachMeasTest) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
  t_meas = 0.0;
  reach_lib::Point p2(0, 0, 0);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(p2);
  human_reach_->measurement(human_joint_pos, t_meas);
  // This should not change as the last timestep was in the past.
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
  t_meas = 1.0000000000000000000000001;
  reach_lib::Point p3(0, 0, 0);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(p3);
  human_reach_->measurement(human_joint_pos, t_meas);
  // This should now change but no velocity calculation.
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 1.0000000000000000000000001);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
  t_meas = 1.1;
  reach_lib::Point p4(0.1, 0, 0);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(p4);
  human_reach_->measurement(human_joint_pos, t_meas);
  // This should now change with velocity calculation.
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 1.1);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 0.1);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 0.0);
  EXPECT_NEAR(human_reach_->getJointVel()[0].x, 1.0, 1e-8);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachMeasVel0Test) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(1, 2, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachMeasVel1Test) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(2, 3, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachAnalysisVelTest) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  std::vector<reach_lib::Capsule> v_cap = human_reach_->getArticulatedVelCapsules();
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.z, 3.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.z, 3.0);
  // radius = thickness/2.0 + meas_err_pos + ((t_command-t_last) + t_break + t_delay) * v_max
  EXPECT_DOUBLE_EQ(v_cap[0].r_, 0.16);
}

TEST_F(HumanReachTest, HumanReachAnalysisAccTest) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(2, 3, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 2.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double a_max = 10;
  double thickness = 0.1;
  double radius = thickness / 2.0;
  double v0 = sqrt(2);
  double r = radius + human_reach_->getMeasurementErrorPos() + human_reach_->getMeasurementErrorVel() * t +
             0.5 * a_max * pow(t, 2.0);
  reach_lib::Point next_pos = q + (q - p) * t;
  std::vector<reach_lib::Capsule> a_cap = human_reach_->getArticulatedAccelCapsules();
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].r_, r);
}

TEST_F(HumanReachTestError, HumanReachAnalysisVelTestError) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  double thickness = 0.1;
  double radius = thickness / 2.0;
  double v_max = 1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  // radius = thickness + meas_err_pos + ((t_command-t_last) + t_break + t_delay) * v_max
  double r = radius + human_reach_->getMeasurementErrorPos() + t * v_max;
  std::vector<reach_lib::Capsule> v_cap = human_reach_->getArticulatedVelCapsules();
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.z, 3.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.z, 3.0);
  EXPECT_DOUBLE_EQ(v_cap[0].r_, r);
}

TEST_F(HumanReachTestError, HumanReachAnalysisAccTestError) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(2, 3, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 2.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double a_max = 10;
  double thickness = 0.1;
  double radius = thickness / 2.0;
  double v0 = sqrt(2);
  double r = radius + human_reach_->getMeasurementErrorPos() + human_reach_->getMeasurementErrorVel() * t +
             0.5 * a_max * pow(t, 2.0);
  reach_lib::Point next_pos = q + (q - p) * t;
  std::vector<reach_lib::Capsule> a_cap = human_reach_->getArticulatedAccelCapsules();
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].r_, r);
}

// Currently failing due to https://github.com/Sven-Schepp/SaRA/issues/14
TEST_F(HumanReachTestPos, HumanReachAnalysisPos) {
  reach_lib::Point p1(0, 0, 0);
  reach_lib::Point p2(0.2, 0.2, 0.2);
  reach_lib::Point p3(0.4, 0.4, 0.4);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p1);
  human_joint_pos.push_back(p2);
  human_joint_pos.push_back(p3);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double length = 0.725;
  double thickness = 0.0;
  double v_max = 2;
  double r = length + thickness + human_reach_->getMeasurementErrorPos() + v_max * t;
  reach_lib::Point next_pos = p1;
  std::vector<reach_lib::Capsule> p_cap = human_reach_->getArticulatedPosCapsules();
  EXPECT_DOUBLE_EQ(p_cap[0].p1_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(p_cap[0].p1_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(p_cap[0].p1_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(p_cap[0].p2_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(p_cap[0].p2_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(p_cap[0].p2_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(p_cap[0].r_, r);
}

/// time interval method is equal to standard reach if reachability set duration is equal or larger to s_diff (with single joint)
TEST_F(HumanReachTimeIntervalTestSingleJoint, Equality) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_single_joint_standard_->measurement(human_joint_pos, t_meas);
  human_reach_single_joint_time_interval_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  double t_start_interval = 0;
  double t_end_interval = t_break;
  human_reach_single_joint_standard_->humanReachabilityAnalysis(t_command, t_break);
  std::vector<std::vector<reach_lib::Capsule>> standard_capsules = human_reach_single_joint_standard_->getAllCapsules();
  std::vector<double> time_intervals = {t_start_interval, t_end_interval};
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> time_interval_capsules = 
    human_reach_single_joint_time_interval_->humanReachabilityAnalysisTimeIntervals(
      t_command, time_intervals
    );
  EXPECT_TRUE(time_interval_capsules.size() == 1);
  EXPECT_TRUE(time_interval_capsules[0].size() == 3);
  for(int i = 0; i < 3; i++) {
    EXPECT_TRUE(time_interval_capsules[0][i].size() == standard_capsules[i].size());
    for(int j = 0; j < time_interval_capsules[0][i].size(); j++) {
      EXPECT_DOUBLE_EQ(time_interval_capsules[0][i][j].p1_.x, standard_capsules[i][j].p1_.x);
      EXPECT_DOUBLE_EQ(time_interval_capsules[0][i][j].p1_.y, standard_capsules[i][j].p1_.y);
      EXPECT_DOUBLE_EQ(time_interval_capsules[0][i][j].p1_.z, standard_capsules[i][j].p1_.z);
      EXPECT_DOUBLE_EQ(time_interval_capsules[0][i][j].p2_.x, standard_capsules[i][j].p2_.x);
      EXPECT_DOUBLE_EQ(time_interval_capsules[0][i][j].p2_.y, standard_capsules[i][j].p2_.y);
      EXPECT_DOUBLE_EQ(time_interval_capsules[0][i][j].p2_.z, standard_capsules[i][j].p2_.z);
      EXPECT_DOUBLE_EQ(time_interval_capsules[0][i][j].r_, standard_capsules[i][j].r_);
    }
  }
}

/// time interval method is equal to standard reach if reachability set duration is equal or larger than s_diff (with test arm)
TEST_F(HumanReachTimeIntervalTestArm, EqualityArm) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  for (int i = 0; i < human_reach_test_arm_standard_->getJointPos().size(); i++) {
    human_joint_pos.push_back(p);
  }
  double t_meas = 1.0;
  human_reach_test_arm_standard_->measurement(human_joint_pos, t_meas);
  human_reach_test_arm_time_interval_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  double t_start_interval = 0.0;
  double t_end_interval = t_break;
  human_reach_test_arm_standard_->humanReachabilityAnalysis(t_command, t_break);
  std::vector<std::vector<reach_lib::Capsule>> standard_capsules = human_reach_test_arm_standard_->getAllCapsules();
  std::vector<double> time_intervals = {t_start_interval, t_end_interval};
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> time_interval_capsules =
    human_reach_test_arm_time_interval_->humanReachabilityAnalysisTimeIntervals(
      t_command, time_intervals
    );

  EXPECT_TRUE(standard_capsules.size() == time_interval_capsules[0].size());
  std::vector<std::vector<reach_lib::Capsule>> time_interval_capsules_0 = time_interval_capsules[0];
  EXPECT_TRUE(standard_capsules[0].size() == time_interval_capsules_0[0].size());
  EXPECT_TRUE(standard_capsules[1].size() == time_interval_capsules_0[1].size());
  EXPECT_TRUE(standard_capsules[2].size() == time_interval_capsules_0[2].size());

  EXPECT_NEAR(time_interval_capsules_0[0][0].p1_.x, standard_capsules[0][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p1_.y, standard_capsules[0][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p1_.z, standard_capsules[0][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p2_.x, standard_capsules[0][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p2_.y, standard_capsules[0][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p2_.z, standard_capsules[0][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].r_, standard_capsules[0][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[1][0].p1_.x, standard_capsules[1][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p1_.y, standard_capsules[1][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p1_.z, standard_capsules[1][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p2_.x, standard_capsules[1][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p2_.y, standard_capsules[1][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p2_.z, standard_capsules[1][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].r_, standard_capsules[1][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[1][1].p1_.x, standard_capsules[1][1].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p1_.y, standard_capsules[1][1].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p1_.z, standard_capsules[1][1].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p2_.x, standard_capsules[1][1].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p2_.y, standard_capsules[1][1].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p2_.z, standard_capsules[1][1].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].r_, standard_capsules[1][1].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[1][2].p1_.x, standard_capsules[1][2].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p1_.y, standard_capsules[1][2].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p1_.z, standard_capsules[1][2].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p2_.x, standard_capsules[1][2].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p2_.y, standard_capsules[1][2].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p2_.z, standard_capsules[1][2].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].r_, standard_capsules[1][2].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[2][0].p1_.x, standard_capsules[2][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p1_.y, standard_capsules[2][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p1_.z, standard_capsules[2][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p2_.x, standard_capsules[2][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p2_.y, standard_capsules[2][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p2_.z, standard_capsules[2][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].r_, standard_capsules[2][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[2][1].p1_.x, standard_capsules[2][1].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p1_.y, standard_capsules[2][1].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p1_.z, standard_capsules[2][1].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p2_.x, standard_capsules[2][1].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p2_.y, standard_capsules[2][1].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p2_.z, standard_capsules[2][1].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].r_, standard_capsules[2][1].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[2][2].p1_.x, standard_capsules[2][2].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p1_.y, standard_capsules[2][2].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p1_.z, standard_capsules[2][2].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p2_.x, standard_capsules[2][2].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p2_.y, standard_capsules[2][2].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p2_.z, standard_capsules[2][2].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].r_, standard_capsules[2][2].r_, 1e-6);
}


/// time interval method with two intervals is equal to standard reach of the two separate intervals (with single joint)
TEST_F(HumanReachTimeIntervalTestSingleJoint, IntervalTest) {
 reach_lib::Point p(1, 2, 3);
 std::vector<reach_lib::Point> human_joint_pos;
 human_joint_pos.push_back(p);
 double t_meas = 0.0;
 human_reach_single_joint_standard_->measurement(human_joint_pos, t_meas);
 human_reach_single_joint_time_interval_->measurement(human_joint_pos, t_meas);
 double t_command = 0.0;
 double t_break_1 = 0.05;
 double t_break_2 = 0.1;
 double current_motion = 0;
 human_reach_single_joint_standard_->humanReachabilityAnalysis(t_command, t_break_1);
 std::vector<std::vector<reach_lib::Capsule>> first_interval_capsules = human_reach_single_joint_standard_->getAllCapsules();
 human_reach_single_joint_standard_->humanReachabilityAnalysis(t_command, t_break_2);
 std::vector<std::vector<reach_lib::Capsule>> second_interval_capsules = human_reach_single_joint_standard_->getAllCapsules();
 std::vector<double> time_intervals = {current_motion, t_break_1, t_break_2};
 std::vector<std::vector<std::vector<reach_lib::Capsule>>> time_interval_capsules =
  human_reach_single_joint_time_interval_->humanReachabilityAnalysisTimeIntervals(
    t_command, time_intervals
  );
 EXPECT_TRUE(time_interval_capsules.size() == 2);
 EXPECT_TRUE(time_interval_capsules[0].size() == 3);

 /// equality of first interval
 for(int i = 0; i < 3; i++) {
   EXPECT_TRUE(time_interval_capsules[0][i].size() == first_interval_capsules[i].size());
   for(int j = 0; j < time_interval_capsules[0][i].size(); j++) {
     EXPECT_NEAR(time_interval_capsules[0][i][j].p1_.x, first_interval_capsules[i][j].p1_.x, 1e-6);
     EXPECT_NEAR(time_interval_capsules[0][i][j].p1_.y, first_interval_capsules[i][j].p1_.y, 1e-6);
     EXPECT_NEAR(time_interval_capsules[0][i][j].p1_.z, first_interval_capsules[i][j].p1_.z, 1e-6);
     EXPECT_NEAR(time_interval_capsules[0][i][j].p2_.x, first_interval_capsules[i][j].p2_.x, 1e-6);
     EXPECT_NEAR(time_interval_capsules[0][i][j].p2_.y, first_interval_capsules[i][j].p2_.y, 1e-6);
     EXPECT_NEAR(time_interval_capsules[0][i][j].p2_.z, first_interval_capsules[i][j].p2_.z, 1e-6);
     EXPECT_NEAR(time_interval_capsules[0][i][j].r_, first_interval_capsules[i][j].r_, 1e-6);
   }
 }

 /// equality of second interval
 for(int i = 0; i < 3; i++) {
   EXPECT_TRUE(time_interval_capsules[1][i].size() == second_interval_capsules[i].size());
   for(int j = 0; j < time_interval_capsules[1][i].size(); j++) {
     EXPECT_NEAR(time_interval_capsules[1][i][j].p1_.x, second_interval_capsules[i][j].p1_.x, 1e-6);
     EXPECT_NEAR(time_interval_capsules[1][i][j].p1_.y, second_interval_capsules[i][j].p1_.y, 1e-6);
     EXPECT_NEAR(time_interval_capsules[1][i][j].p1_.z, second_interval_capsules[i][j].p1_.z, 1e-6);
     EXPECT_NEAR(time_interval_capsules[1][i][j].p2_.x, second_interval_capsules[i][j].p2_.x, 1e-6);
     EXPECT_NEAR(time_interval_capsules[1][i][j].p2_.y, second_interval_capsules[i][j].p2_.y, 1e-6);
     EXPECT_NEAR(time_interval_capsules[1][i][j].p2_.z, second_interval_capsules[i][j].p2_.z, 1e-6);
     EXPECT_NEAR(time_interval_capsules[1][i][j].r_, second_interval_capsules[i][j].r_, 1e-6);
   }
 }
}

/// time interval method with two intervals is equal to standard reach of the two separate intervals (with test arm)
TEST_F(HumanReachTimeIntervalTestArm, IntervalTest1) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  for (int i = 0; i < human_reach_test_arm_standard_->getJointPos().size(); i++) {
    human_joint_pos.push_back(p);
  }
  double t_meas = 0.0;
  human_reach_test_arm_standard_->measurement(human_joint_pos, t_meas);
  human_reach_test_arm_time_interval_->measurement(human_joint_pos, t_meas);
  double t_command = 0.0;
  double t_break_1 = 0.05;
  double t_break_2 = 0.1;
  double current_motion = 0;
  std::vector<double> time_intervals = {current_motion, t_break_1, t_break_2};
  std::vector<double> first_interval = {current_motion, t_break_1};
  // TODO: second interval?
  std::vector<double> second_interval = {t_break_1, t_break_2}; //{current_motion, t_break_2};
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> first_interval_capsules =
    human_reach_test_arm_time_interval_->humanReachabilityAnalysisTimeIntervals(
      t_command, first_interval
    );
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> second_interval_capsules =
    human_reach_test_arm_time_interval_->humanReachabilityAnalysisTimeIntervals(
      t_command, second_interval
    );
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> time_interval_capsules =
    human_reach_test_arm_time_interval_->humanReachabilityAnalysisTimeIntervals(
      t_command, time_intervals
    );
  EXPECT_TRUE(time_interval_capsules.size() == 2);

  /// test equality with first interval
  for(int i = 0; i < 3; i++) {
    EXPECT_TRUE(time_interval_capsules[0][i].size() == first_interval_capsules[0][i].size());
    for(int j = 0; j < time_interval_capsules[0][i].size(); j++) {
      EXPECT_NEAR(time_interval_capsules[0][i][j].p1_.x, first_interval_capsules[0][i][j].p1_.x, 1e-6);
      EXPECT_NEAR(time_interval_capsules[0][i][j].p1_.y, first_interval_capsules[0][i][j].p1_.y, 1e-6);
      EXPECT_NEAR(time_interval_capsules[0][i][j].p1_.z, first_interval_capsules[0][i][j].p1_.z, 1e-6);
      EXPECT_NEAR(time_interval_capsules[0][i][j].p2_.x, first_interval_capsules[0][i][j].p2_.x, 1e-6);
      EXPECT_NEAR(time_interval_capsules[0][i][j].p2_.y, first_interval_capsules[0][i][j].p2_.y, 1e-6);
      EXPECT_NEAR(time_interval_capsules[0][i][j].p2_.z, first_interval_capsules[0][i][j].p2_.z, 1e-6);
      EXPECT_NEAR(time_interval_capsules[0][i][j].r_, first_interval_capsules[0][i][j].r_, 1e-6);
    }
  }

  /// test equality with second interval
  for(int i = 0; i < 3; i++) {
    EXPECT_TRUE(time_interval_capsules[1][i].size() == second_interval_capsules[0][i].size());
    for(int j = 0; j < time_interval_capsules[1][i].size(); j++) {
      EXPECT_NEAR(time_interval_capsules[1][i][j].p1_.x, second_interval_capsules[0][i][j].p1_.x, 1e-6);
      EXPECT_NEAR(time_interval_capsules[1][i][j].p1_.y, second_interval_capsules[0][i][j].p1_.y, 1e-6);
      EXPECT_NEAR(time_interval_capsules[1][i][j].p1_.z, second_interval_capsules[0][i][j].p1_.z, 1e-6);
      EXPECT_NEAR(time_interval_capsules[1][i][j].p2_.x, second_interval_capsules[0][i][j].p2_.x, 1e-6);
      EXPECT_NEAR(time_interval_capsules[1][i][j].p2_.y, second_interval_capsules[0][i][j].p2_.y, 1e-6);
      EXPECT_NEAR(time_interval_capsules[1][i][j].p2_.z, second_interval_capsules[0][i][j].p2_.z, 1e-6);
      EXPECT_NEAR(time_interval_capsules[1][i][j].r_, second_interval_capsules[0][i][j].r_, 1e-6);
    }
  }
}

/// time interval method with two intervals is equal to standard reach of the two separate intervals (with test arm)
TEST_F(HumanReachTimeIntervalTestArm, IntervalTest2) {
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  for (int i = 0; i < human_reach_test_arm_standard_->getJointPos().size(); i++) {
    human_joint_pos.push_back(p);
  }
  double t_meas = 0.0;
  human_reach_test_arm_standard_->measurement(human_joint_pos, t_meas);
  human_reach_test_arm_time_interval_->measurement(human_joint_pos, t_meas);
  double t_command = 0.0;
  double t_break_1 = 0.05;
  double t_break_2 = 0.1;
  double current_motion = 0;
  human_reach_test_arm_standard_->humanReachabilityAnalysis(t_command, t_break_1);
  std::vector<std::vector<reach_lib::Capsule>> first_interval_capsules = human_reach_test_arm_standard_->getAllCapsules();
  human_reach_test_arm_standard_->humanReachabilityAnalysis(t_command, t_break_2);
  std::vector<std::vector<reach_lib::Capsule>> second_interval_capsules = human_reach_test_arm_standard_->getAllCapsules();
  std::vector<double> time_intervals = {current_motion, t_break_1, t_break_2};
  std::vector<std::vector<std::vector<reach_lib::Capsule>>> time_interval_capsules =
    human_reach_test_arm_time_interval_->humanReachabilityAnalysisTimeIntervals(
      t_command, time_intervals
    );
  EXPECT_TRUE(time_interval_capsules.size() == 2);

  /// test equality with first interval
  std::vector<std::vector<reach_lib::Capsule>>  time_interval_capsules_0 = time_interval_capsules[0];
  EXPECT_NEAR(time_interval_capsules_0[0][0].p1_.x, first_interval_capsules[0][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p1_.y, first_interval_capsules[0][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p1_.z, first_interval_capsules[0][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p2_.x, first_interval_capsules[0][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p2_.y, first_interval_capsules[0][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].p2_.z, first_interval_capsules[0][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[0][0].r_, first_interval_capsules[0][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[1][0].p1_.x, first_interval_capsules[1][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p1_.y, first_interval_capsules[1][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p1_.z, first_interval_capsules[1][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p2_.x, first_interval_capsules[1][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p2_.y, first_interval_capsules[1][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].p2_.z, first_interval_capsules[1][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][0].r_, first_interval_capsules[1][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[1][1].p1_.x, first_interval_capsules[1][1].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p1_.y, first_interval_capsules[1][1].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p1_.z, first_interval_capsules[1][1].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p2_.x, first_interval_capsules[1][1].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p2_.y, first_interval_capsules[1][1].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].p2_.z, first_interval_capsules[1][1].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][1].r_, first_interval_capsules[1][1].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[1][2].p1_.x, first_interval_capsules[1][2].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p1_.y, first_interval_capsules[1][2].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p1_.z, first_interval_capsules[1][2].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p2_.x, first_interval_capsules[1][2].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p2_.y, first_interval_capsules[1][2].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].p2_.z, first_interval_capsules[1][2].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[1][2].r_, first_interval_capsules[1][2].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[2][0].p1_.x, first_interval_capsules[2][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p1_.y, first_interval_capsules[2][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p1_.z, first_interval_capsules[2][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p2_.x, first_interval_capsules[2][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p2_.y, first_interval_capsules[2][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].p2_.z, first_interval_capsules[2][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][0].r_, first_interval_capsules[2][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[2][1].p1_.x, first_interval_capsules[2][1].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p1_.y, first_interval_capsules[2][1].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p1_.z, first_interval_capsules[2][1].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p2_.x, first_interval_capsules[2][1].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p2_.y, first_interval_capsules[2][1].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].p2_.z, first_interval_capsules[2][1].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][1].r_, first_interval_capsules[2][1].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_0[2][2].p1_.x, first_interval_capsules[2][2].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p1_.y, first_interval_capsules[2][2].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p1_.z, first_interval_capsules[2][2].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p2_.x, first_interval_capsules[2][2].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p2_.y, first_interval_capsules[2][2].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].p2_.z, first_interval_capsules[2][2].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_0[2][2].r_, first_interval_capsules[2][2].r_, 1e-6);

  /// test equality with second interval
  std::vector<std::vector<reach_lib::Capsule>>  time_interval_capsules_1 = time_interval_capsules[1];
  EXPECT_NEAR(time_interval_capsules_1[0][0].p1_.x, second_interval_capsules[0][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[0][0].p1_.y, second_interval_capsules[0][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[0][0].p1_.z, second_interval_capsules[0][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[0][0].p2_.x, second_interval_capsules[0][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[0][0].p2_.y, second_interval_capsules[0][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[0][0].p2_.z, second_interval_capsules[0][0].p2_.z, 1e-6);
  // Currently failing due to https://github.com/Sven-Schepp/SaRA/issues/14
  EXPECT_NEAR(time_interval_capsules_1[0][0].r_, second_interval_capsules[0][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_1[1][0].p1_.x, second_interval_capsules[1][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][0].p1_.y, second_interval_capsules[1][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][0].p1_.z, second_interval_capsules[1][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][0].p2_.x, second_interval_capsules[1][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][0].p2_.y, second_interval_capsules[1][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][0].p2_.z, second_interval_capsules[1][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][0].r_, second_interval_capsules[1][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_1[1][1].p1_.x, second_interval_capsules[1][1].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][1].p1_.y, second_interval_capsules[1][1].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][1].p1_.z, second_interval_capsules[1][1].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][1].p2_.x, second_interval_capsules[1][1].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][1].p2_.y, second_interval_capsules[1][1].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][1].p2_.z, second_interval_capsules[1][1].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][1].r_, second_interval_capsules[1][1].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_1[1][2].p1_.x, second_interval_capsules[1][2].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][2].p1_.y, second_interval_capsules[1][2].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][2].p1_.z, second_interval_capsules[1][2].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][2].p2_.x, second_interval_capsules[1][2].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][2].p2_.y, second_interval_capsules[1][2].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][2].p2_.z, second_interval_capsules[1][2].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[1][2].r_, second_interval_capsules[1][2].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_1[2][0].p1_.x, second_interval_capsules[2][0].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][0].p1_.y, second_interval_capsules[2][0].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][0].p1_.z, second_interval_capsules[2][0].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][0].p2_.x, second_interval_capsules[2][0].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][0].p2_.y, second_interval_capsules[2][0].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][0].p2_.z, second_interval_capsules[2][0].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][0].r_, second_interval_capsules[2][0].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_1[2][1].p1_.x, second_interval_capsules[2][1].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][1].p1_.y, second_interval_capsules[2][1].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][1].p1_.z, second_interval_capsules[2][1].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][1].p2_.x, second_interval_capsules[2][1].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][1].p2_.y, second_interval_capsules[2][1].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][1].p2_.z, second_interval_capsules[2][1].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][1].r_, second_interval_capsules[2][1].r_, 1e-6);

  EXPECT_NEAR(time_interval_capsules_1[2][2].p1_.x, second_interval_capsules[2][2].p1_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][2].p1_.y, second_interval_capsules[2][2].p1_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][2].p1_.z, second_interval_capsules[2][2].p1_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][2].p2_.x, second_interval_capsules[2][2].p2_.x, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][2].p2_.y, second_interval_capsules[2][2].p2_.y, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][2].p2_.z, second_interval_capsules[2][2].p2_.z, 1e-6);
  EXPECT_NEAR(time_interval_capsules_1[2][2].r_, second_interval_capsules[2][2].r_, 1e-6);

}

}  // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}