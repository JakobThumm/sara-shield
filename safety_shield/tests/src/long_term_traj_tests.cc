#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <vector>

#include "long_term_traj_fixture.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"

namespace safety_shield {

TEST_F(LongTermTrajTestIdx, GetLengthTest) {
  EXPECT_EQ(long_term_trajectory_.getLength(), 4);
  LongTermTraj* ltt_pointer = new LongTermTraj();
  delete ltt_pointer;
}

TEST_F(LongTermTrajTestIdx, ChangeTrajectoryTest) {
  double sample_time = 0.001;
  std::vector<Motion> mo_vec;
  int n_joints = 2;
  std::vector<double> p0;
  std::vector<double> v0;
  std::vector<double> j0;
  for (int i = 0; i < n_joints; i++) {
      p0.push_back(0.0);
      v0.push_back(0.0);
      j0.push_back(0.0);
  }
  std::vector<double> a0;
  a0.push_back(12);
  a0.push_back(13);
  Motion m0(0, p0, v0, a0, j0);
  mo_vec.push_back(m0);
  std::vector<double> a1;
  a1.push_back(1);
  a1.push_back(2);
  Motion m1(1, p0, v0, a1, j0);
  mo_vec.push_back(m1);
  EXPECT_NO_THROW(long_term_trajectory_.setLongTermTrajectory(mo_vec, sample_time));
  // Sample time <= 0
  EXPECT_THROW(long_term_trajectory_.setLongTermTrajectory(mo_vec, 0.0), std::invalid_argument);
  // Wrong number of joints
  std::vector<Motion> mo_vec_2;
  std::vector<double> p0_1;
  std::vector<double> v0_1;
  std::vector<double> j0_1;
  std::vector<double> a0_1;
  Motion m0_1(0, p0_1, v0_1, a0_1, j0_1);
  mo_vec_2.push_back(m0_1);
  EXPECT_THROW(long_term_trajectory_.setLongTermTrajectory(mo_vec_2, sample_time), std::invalid_argument);
  // Wrong length
  std::vector<Motion> mo_vec_3;
  EXPECT_THROW(long_term_trajectory_.setLongTermTrajectory(mo_vec_3, sample_time), std::invalid_argument);
}

TEST_F(LongTermTrajTestIdx, GetCurrentPosTest) {
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 0);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 1);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 2);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 3);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 3);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 3);
}

TEST_F(LongTermTrajTestIdx, GetMotionTest) {
  Motion mo = long_term_trajectory_.getCurrentMotion();
  for (int i = 0; i < 2; i++) {
    EXPECT_DOUBLE_EQ(mo.getAngle()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getVelocity()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getJerk()[i], 0.0);
  }
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[0], 12.0);
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[1], 13.0);
  mo = long_term_trajectory_.getNextMotion();
  for (int i = 0; i < 2; i++) {
    EXPECT_DOUBLE_EQ(mo.getAngle()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getVelocity()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getJerk()[i], 0.0);
  }
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[0], 1.0);
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[1], 2.0);
  mo = long_term_trajectory_.getNextMotionAtIndex(5);
  for (int i = 0; i < 2; i++) {
    EXPECT_DOUBLE_EQ(mo.getAngle()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getVelocity()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getJerk()[i], 0.0);
  }
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[0], 78.0);
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[1], 79.0);
}

TEST_F(LongTermTrajTestIdx, GetTrajectoryIndexTest) {
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(-1), 0);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(3), 0);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(4), 1);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(5), 2);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(6), 3);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(10), 3);
}

TEST_F(LongTermTrajTest, MaxAccWindowTest) {
  std::vector<Motion> mo_vec;
  int n_joints = 2;
  std::vector<double> p0;
  std::vector<double> v0;
  std::vector<double> j0;
  for (int i = 0; i < n_joints; i++) {
    p0.push_back(0.0);
    v0.push_back(0.0);
    j0.push_back(0.0);
  }
  std::vector<double> a0;
  a0.push_back(12);
  a0.push_back(13);
  Motion m0(0, p0, v0, a0, j0);
  mo_vec.push_back(m0);
  std::vector<double> a1;
  a1.push_back(1);
  a1.push_back(2);
  Motion m1(1, p0, v0, a1, j0);
  mo_vec.push_back(m1);
  std::vector<double> a2;
  a2.push_back(78);
  a2.push_back(79);
  Motion m2(2, p0, v0, a2, j0);
  mo_vec.push_back(m2);
  std::vector<double> a3;
  a3.push_back(90);
  a3.push_back(91);
  Motion m3(3, p0, v0, a3, j0);
  mo_vec.push_back(m3);
  std::vector<double> a4;
  a4.push_back(57);
  a4.push_back(58);
  Motion m4(4, p0, v0, a4, j0);
  mo_vec.push_back(m4);
  std::vector<double> a5;
  a5.push_back(89);
  a5.push_back(90);
  Motion m5(5, p0, v0, a5, j0);
  mo_vec.push_back(m5);
  std::vector<double> a6;
  a6.push_back(56);
  a6.push_back(57);
  Motion m6(6, p0, v0, a6, j0);
  mo_vec.push_back(m6);
  long_term_trajectory_.setLongTermTrajectory(mo_vec);
  long_term_trajectory_.calculateMaxAccJerkWindow(mo_vec, 3);

  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(0)[0], 78);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(1)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(2)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(3)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(4)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(5)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(6)[0], 56);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(0)[1], 79);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(1)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(2)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(3)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(4)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(5)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(6)[1], 57);
}

TEST_F(LongTermTrajTest, MaxJerkWindowTest) {
  std::vector<Motion> mo_vec;
  int n_joints = 2;
  std::vector<double> p0;
  std::vector<double> v0;
  std::vector<double> a0;
  for (int i = 0; i < n_joints; i++) {
    p0.push_back(0.0);
    v0.push_back(0.0);
    a0.push_back(0.0);
  }
  std::vector<double> j0;
  j0.push_back(12);
  j0.push_back(13);
  Motion m0(0, p0, v0, a0, j0);
  mo_vec.push_back(m0);
  std::vector<double> j1;
  j1.push_back(1);
  j1.push_back(2);
  Motion m1(1, p0, v0, a0, j1);
  mo_vec.push_back(m1);
  std::vector<double> j2;
  j2.push_back(78);
  j2.push_back(79);
  Motion m2(2, p0, v0, a0, j2);
  mo_vec.push_back(m2);
  std::vector<double> j3;
  j3.push_back(90);
  j3.push_back(91);
  Motion m3(3, p0, v0, a0, j3);
  mo_vec.push_back(m3);
  std::vector<double> j4;
  j4.push_back(57);
  j4.push_back(58);
  Motion m4(4, p0, v0, a0, j4);
  mo_vec.push_back(m4);
  std::vector<double> j5;
  j5.push_back(89);
  j5.push_back(90);
  Motion m5(5, p0, v0, a0, j5);
  mo_vec.push_back(m5);
  std::vector<double> j6;
  j6.push_back(56);
  j6.push_back(57);
  Motion m6(6, p0, v0, a0, j6);
  mo_vec.push_back(m6);
  long_term_trajectory_.setLongTermTrajectory(mo_vec);
  long_term_trajectory_.calculateMaxAccJerkWindow(mo_vec, 3);

  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(0)[0], 78);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(1)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(2)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(3)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(4)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(5)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(6)[0], 56);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(0)[1], 79);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(1)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(2)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(3)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(4)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(5)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(6)[1], 57);
}

TEST_F(LongTermTrajTest, InterpolateTest) {
  std::vector<Motion> motions;
  int n_joints = 2;
  std::vector<double> p0 = {0.0, 0.0};
  std::vector<double> v0 = {1.0, 0.0};
  std::vector<double> a0 = {1.0, 0.0};
  std::vector<double> j0 = {-1.0, 0.0};
  Motion m0(0, p0, v0, a0, j0);
  motions.push_back(m0);
  std::vector<double> p1 = {0.104833333333333, 0.0};
  std::vector<double> v1 = {1.095, 0.0};
  std::vector<double> a1 = {0.9, 0.0};
  std::vector<double> j1 = {0.0, 0.0};
  Motion m1(0.1, p1, v1, a1, j1);
  motions.push_back(m1);
  std::vector<double> v_max_allowed = {2.0, 2.0};
  std::vector<double> a_max_allowed = {10.0, 10.0};
  std::vector<double> j_max_allowed = {100.0, 100.0};
  int starting_index = 0;
  long_term_trajectory_.setStartingIndex(starting_index);
  long_term_trajectory_.setVMaxAllowed(v_max_allowed);
  long_term_trajectory_.setAMaxAllowed(a_max_allowed);
  long_term_trajectory_.setJMaxAllowed(j_max_allowed);
  long_term_trajectory_.setLongTermTrajectory(motions, 0.1);
  Motion motion_int = long_term_trajectory_.interpolate(0.01, 1.0, 0.0, 0.0);
  EXPECT_NEAR(motion_int.getAngle()[0], 0.010049833333333, 1e-5);
  EXPECT_NEAR(motion_int.getVelocity()[0], 1.00995, 1e-5);
  EXPECT_NEAR(motion_int.getAcceleration()[0], 0.99, 1e-5);
  EXPECT_NEAR(motion_int.getJerk()[0], -1.0, 1e-5);
  Motion motion_int2 = long_term_trajectory_.interpolate(0.01, 0.5, 0.0, 0.0);
  EXPECT_NEAR(motion_int2.getAngle()[0], 0.010049833333333, 1e-5);
  EXPECT_NEAR(motion_int2.getVelocity()[0], 0.504975, 1e-5);
  EXPECT_NEAR(motion_int2.getAcceleration()[0], 0.2475, 1e-5);
  EXPECT_NEAR(motion_int2.getJerk()[0], -0.1250, 1e-5);
  Motion motion_int3 = long_term_trajectory_.interpolate(0.01, 0.5, 1.0, 0.2);
  EXPECT_NEAR(motion_int3.getAngle()[0], 0.010049833333333, 1e-5);
  EXPECT_NEAR(motion_int3.getVelocity()[0], 0.504975, 1e-5);
  EXPECT_NEAR(motion_int3.getAcceleration()[0], 1.25745, 1e-5);
  EXPECT_NEAR(motion_int3.getJerk()[0], 1.561990000000000, 1e-5);
}

TEST_F(LongTermTrajTestIdx, getLowerIndexTest) {
  // Sample time = 0.001
  // Trajectory length = 4
  // Starting index = 3
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(-0.1), 0);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.0), 0);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.0005), 0);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.001), 0);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.0015), 0);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.002), 0);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.003), 0);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.004), 1);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.0045), 1);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.005), 2);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.0059), 2);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.006), 3);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.0065), 3);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.007), 3);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(0.1), 3);
  EXPECT_EQ(long_term_trajectory_.getLowerIndex(1.1), 3);
}

TEST_F(LongTermTrajTestIdx, getUpperIndexTest) {
  // Sample time = 0.001
  // Trajectory length = 4
  // Starting index = 3
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(-0.1), 0);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.0), 0);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.0005), 0);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.001), 0);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.0015), 0);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.002), 0);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.003), 0);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.004), 1);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.0045), 2);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.005), 2);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(0.00501), 3);
  EXPECT_EQ(long_term_trajectory_.getUpperIndex(1.1), 3);
}

TEST_F(LongTermTrajTestIdx, getModIndexTest) {
  // Sample time = 0.001
  // Trajectory length = 4
  // Starting index = 3
  EXPECT_NEAR(long_term_trajectory_.getModIndex(0.004), 0, 1e-9);
  EXPECT_NEAR(long_term_trajectory_.getModIndex(0.0045), 0.5, 1e-9);
  EXPECT_NEAR(long_term_trajectory_.getModIndex(0.00476), 0.76, 1e-9);
  EXPECT_NEAR(long_term_trajectory_.getModIndex(0.005), 0, 1e-9);
}

TEST(LongTermTrajUtilsTest, calcTimePointsForEquidistantIntervalsTest) {
  std::vector<double> time_points = calcTimePointsForEquidistantIntervals(0.0, 1.05, 0.5);
  EXPECT_DOUBLE_EQ(time_points[0], 0.0);
  EXPECT_DOUBLE_EQ(time_points[1], 0.5);
  EXPECT_DOUBLE_EQ(time_points[2], 1.0);
  EXPECT_DOUBLE_EQ(time_points[3], 1.05);
}

TEST_F(LongTermTrajTestInterpolation, getMotionsOfAllTimeStepsFromPathTest) {
  Path path = Path();
  std::array<double, 3> times = {1, 2, 3};
  std::array<double, 3> jerks = {1, 0, -1};
  path.setPhases(times, jerks);
  std::vector<double> time_points = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
  std::vector<Motion> interval_edge_motions = getMotionsOfAllTimeStepsFromPath(long_term_trajectory_, path, time_points);
  EXPECT_EQ(interval_edge_motions.size(), time_points.size());
  EXPECT_NEAR(interval_edge_motions[0].getAngle()[0], 0.0, 1e-8);
  EXPECT_NEAR(interval_edge_motions[1].getAngle()[0], 0.02083333333, 1e-8);
  EXPECT_NEAR(interval_edge_motions[2].getAngle()[0], 0.16666666666, 1e-8);
  EXPECT_NEAR(interval_edge_motions[3].getAngle()[0], 0.54166666666, 1e-8);
  EXPECT_NEAR(interval_edge_motions[4].getAngle()[0], 1.16666666666, 1e-8);
  EXPECT_NEAR(interval_edge_motions[5].getAngle()[0], 2.02083333333, 1e-8);
  EXPECT_NEAR(interval_edge_motions[6].getAngle()[0], 3.0, 1e-8);
  EXPECT_NEAR(interval_edge_motions[7].getAngle()[0], 4.0, 1e-8);
  EXPECT_NEAR(interval_edge_motions[8].getAngle()[0], 4.0, 1e-8);
}

TEST_F(LongTermTrajTestVelocity, OverapproximativeVelocityTest) {
  double sum_deviation = 0;
  double epsilon = 1e-5;
  for (int i = 0; i < long_term_trajectory_approximate_.getLength(); i++) {
    double approximate = long_term_trajectory_approximate_.getMotion(i).getMaximumCartesianVelocity();
    double exact = long_term_trajectory_exact_.getMotion(i).getMaximumCartesianVelocity();
    sum_deviation += approximate / exact;
    EXPECT_TRUE(approximate >= exact - epsilon);
    std::cout << "exact: " << exact << ", approximate: " << approximate << std::endl;
  }
  std::cout << "average deviation from exact is " << sum_deviation / long_term_trajectory_approximate_.getLength() - 1
            << std::endl;
}

TEST_F(LongTermTrajTest, InterpolateTestConst) {
  std::vector<Motion> motions;
  int n_joints = 1;
  std::vector<double> p0 = {0.0};
  std::vector<double> v0 = {1.0};
  std::vector<double> a0 = {0.0};
  std::vector<double> j0 = {0.0};
  Motion m0(0, p0, v0, a0, j0);
  motions.push_back(m0);
  std::vector<double> p1 = {0.001};
  std::vector<double> v1 = {1.0};
  std::vector<double> a1 = {0.0};
  std::vector<double> j1 = {0.0};
  Motion m1(0.001, p1, v1, a1, j1);
  motions.push_back(m1);
  std::vector<double> v_max_allowed = {2.0};
  std::vector<double> a_max_allowed = {10.0};
  std::vector<double> j_max_allowed = {100.0};
  int starting_index = 0;
  long_term_trajectory_.setStartingIndex(starting_index);
  long_term_trajectory_.setVMaxAllowed(v_max_allowed);
  long_term_trajectory_.setAMaxAllowed(a_max_allowed);
  long_term_trajectory_.setJMaxAllowed(j_max_allowed);
  long_term_trajectory_.setLongTermTrajectory(motions, 0.001);
  // Slowing down on trajectory
  double s = 0.001/2.0;
  double ds = 0.7;
  double dds = -0.5;
  double ddds = -2;
  Motion motion_int = long_term_trajectory_.interpolate(s, ds, dds, ddds);
  EXPECT_NEAR(motion_int.getAngle()[0], 0.001/2.0, 1e-8);
  EXPECT_NEAR(motion_int.getVelocity()[0], 0.7, 1e-8);
  EXPECT_NEAR(motion_int.getAcceleration()[0], -0.5, 1e-8);
  EXPECT_NEAR(motion_int.getJerk()[0], -2, 1e-5);
}

TEST_F(LongTermTrajTest, InterpolateTestStanding) {
  std::vector<Motion> motions;
  int n_joints = 1;
  std::vector<double> p0 = {0.0};
  std::vector<double> v0 = {1.0};
  std::vector<double> a0 = {1.0};
  std::vector<double> j0 = {0.0};
  Motion m0(0, p0, v0, a0, j0);
  motions.push_back(m0);
  std::vector<double> p1 = {0.001};  // Not fully correct but not tested here.
  std::vector<double> v1 = {1.001};
  std::vector<double> a1 = {1.0};
  std::vector<double> j1 = {0.0};
  Motion m1(0.001, p1, v1, a1, j1);
  motions.push_back(m1);
  std::vector<double> v_max_allowed = {2.0};
  std::vector<double> a_max_allowed = {10.0};
  std::vector<double> j_max_allowed = {100.0};
  int starting_index = 0;
  long_term_trajectory_.setStartingIndex(starting_index);
  long_term_trajectory_.setVMaxAllowed(v_max_allowed);
  long_term_trajectory_.setAMaxAllowed(a_max_allowed);
  long_term_trajectory_.setJMaxAllowed(j_max_allowed);
  long_term_trajectory_.setLongTermTrajectory(motions, 0.001);
  // Stopped on trajectory
  double s = 0.001/2.0;
  double ds = 0.0;
  double dds = 0.0;
  double ddds = 0.0;
  Motion motion_int = long_term_trajectory_.interpolate(s, ds, dds, ddds);
  EXPECT_NEAR(motion_int.getVelocity()[0], 0.0, 1e-8);
  EXPECT_NEAR(motion_int.getAcceleration()[0], 0.0, 1e-8);
  EXPECT_NEAR(motion_int.getJerk()[0], 0.0, 1e-5);
}

TEST_F(LongTermTrajTest, InterpolateTestStandingLTT) {
  std::vector<Motion> motions;
  int n_joints = 1;
  std::vector<double> p0 = {0.0};
  std::vector<double> v0 = {0.0};
  std::vector<double> a0 = {0.0};
  std::vector<double> j0 = {0.0};
  Motion m0(0, p0, v0, a0, j0);
  motions.push_back(m0);
  std::vector<double> p1 = {0.0};  // Not fully correct but not tested here.
  std::vector<double> v1 = {0.0};
  std::vector<double> a1 = {0.0};
  std::vector<double> j1 = {0.0};
  Motion m1(0.001, p1, v1, a1, j1);
  motions.push_back(m1);
  std::vector<double> v_max_allowed = {2.0};
  std::vector<double> a_max_allowed = {10.0};
  std::vector<double> j_max_allowed = {100.0};
  int starting_index = 0;
  long_term_trajectory_.setStartingIndex(starting_index);
  long_term_trajectory_.setVMaxAllowed(v_max_allowed);
  long_term_trajectory_.setAMaxAllowed(a_max_allowed);
  long_term_trajectory_.setJMaxAllowed(j_max_allowed);
  long_term_trajectory_.setLongTermTrajectory(motions, 0.001);
  // Stopped on trajectory
  double s = 0.001/2.0;
  double ds = 0.0;
  double dds = 1.0;
  double ddds = 10.0;
  Motion motion_int = long_term_trajectory_.interpolate(s, ds, dds, ddds);
  EXPECT_NEAR(motion_int.getAngle()[0], 0.0, 1e-8);
  EXPECT_NEAR(motion_int.getVelocity()[0], 0.0, 1e-8);
  EXPECT_NEAR(motion_int.getAcceleration()[0], 0.0, 1e-8);
  EXPECT_NEAR(motion_int.getJerk()[0], 0.0, 1e-5);
}

TEST_F(LongTermTrajTest, InterpolateTestAcceleratingLTT) {
  std::vector<Motion> motions;
  int n_joints = 1;
  std::vector<double> p0 = {0.0};
  std::vector<double> v0 = {0.0};
  std::vector<double> a0 = {1.0};
  std::vector<double> j0 = {0.0};
  Motion m0(0, p0, v0, a0, j0);
  motions.push_back(m0);
  std::vector<double> p1 = {0.0};  // Not fully correct but not tested here.
  std::vector<double> v1 = {0.001};
  std::vector<double> a1 = {1.0};
  std::vector<double> j1 = {0.0};
  Motion m1(0.001, p1, v1, a1, j1);
  motions.push_back(m1);
  std::vector<double> v_max_allowed = {2.0};
  std::vector<double> a_max_allowed = {10.0};
  std::vector<double> j_max_allowed = {100.0};
  int starting_index = 0;
  long_term_trajectory_.setStartingIndex(starting_index);
  long_term_trajectory_.setVMaxAllowed(v_max_allowed);
  long_term_trajectory_.setAMaxAllowed(a_max_allowed);
  long_term_trajectory_.setJMaxAllowed(j_max_allowed);
  long_term_trajectory_.setLongTermTrajectory(motions, 0.001);
  // Stopped on trajectory
  double s = 0.0;
  double ds = 1.0;
  double dds = 0.0;
  double ddds = 0.0;
  Motion motion_int = long_term_trajectory_.interpolate(s, ds, dds, ddds);
  EXPECT_NEAR(motion_int.getAngle()[0], 0.0, 1e-8);
  EXPECT_NEAR(motion_int.getVelocity()[0], 0.0, 1e-8);
  EXPECT_NEAR(motion_int.getAcceleration()[0], 1.0, 1e-8);
  EXPECT_NEAR(motion_int.getJerk()[0], 0.0, 1e-5);
}

TEST_F(LongTermTrajInterpolateWithVelocityTest, InterpolateWithCartVelTest) {
  Motion motion_int = long_term_trajectory_.interpolate(0.01, 1.0, 0.0, 0.0);
  EXPECT_NEAR(motion_int.getAngle()[0], 0.010049833333333, 1e-5);
  EXPECT_NEAR(motion_int.getVelocity()[0], 1.00995, 1e-5);
  EXPECT_NEAR(motion_int.getAcceleration()[0], 0.99, 1e-5);
  EXPECT_NEAR(motion_int.getJerk()[0], -1.0, 1e-5);
  double cart_v_max_0 = motion_int.getMaximumCartesianVelocities()[0];
  Motion motion_int2 = long_term_trajectory_.interpolate(0.01, 0.5, 0.0, 0.0);
  EXPECT_NEAR(motion_int2.getAngle()[0], 0.010049833333333, 1e-5);
  EXPECT_NEAR(motion_int2.getVelocity()[0], 0.504975, 1e-5);
  EXPECT_NEAR(motion_int2.getAcceleration()[0], 0.2475, 1e-5);
  EXPECT_NEAR(motion_int2.getJerk()[0], -0.1250, 1e-5);
  EXPECT_NEAR(motion_int2.getMaximumCartesianVelocities()[0], 0.5 * cart_v_max_0, 1e-5);
  Motion motion_int3 = long_term_trajectory_.interpolate(0.01, 0.5, 1.0, 0.2);
  EXPECT_NEAR(motion_int3.getAngle()[0], 0.010049833333333, 1e-5);
  EXPECT_NEAR(motion_int3.getVelocity()[0], 0.504975, 1e-5);
  EXPECT_NEAR(motion_int3.getAcceleration()[0], 1.25745, 1e-5);
  EXPECT_NEAR(motion_int3.getJerk()[0], 1.561990000000000, 1e-5);
  EXPECT_NEAR(motion_int3.getMaximumCartesianVelocities()[0], 0.5 * cart_v_max_0, 1e-5);
}

TEST_F(LongTermTrajInterpolateWithVelocityTest, GetMaxCartVelTest) {
  double v_max = long_term_trajectory_.getMaxofMaximumCartesianVelocity();
  double v_pred = 0.11 * 1.095;
  EXPECT_NEAR(v_max, v_pred, 1e-5);
  double v_max_0 = long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(0.0);
  v_pred = 0.11 * 1.0;
  EXPECT_NEAR(v_max_0, v_pred, 1e-5);
  auto inertia_matrices = long_term_trajectory_.getInertiaMatrices(0);
}

TEST_F(LongTermTrajInterpolateWithVelocityTest, getInertiaMatricesOfAllTimeStepsFromPathTest) {
  Path path = Path();
  std::array<double, 3> times = {1, 2, 3};
  std::array<double, 3> jerks = {1, 0, -1};
  path.setPhases(times, jerks);
  std::vector<double> time_points = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
  std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> inertia_matrices = getInertiaMatricesOfAllTimeStepsFromPath(long_term_trajectory_, path, time_points);
  EXPECT_EQ(inertia_matrices.size(), time_points.size());
  EXPECT_EQ(inertia_matrices[0].size(), 1);
}


} // namespace safety_shield

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}