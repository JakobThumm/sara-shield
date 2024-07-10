#include <gtest/gtest.h>
#include "safety_shield/trajectory_utils.h"

TEST(TrajectoryUtilsTests, CalcTimePointsForEquidistantIntervalsTest0) {
  double start = 1.0;
  double end = 4.5;
  double duration = 1.0;
  std::vector<double> time_points = safety_shield::calcTimePointsForEquidistantIntervals(start, end, duration);
  EXPECT_DOUBLE_EQ(time_points[0], 1.0);
  EXPECT_DOUBLE_EQ(time_points[1], 2.0);
  EXPECT_DOUBLE_EQ(time_points[2], 3.0);
  EXPECT_DOUBLE_EQ(time_points[3], 4.0);
  EXPECT_DOUBLE_EQ(time_points[4], 4.5);
}

/// when improved verification falls back to standard mode
TEST(TrajectoryUtilsTests, CalcTimePointsForEquidistantIntervalsTest1) {
  /// reachability_set_duration equal to s_diff
  std::vector<double> time_points = safety_shield::calcTimePointsForEquidistantIntervals(0, 0.001, 0.001);
  EXPECT_NEAR(time_points[0], 0, 1e-6);
  EXPECT_NEAR(time_points[1], 0.001, 1e-6);

  /// reachability_set_duration larger than s_diff
  std::vector<double> time_points_other = safety_shield::calcTimePointsForEquidistantIntervals(0, 0.001, 1);
  EXPECT_NEAR(time_points_other[0], 0, 1e-6);
  EXPECT_NEAR(time_points_other[1], 0.001, 1e-6);
}

/// splitting up reachability set in several intervals
TEST(TrajectoryUtilsTests, CalcTimePointsForEquidistantIntervalsTest2) {
  /// last interval same size as all other intervals
  std::vector<double> time_points = safety_shield::calcTimePointsForEquidistantIntervals(0, 0.01, 0.005);
  EXPECT_NEAR(time_points[0], 0, 1e-6);
  EXPECT_NEAR(time_points[1], 0.005, 1e-6);
  EXPECT_NEAR(time_points[2], 0.01, 1e-6);

  /// last interval not same size as all other intervals
  std::vector<double> time_points_other = safety_shield::calcTimePointsForEquidistantIntervals(0, 0.01, 0.007);
  EXPECT_NEAR(time_points_other[0], 0, 1e-6);
  EXPECT_NEAR(time_points_other[1], 0.007, 1e-6);
  EXPECT_NEAR(time_points_other[2], 0.01, 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}