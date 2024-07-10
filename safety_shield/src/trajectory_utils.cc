#include "safety_shield/trajectory_utils.h"

namespace safety_shield {

std::vector<double> calcTimePointsForEquidistantIntervals(double begin, double end, double reachability_set_duration) {
  std::vector<double> time_points;
  double current_time = begin;
  while (current_time < end) {
    time_points.push_back(current_time);
    current_time += reachability_set_duration;
  }
  time_points.push_back(end);
  return time_points;
}

std::vector<Motion> getMotionsOfAllTimeStepsFromPath(const LongTermTraj& ltt, const Path& path, const std::vector<double>& time_points) {
  Path path_copy = path;
  std::vector<Motion> motion_samples;
  motion_samples.push_back(ltt.interpolate(path_copy.getPosition(), path_copy.getVelocity(), path_copy.getAcceleration(), path_copy.getJerk()));
  for (int i = 0; i < time_points.size() - 1; i++) {
    // increment path and push motion to list
    double time_interval_duration = time_points[i + 1] - time_points[i];
    path_copy.increment(time_interval_duration);
    Motion motion = ltt.interpolate(path_copy.getPosition(), path_copy.getVelocity(), path_copy.getAcceleration(), path_copy.getJerk());
    motion_samples.push_back(motion);
  }
  return motion_samples;
}
}