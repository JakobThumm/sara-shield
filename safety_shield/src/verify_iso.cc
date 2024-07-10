#include "safety_shield/verify_iso.h"

namespace safety_shield {

bool VerifyISO::verify_human_reach(const std::vector<reach_lib::Capsule>& robot_capsules,
                                   std::vector<std::vector<reach_lib::Capsule>> human_capsules) {
  try {
    for (const auto& capsule_list : human_capsules) {
      // If no collision occured, we are safe and don't have to check the rest.
      if (!robotHumanCollision(robot_capsules, capsule_list)) {
        return true;
      }
    }
    return false;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in VerifyISO::verify_human_reach: {}", exc.what());
    return false;
  }
}

/// checks safety for each time interval
bool VerifyISO::verify_human_reach_time_intervals(
  const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
  const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
  int& collision_index
) {
  assert(robot_reachable_sets.size() == human_reachable_sets.size());
  try {
    for (int i = 0; i < robot_reachable_sets.size(); i++) {
      if (!verify_human_reach(robot_reachable_sets[i], human_reachable_sets[i])) {
        // returns false if at least in one time step, there is a collision
        collision_index = i;
        return false;
      }
    }
    // returns true if it was safe in all time intervals
    collision_index = -1;
    return true;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in VerifyISO::improved_verify_human_reach: {}", exc.what());
    collision_index = 0;
    return false;
  }
}

bool VerifyISO::verify_human_reach_velocity(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
  const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
  const std::vector<std::vector<double>>& robot_link_velocities,
  const std::vector<std::vector<double>>& maximal_contact_velocities,
  int& collision_index) {
  assert(robot_reachable_sets.size() == human_reachable_sets.size());  // same number of time intervals
  assert(robot_reachable_sets.size() == robot_link_velocities.size());  // same number of time intervals
  assert(robot_reachable_sets[0].size() == robot_link_velocities[0].size());  // same number of robot links
  assert(human_reachable_sets[0].size() == maximal_contact_velocities.size());  // same number of human models
  assert(human_reachable_sets[0][0].size() == maximal_contact_velocities[0].size());  // same number of human bodies
  int n_time_intervals = robot_reachable_sets.size();
  int n_robot_links = robot_reachable_sets[0].size();
  int n_human_models = human_reachable_sets[0].size();
  int n_human_bodies = human_reachable_sets[0][0].size();
  try {
    for (int i = 0; i < n_time_intervals; i++) {
      for (int m = 0; m < n_human_models; m++) {
        std::map<int, std::vector<int>> human_robot_contacts = find_all_human_robot_contacts(human_reachable_sets[i][m], robot_reachable_sets[i]);
        /*
        if (verify_human_reach(robot_reachable_sets[i], human_reachable_sets[i])) {
          // No collision in this time interval possible.
          continue;
        }
        */
      }
    }
    // returns true if it was safe in all time intervals
    collision_index = -1;
    return true;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in VerifyISO::improved_verify_human_reach: {}", exc.what());
    collision_index = 0;
    return false;
  }
}

}  // namespace safety_shield