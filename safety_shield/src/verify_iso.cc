#include "safety_shield/verify_iso.h"

namespace safety_shield {

bool VerifyISO::robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules,
                                    const std::vector<reach_lib::Capsule>& human_capsules) {
  // Check position capsules
  for (auto& human_capsule : human_capsules) {
    for (auto& robot_capsule : robot_capsules) {
      // If there is a collision, return true
      if (capsuleCollisionCheck(robot_capsule, human_capsule)) {
        return true;
      }
    }
  }
  return false;
}

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

/// checks safety for each time step
bool VerifyISO::improved_verify_human_reach(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
                                 std::vector<std::vector<std::vector<reach_lib::Capsule>>> human_reachable_sets) {
  try {
    assert(robot_reachable_sets.size() == human_reachable_sets.size());
    for(int i = 0; i < robot_reachable_sets.size(); i++) {
      bool temp = verify_human_reach(robot_reachable_sets[i], human_reachable_sets[i]);
      if(!temp) {
        // returns false if at least in one time step, there is a collision
        return false;
      }
    }
    // returns true if it was safe in all time steps
    return true;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in VerifyISO::improved_verify_human_reach: {}", exc.what());
    return false;
  }
}

}  // namespace safety_shield