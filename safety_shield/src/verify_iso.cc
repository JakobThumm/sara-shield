#include "safety_shield/verify_iso.h"

namespace safety_shield {


bool VerifyISO::robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules) {
  // Check position capsules
  for(auto& human_capsule : human_capsules) {
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
      if(!robotHumanCollision(robot_capsules, capsule_list)) {
        return true;
      }
    }
    return false;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_human_reach: {}", exc.what());
    return false;
  }
}

bool VerifyISO::clamping_possible(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements) {
  for (const auto& human_capsule : human_capsules) {
    for (const auto& environment_element : environment_elements) {
      if (reach_lib::intersections::capsule_aabb_intersection(human_capsule, environment_element)) {
        for (const auto& robot_capsule : robot_capsules) {
          if (capsuleCollisionCheck(human_capsule, robot_capsule)) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool VerifyISO::verify_clamping(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<std::vector<reach_lib::Capsule>>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements) {
  try {
    for (const auto& capsule_list : human_capsules) {
      // If no collision occured, we are safe and don't have to check the rest.
      if(!clamping_possible(robot_capsules, capsule_list, environment_elements)) {
        return true;
      }
    }
    return false;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_clamping: {}", exc.what());
    return false;
  }
}
} // namespace safety_shield