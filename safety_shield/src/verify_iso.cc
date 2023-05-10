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
  // Build a map that maps the robot capsule/environment indices in collision with the human capsules
  // We choose this indexing to be in line with the paper.
  std::map<int, std::vector<int>> robot_collision_map;
  std::map<int, std::vector<int>> environment_collision_map;
  for (int j = 0; j < human_capsules.size(); j++) {
    // Collisions with robot links
    for (int i = 0; i < robot_capsules.size(); i++) {
      if (capsuleCollisionCheck(human_capsules[j], robot_capsules[i])) {
        // Robot link i and human body part j could intersect.
        if (robot_collision_map.find(j) == robot_collision_map.end()) {
          robot_collision_map[j] = std::vector<int>();
        }
        robot_collision_map[j].push_back(i);
      }
    }
    // Maybe:
    // if (robot_collision_map.find(j) == robot_collision_map.end()) {
    //   continue;
    // }
    // Collisions with static environment elements
    for (int k = 0; k < environment_elements.size(); k++) {
      if (reach_lib::intersections::capsule_aabb_intersection(human_capsules[j], environment_elements[k])) {
        // Human body part j and environment element k could intersect.
        if (environment_collision_map.find(j) == environment_collision_map.end()) {
          environment_collision_map[j] = std::vector<int>();
        }
        environment_collision_map[j].push_back(k);
      }
    }
  }
  for (const auto& robot_collisions : robot_collision_map) {
    if (robot_collisions.second.size() >= 2) {
      // Self-constrained collision possible
      return true;
    }
    if (environment_collision_map.find(robot_collisions.first) != environment_collision_map.end()) {
      // Environment collision possible
      return true;
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