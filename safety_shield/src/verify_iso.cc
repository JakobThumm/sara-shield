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
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<double>& human_radii) {
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
    int human_index = robot_collisions.first;
    if (robot_collisions.second.size() >= 2) {
      // Self-constrained collision possible
      // Check distance between the two links
      return true;
    }
    if (environment_collision_map.find(human_index) != environment_collision_map.end()) {
      // Environment collision possible
      // Check distance between link and environment
      // by expanding the link capsule by the human body diameter
      // and checking for intersection with the environment element.
      // diameter of the human lower leg. #TODO: make this a parameter
      double d_human = 2 * human_radii[human_index];
      for (const auto& environment_collision : environment_collision_map[human_index]) {
        reach_lib::Capsule expanded_robot_capsule(
          reach_lib::Point(
            robot_capsules[robot_collisions.second[0]].p1_.x,
            robot_capsules[robot_collisions.second[0]].p1_.y,
            robot_capsules[robot_collisions.second[0]].p1_.z),
          reach_lib::Point(
            robot_capsules[robot_collisions.second[0]].p2_.x,
            robot_capsules[robot_collisions.second[0]].p2_.y,
            robot_capsules[robot_collisions.second[0]].p2_.z),
          robot_capsules[robot_collisions.second[0]].r_ + d_human);
        if (reach_lib::intersections::capsule_aabb_intersection(expanded_robot_capsule,
            environment_elements[environment_collision])) {
          return true;
        }
      }
    }
  }
  return false;
}

bool VerifyISO::verify_clamping(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<std::vector<reach_lib::Capsule>>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<std::vector<double>>& human_radii) {
  try {
    for (int i = 0; i < human_capsules.size(); i++) {
      // If no collision occured, we are safe and don't have to check the rest.
      if(!clamping_possible(robot_capsules, human_capsules[i], environment_elements, human_radii[i])) {
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