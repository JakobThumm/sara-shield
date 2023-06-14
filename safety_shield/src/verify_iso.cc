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

std::vector<int> VerifyISO::find_human_robot_contact(const reach_lib::Capsule& human_capsule,
      const std::vector<reach_lib::Capsule>& robot_capsules) {
  std::vector<int> human_robot_collisions;
  for (int i = 0; i < robot_capsules.size(); i++) {
    if (capsuleCollisionCheck(human_capsule, robot_capsules[i])) {
      // Robot link i and human body part j could intersect.
      human_robot_collisions.push_back(i);
    }
  }
  return human_robot_collisions;
}

std::vector<int> VerifyISO::find_human_environment_contact(const reach_lib::Capsule& human_capsule,
      const std::vector<reach_lib::AABB>& environment_elements) {
  std::vector<int> human_environment_collisions;
  for (int i = 0; i < environment_elements.size(); i++) {
      if (reach_lib::intersections::capsule_aabb_intersection(human_capsule, environment_elements[i])) {
        // Robot link i and human body part j could intersect.
        human_environment_collisions.push_back(i);
      }
  }
  return human_environment_collisions;
}

void VerifyISO::build_contact_maps(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      std::unordered_map<int, std::vector<int>>& robot_collision_map,
      std::unordered_map<int, std::vector<int>>& environment_collision_map) {
  for (int i = 0; i < human_capsules.size(); i++) {
    // Collisions with robot links
    std::vector<int> robot_collisions = find_human_robot_contact(human_capsules[i], robot_capsules);
    if (robot_collisions.size() > 0) {
      if (robot_collision_map.find(i) == robot_collision_map.end()) {
        robot_collision_map[i] = std::vector<int>();
      }
      robot_collision_map[i].insert(robot_collision_map[i].end(), robot_collisions.begin(), robot_collisions.end());
    }
    // Maybe:
    // if (robot_collision_map.find(i) == robot_collision_map.end()) {
    //   continue;
    // }
    // Collisions with static environment elements
    std::vector<int> environment_collisions = find_human_robot_contact(human_capsules[i], robot_capsules);
    if (environment_collisions.size() > 0) {
      if (environment_collision_map.find(i) == environment_collision_map.end()) {
        environment_collision_map[i] = std::vector<int>();
      }
      environment_collision_map[i].insert(environment_collision_map[i].end(), environment_collisions.begin(), environment_collisions.end());
    }
  }
}

bool VerifyISO::self_constrained_collision_check(const std::vector<int>& robot_collisions,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      double d_human) {
  // At least two links must be in contact with the human for an SCC.
  if (robot_collisions.size() < 2) {
    return false;
  }
  // Create pairs of robot links from list of links in collision
  for (int i = 0; i < robot_collisions.size()-1; i++) {
    for (int j = i + 1; j < robot_collisions.size(); j++) {
      // Check if the two links cannot cause a self-constrained collision
      int link1 = robot_collisions[i];
      int link2 = robot_collisions[j];
      if ((unclampable_enclosures_map.find(link1) != unclampable_enclosures_map.end() &&
          unclampable_enclosures_map.at(link1).find(link2) != unclampable_enclosures_map.at(link1).end()) ||
          (unclampable_enclosures_map.find(link2) != unclampable_enclosures_map.end() &&
          unclampable_enclosures_map.at(link2).find(link1) != unclampable_enclosures_map.at(link2).end())) {
        continue;
      }
      // Check if the two links can cause a self-constrained collision
      reach_lib::Capsule expanded_robot_capsule(
        reach_lib::Point(
          robot_capsules[link1].p1_.x,
          robot_capsules[link1].p1_.y,
          robot_capsules[link1].p1_.z),
        reach_lib::Point(
          robot_capsules[link1].p2_.x,
          robot_capsules[link1].p2_.y,
          robot_capsules[link1].p2_.z),
        robot_capsules[link1].r_ + d_human);
      if (capsuleCollisionCheck(expanded_robot_capsule, robot_capsules[link2])) {
        // Self-constrained collision detected
        return true;
      }
    }
  }
  // No collision was possible
  return false;
}

bool VerifyISO::environmentally_constrained_collision_check(const std::vector<int>& robot_collisions,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      const std::vector<int>& environment_collisions,
      const std::vector<reach_lib::AABB>& environment_elements,
      double d_human) {
  // Check distance between link and environment
  // by expanding the link capsule by the human body diameter
  // and checking for intersection with the environment element.
  for (const int& environment_collision : environment_collisions) {
    for (const int& robot_collision : robot_collisions) {
      reach_lib::Capsule expanded_robot_capsule(
        reach_lib::Point(
          robot_capsules[robot_collision].p1_.x,
          robot_capsules[robot_collision].p1_.y,
          robot_capsules[robot_collision].p1_.z),
        reach_lib::Point(
          robot_capsules[robot_collision].p2_.x,
          robot_capsules[robot_collision].p2_.y,
          robot_capsules[robot_collision].p2_.z),
        robot_capsules[robot_collision].r_ + d_human);
      if (reach_lib::intersections::capsule_aabb_intersection(expanded_robot_capsule,
          environment_elements[environment_collision])) {
        return true;
      }
    }
  }
  return false;
}

bool VerifyISO::clamping_possible(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<double>& human_radii,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end) {
  // TODO: Write more modular!
  // Build a map that maps the robot capsule/environment indices in collision with the human capsules
  // We choose this indexing to be in line with the paper.
  std::unordered_map<int, std::vector<int>> robot_collision_map;
  std::unordered_map<int, std::vector<int>> environment_collision_map;
  build_contact_maps(robot_capsules, human_capsules, environment_elements, robot_collision_map, environment_collision_map);
  // Check for self-constrained collisions and environmentally-constrained collisions
  for (const auto& robot_collisions : robot_collision_map) {
    int human_index = robot_collisions.first;
    double d_human = 2 * human_radii[human_index];
    // Self-constrained collision check
    if (self_constrained_collision_check(robot_collisions.second, unclampable_enclosures_map, robot_capsules, d_human)) {
      return true;
    }
    // Environmentally-constrained collision check
    if (environment_collision_map.find(human_index) == environment_collision_map.end()) {
      continue;
    }
    if (environmentally_constrained_collision_check(robot_collisions.second, robot_capsules,
        environment_collision_map.at(human_index), environment_elements, d_human)) {
      return true;
    }
  }
  return false;
}

bool VerifyISO::verify_clamping(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<std::vector<reach_lib::Capsule>>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<std::vector<double>>& human_radii,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end) 
{
  try {
    for (int i = 0; i < human_capsules.size(); i++) {
      // If no collision occured, we are safe and don't have to check the rest.
      if(!clamping_possible(robot_capsules, human_capsules[i], environment_elements, human_radii[i], unclampable_enclosures_map,
          robot_capsule_velocities_it, robot_capsule_velocities_end)) {
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
