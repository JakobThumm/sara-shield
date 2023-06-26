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
    std::vector<int> environment_collisions = find_human_environment_contact(human_capsules[i], environment_elements);
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
      if (link_pair_unclampable(link1, link2, unclampable_enclosures_map)) {
        continue;
      }
      // Check if the two links can cause a self-constrained collision
      reach_lib::Capsule expanded_robot_capsule = create_expanded_capsule(robot_capsules[link1], d_human);
      if (capsuleCollisionCheck(expanded_robot_capsule, robot_capsules[link2])) {
        // Self-constrained collision detected
        return true;
      }
    }
  }
  // No collision was possible
  return false;
}

bool VerifyISO::calculate_normal_vector(const reach_lib::Capsule& robot_capsule,
      const reach_lib::AABB& environment_element,
      Eigen::Vector3d& normal) {
  // Calculate the normal vectors of the environment element
  std::vector<Eigen::Vector3d> normals;
  for (int i = 0; i < 3; i++) {
    // Check if robot capsule is left of left face of AABB
    bool left_of_face = false;
    double pos_1, pos_2;
    switch(i) {
      case 0:
        pos_1 = robot_capsule.p1_.x;
        pos_2 = robot_capsule.p2_.x;
        break;
      case 1:
        pos_1 = robot_capsule.p1_.y;
        pos_2 = robot_capsule.p2_.y;
        break;
      case 2:
        pos_1 = robot_capsule.p1_.z;
        pos_2 = robot_capsule.p2_.z;
        break;
      default:
        std::cout << "Error: Invalid index for normal vector calculation." << std::endl;
        return false;
    }
    if (pos_1 < environment_element.min_[i] || pos_2 < environment_element.min_[i]) {
      Eigen::Vector3d n(0, 0, 0);
      n[i] = -1;
      normals.push_back(n);
      left_of_face = true;
    }
    // Check if robot capsule is right of right face of AABB
    bool right_of_face = false;
    if (pos_1 > environment_element.max_[i] || pos_2 > environment_element.max_[i]) {
      Eigen::Vector3d n(0, 0, 0);
      n[i] = 1;
      normals.push_back(n);
      right_of_face = true;
    }
    if (left_of_face && right_of_face) {
      // Robot capsule is outside of environment element
      return false;
    }
  }
  if (normals.size() == 0) {
    // Robot capsule is inside of environment element
    return false;
  }
  assert(normals.size() <= 3);
  // Calculate the final normal vector of the environment element
  normal << 0.0, 0.0, 0.0;
  for (const Eigen::Vector3d& n : normals) {
    normal += n;
  }
  normal.normalize();
  return true;
}

bool VerifyISO::capsule_moving_towards_element(const RobotReach::CapsuleVelocity velocity_capsule,
      const Eigen::Vector3d& normal,
      double radius,
      double velocity_error) {
  // d = v \cdot n
  double d1 = velocity_capsule.v1.v.dot(normal);
  double d2 = velocity_capsule.v2.v.dot(normal);
  // d_{\omega} >= - |r| * |n x \omega|
  double dw1 = - abs(radius) * normal.cross(velocity_capsule.v1.w).norm();
  double dw2 = - abs(radius) * normal.cross(velocity_capsule.v2.w).norm();
  // d + d_{\omega} < 0 -> link is moving towards environment element
  return (d1 + dw1 < velocity_error || d2 + dw2 < velocity_error);
}

bool VerifyISO::capsule_trajectory_moving_towards_element(
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_start,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_end,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      int capsule_index,
      const Eigen::Vector3d& normal,
      double velocity_error) {
  // For every velocity of the link, check if the link is moving towards the environment element normal vector
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it = robot_capsule_velocities_start;
  while (robot_capsule_velocities_it != robot_capsule_velocities_end) {
    if (capsule_moving_towards_element((*robot_capsule_velocities_it)[capsule_index], normal, robot_capsules[capsule_index].r_, velocity_error)) {
      return true;
    }
    robot_capsule_velocities_it++;
  }
  return false;
}

bool VerifyISO::environmentally_constrained_collision_check(const std::vector<int>& robot_collisions,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      const std::vector<int>& environment_collisions,
      const std::vector<reach_lib::AABB>& environment_elements,
      double d_human,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_start,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_end,
      const std::vector<double>& alpha_i,
      const std::vector<double>& beta_i,
      double delta_s) {
  // Check distance between link and environment
  // by expanding the link capsule by the human body diameter
  // and checking for intersection with the environment element.
  for (const int& environment_collision : environment_collisions) {
    for (const int& link_index : robot_collisions) {
      spdlog::info("Checking link {} against environment element {}", link_index, environment_collision);
      reach_lib::Capsule expanded_robot_capsule = create_expanded_capsule(robot_capsules[link_index], d_human);
      if (!reach_lib::intersections::capsule_aabb_intersection(expanded_robot_capsule,
          environment_elements[environment_collision])) {
        continue;
      }
      spdlog::info("Collision is possible between link {} and environment element {}", link_index, environment_collision);
      // Check if the link is moving towards the environment element
      // Find normal vector on the environment element pointing towards the link
      Eigen::Vector3d normal;
      if (!calculate_normal_vector(robot_capsules[link_index], environment_elements[environment_collision], normal)) {
        // If the calculation of the normal vector fails, the velocity criterion fails.
        spdlog::info("Could not calculate normal vector for link {} and environment element {}", link_index, environment_collision);
        return true;
      }
      // Calculate maximal velocity error
      // \epsilon <= 1/2 \Delta s (\alpha_i + \beta_i * r_i)
      assert (link_index < alpha_i.size());
      assert (link_index < beta_i.size());
      double velocity_error = 1/2 * delta_s * (alpha_i[link_index] + beta_i[link_index] * robot_capsules[link_index].r_);
      if (capsule_trajectory_moving_towards_element(robot_capsule_velocities_start, robot_capsule_velocities_end, robot_capsules, link_index, normal, velocity_error)) {
        spdlog::info("Link {} moving towards environment element {}", link_index, environment_collision);
        return true;
      }
      spdlog::info("Collision between link {} and environment element {} was disregarded.", link_index, environment_collision);
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
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end,
      const std::vector<double>& alpha_i,
      const std::vector<double>& beta_i,
      double delta_s) {
  // Build a map that maps the robot capsule/environment indices in collision with the human capsules
  // We choose this indexing to be in line with the paper.
  assert(robot_capsules.size() == robot_capsule_velocities_it->size());
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
        environment_collision_map.at(human_index), environment_elements, d_human,
        robot_capsule_velocities_it, robot_capsule_velocities_end, alpha_i, beta_i, delta_s)) {
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
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end,
      const std::vector<double>& alpha_i,
      const std::vector<double>& beta_i,
      double delta_s) 
{
  try {
    for (int i = 0; i < human_capsules.size(); i++) {
      // If no collision occured, we are safe and don't have to check the rest.
      if (!clamping_possible(robot_capsules, human_capsules[i], environment_elements, human_radii[i], unclampable_enclosures_map,
          robot_capsule_velocities_it, robot_capsule_velocities_end, alpha_i, beta_i, delta_s)) {
        spdlog::info("Clamping is not possible for human capsule set {}", i);
        return true;
      }
    }
    spdlog::info("Clamping is possible for all human capsule sets");
    return false;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_clamping: {}", exc.what());
    return false;
  }
}
} // namespace safety_shield
