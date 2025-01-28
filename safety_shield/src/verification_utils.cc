#include "safety_shield/verification_utils.h"

namespace safety_shield {

bool robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules,
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

std::vector<int> findHumanRobotContact(const reach_lib::Capsule& human_capsule,
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

std::map<int, std::vector<int>> findAllHumanRobotContacts(const std::vector<reach_lib::Capsule>& human_capsule,
      const std::vector<reach_lib::Capsule>& robot_capsules) {
  std::map<int, std::vector<int>> human_robot_contacts;
  for (int i = 0; i < human_capsule.size(); i++) {
    human_robot_contacts[i] = findHumanRobotContact(human_capsule[i], robot_capsules);
  }
  return human_robot_contacts;
}

std::vector<int> findHumanEnvironmentContacts(const reach_lib::Capsule& human_capsule,
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

void buildContactMaps(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      std::unordered_map<int, std::unordered_set<int>>& robot_collision_map,
      std::unordered_map<int, std::unordered_set<int>>& environment_collision_map) {
  for (int i = 0; i < human_capsules.size(); i++) {
    // Collisions with robot links
    std::vector<int> robot_collisions = findHumanRobotContact(human_capsules[i], robot_capsules);
    if (robot_collisions.size() > 0) {
      if (robot_collision_map.find(i) == robot_collision_map.end()) {
        robot_collision_map[i] = std::unordered_set<int>();
      }
      robot_collision_map[i].insert(robot_collisions.begin(), robot_collisions.end());
    }
    // Maybe:
    // if (robot_collision_map.find(i) == robot_collision_map.end()) {
    //   continue;
    // }
    // Collisions with static environment elements
    std::vector<int> environment_collisions = findHumanEnvironmentContacts(human_capsules[i], environment_elements);
    if (environment_collisions.size() > 0) {
      if (environment_collision_map.find(i) == environment_collision_map.end()) {
        environment_collision_map[i] = std::unordered_set<int>();
      }
      environment_collision_map[i].insert(environment_collisions.begin(), environment_collisions.end());
    }
  }
}

void combineContactMaps(const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<double>& human_radii,
      const std::unordered_map<int, std::set<int>>& unclampable_body_part_map,
      std::unordered_map<int, std::unordered_set<int>>& robot_collision_map,
      std::unordered_map<int, std::unordered_set<int>>& environment_collision_map,
      std::vector<double>& combined_human_radii) {
  // Find human capsules in contact and merge them together
  std::vector<std::unordered_set<int>> human_contact_graphs = buildHumanContactGraphs(human_capsules, unclampable_body_part_map);
  // Combine the radii, robot collisions, and environment collisions of the human capsules in contact
  std::unordered_map<int, std::unordered_set<int>> combined_robot_collision_map;
  std::unordered_map<int, std::unordered_set<int>> combined_environment_collision_map;
  combined_human_radii.resize(human_contact_graphs.size());
  for (int i = 0; i < human_contact_graphs.size(); i++) {
    combined_human_radii[i] = 0;
    for (const auto& idx : human_contact_graphs[i]) {
      combined_human_radii[i] += human_radii[idx];
      combined_robot_collision_map[i].insert(
        robot_collision_map[idx].begin(),
        robot_collision_map[idx].end());
      combined_environment_collision_map[i].insert(
        environment_collision_map[idx].begin(),
        environment_collision_map[idx].end());
    }
  }
  robot_collision_map = combined_robot_collision_map;
  environment_collision_map = combined_environment_collision_map;
}

void buildHumanContactGraph(
      int current,
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::unordered_map<int, std::set<int>>& unclampable_body_part_map,
      std::unordered_set<int>& visited_body_parts,
      std::unordered_set<int>& human_contact_graph) {
  // Iterate over all remaining unvisited body parts while checking if the unvisited body part is still in the set.
  for (int i = 0; i < human_capsules.size(); i++) {
    if (i == current || human_contact_graph.find(i) != human_contact_graph.end()) {
      continue;
    }
    // A new body should only be added to the graph if it is clampable with all existing bodies in the graph.
    bool unclampable = false;
    for (const auto& body_in_graph : human_contact_graph) {
      if (linkPairUnclampable(body_in_graph, i, unclampable_body_part_map)) {
        unclampable = true;
        break;
      }
    }
    if (unclampable) {
      continue;
    }
    // Check if the current body part is in contact with the unvisited body part
    if (capsuleCollisionCheck(human_capsules[current], human_capsules[i])) {
      human_contact_graph.insert(i);
      visited_body_parts.insert(i);
      // Recursively build the contact graph
      buildHumanContactGraph(i, human_capsules, unclampable_body_part_map, visited_body_parts, human_contact_graph);
    }
  }
}

std::vector<std::unordered_set<int>> buildHumanContactGraphs(
  const std::vector<reach_lib::Capsule>& human_capsules,
  const std::unordered_map<int, std::set<int>>& unclampable_body_part_map
) {
  std::vector<std::unordered_set<int>> human_contact_graphs;
  std::unordered_set<int> visited_body_parts;
  for (int i = 0; i < human_capsules.size(); i++) {
    if (visited_body_parts.find(i) != visited_body_parts.end()) {
      continue;
    }
    std::unordered_set<int> human_contact_graph;
    human_contact_graph.insert(i);
    visited_body_parts.insert(i);
    buildHumanContactGraph(
      i,
      human_capsules,
      unclampable_body_part_map,
      visited_body_parts,
      human_contact_graph);
    human_contact_graphs.push_back(human_contact_graph);
  }
  return human_contact_graphs;
}

bool selfConstrainedCollisionCheck(const std::vector<int>& robot_collisions,
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
      if (linkPairUnclampable(link1, link2, unclampable_enclosures_map)) {
        continue;
      }
      // Check if the two links can cause a self-constrained collision
      reach_lib::Capsule expanded_robot_capsule = createExpandedCapsule(robot_capsules[link1], d_human);
      if (capsuleCollisionCheck(expanded_robot_capsule, robot_capsules[link2])) {
        // Self-constrained collision detected
        return true;
      }
    }
  }
  // No collision was possible
  return false;
}

bool calculateNormalVector(const reach_lib::Capsule& robot_capsule,
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
  if (normals.size() > 3) {
    throw std::length_error("The number of normal vectors is greater than 3.");
  }
  // Calculate the final normal vector of the environment element
  normal << 0.0, 0.0, 0.0;
  for (const Eigen::Vector3d& n : normals) {
    normal += n;
  }
  normal.normalize();
  return true;
}

bool capsuleMovingTowardsElement(const RobotReach::CapsuleVelocity velocity_capsule,
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

bool capsuleTrajectoryMovingTowardsElement(
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_start,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_end,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      int capsule_index,
      const Eigen::Vector3d& normal,
      double velocity_error) {
  // For every velocity of the link, check if the link is moving towards the environment element normal vector
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it = robot_capsule_velocities_start;
  while (robot_capsule_velocities_it != robot_capsule_velocities_end) {
    if (capsuleMovingTowardsElement((*robot_capsule_velocities_it)[capsule_index], normal, robot_capsules[capsule_index].r_, velocity_error)) {
      return true;
    }
    robot_capsule_velocities_it++;
  }
  return false;
}

bool environmentallyConstrainedCollisionCheck(const std::vector<int>& robot_collisions,
      const std::vector<reach_lib::Capsule>& robot_capsules,
      const std::vector<int>& environment_collisions,
      const std::vector<reach_lib::AABB>& environment_elements,
      double d_human,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_start,
      const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_end,
      std::vector<double> velocity_errors) {
  if (velocity_errors.size() != robot_capsules.size()) {
    throw std::length_error("The number of velocity errors does not match the number of robot capsules.");
  }
  // Check distance between link and environment
  // by expanding the link capsule by the human body diameter
  // and checking for intersection with the environment element.
  for (const int& environment_collision : environment_collisions) {
    for (const int& link_index : robot_collisions) {
      // spdlog::info("Checking link {} against environment element {} with max diameter {}", link_index, environment_collision, d_human);
      reach_lib::Capsule expanded_robot_capsule = createExpandedCapsule(robot_capsules[link_index], d_human);
      if (!reach_lib::intersections::capsule_aabb_intersection(expanded_robot_capsule,
          environment_elements[environment_collision])) {
        continue;
      }
      // spdlog::info("Collision is possible between link {} and environment element {}", link_index, environment_collision);
      // Check if the link is moving towards the environment element
      // Find normal vector on the environment element pointing towards the link
      Eigen::Vector3d normal;
      if (!calculateNormalVector(robot_capsules[link_index], environment_elements[environment_collision], normal)) {
        // If the calculation of the normal vector fails, the velocity criterion fails.
        // spdlog::info("Could not calculate normal vector for link {} and environment element {}", link_index, environment_collision);
        return true;
      }
      if (capsuleTrajectoryMovingTowardsElement(robot_capsule_velocities_start, robot_capsule_velocities_end, robot_capsules, link_index, normal, velocity_errors[link_index])) {
        // spdlog::info("Link {} moving towards environment element {}", link_index, environment_collision);
        return true;
      }
      /* 
        spdlog::info("Collision between link {} and environment element {} was disregarded: normal [{}, {}, {}], velocity error {}.", 
        link_index,
        environment_collision,
        normal[0], normal[1], normal[2],
        velocity_error);*/
    }
  }
  return false;
}

void separateConstrainedCollisions(
  const std::vector<reach_lib::Capsule>& robot_capsules, 
  const std::vector<reach_lib::Capsule>& human_capsules,
  const std::vector<reach_lib::AABB>& environment_elements,
  const std::vector<double>& human_radii,
  const std::unordered_map<int, std::set<int>>& unclampable_body_part_map,
  const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end,
  const std::vector<double>& velocity_errors,
  std::vector<std::pair<int, int>>& unconstrained_collisions,
  std::vector<std::pair<int, int>>& constrained_collisions
) {
  // Build a map that maps the robot capsule/environment indices in collision with the human capsules
  if (robot_capsules.size() != robot_capsule_velocities_it->size()) {
    throw std::length_error("The number of robot capsules does not match the number of robot capsule velocities.");
  }
  std::unordered_map<int, std::unordered_set<int>> robot_collision_map;
  std::unordered_map<int, std::unordered_set<int>> environment_collision_map;
  buildContactMaps(robot_capsules, human_capsules, environment_elements, robot_collision_map, environment_collision_map);
  // Check for self-constrained collisions and environmentally-constrained collisions
  for (const auto& robot_collisions : robot_collision_map) {
    int human_index = robot_collisions.first;
    
    // Build contact maps for this human capsule
    std::unordered_set<int> visited_body_parts;
    std::unordered_set<int> human_contact_graph;
    human_contact_graph.insert(human_index);
    visited_body_parts.insert(human_index);
    buildHumanContactGraph(
      human_index,
      human_capsules,
      unclampable_body_part_map,
      visited_body_parts,
      human_contact_graph
    );
    double combined_human_radius = 0;
    std::unordered_set<int> combined_robot_collisions_set;
    std::unordered_set<int> combined_environment_collisions_set;
    for (const auto& idx : human_contact_graph) {
      combined_human_radius += human_radii[idx];
      if (robot_collision_map.find(idx) != robot_collision_map.end()) {
        combined_robot_collisions_set.insert(robot_collision_map.at(idx).begin(), robot_collision_map.at(idx).end());
      }
      if (environment_collision_map.find(idx) != environment_collision_map.end()) {
        combined_environment_collisions_set.insert(environment_collision_map.at(idx).begin(), environment_collision_map.at(idx).end());
      }
    }
    std::vector<int> combined_robot_collisions_vec(combined_robot_collisions_set.begin(), combined_robot_collisions_set.end());
    std::vector<int> combined_environment_collisions_vec(combined_environment_collisions_set.begin(), combined_environment_collisions_set.end());

    if (selfConstrainedCollisionCheck(combined_robot_collisions_vec, unclampable_enclosures_map, robot_capsules, combined_human_radius)) {
      for (const int& link_index : combined_robot_collisions_vec) {
        constrained_collisions.push_back(std::make_pair(human_index, link_index));
      }
      continue;  // This human element can be clamped in the robot itself.
    }

    // Environmentally-constrained collision check
    if (environmentallyConstrainedCollisionCheck(combined_robot_collisions_vec, robot_capsules,
        combined_environment_collisions_vec, environment_elements, combined_human_radius,
        robot_capsule_velocities_it, robot_capsule_velocities_end, velocity_errors)) {
      // This human element can be clamped.
      for (const int& link_index : combined_robot_collisions_vec) {
        constrained_collisions.push_back(std::make_pair(human_index, link_index));
      }
      continue;  // This human element can be clamped between the robot and the environment.
    }

    // Only unconstrained collisions are left
    std::vector<int> robot_collisions_vec(robot_collisions.second.begin(), robot_collisions.second.end());
    for (const int& link_index : robot_collisions_vec) {
      unconstrained_collisions.push_back(std::make_pair(human_index, link_index));
    }
  }
}

bool clampingPossible(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules,
      const std::vector<reach_lib::AABB>& environment_elements,
      const std::vector<double>& human_radii,
      const std::unordered_map<int, std::set<int>>& unclampable_body_part_map,
      const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
      std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end,
      std::vector<double> velocity_errors) {
  // Build a map that maps the robot capsule/environment indices in collision with the human capsules
  // We choose this indexing to be in line with the paper.
  if (robot_capsules.size() != robot_capsule_velocities_it->size()) {
    throw std::length_error("The number of robot capsules does not match the number of robot capsule velocities.");
  }
  std::unordered_map<int, std::unordered_set<int>> robot_collision_map;
  std::unordered_map<int, std::unordered_set<int>> environment_collision_map;
  buildContactMaps(robot_capsules, human_capsules, environment_elements, robot_collision_map, environment_collision_map);
  std::vector<double> combined_human_radii;
  combineContactMaps(human_capsules, human_radii, unclampable_body_part_map, robot_collision_map, environment_collision_map, combined_human_radii);
  // Check for self-constrained collisions and environmentally-constrained collisions
  for (const auto& robot_collisions : robot_collision_map) {
    int human_index = robot_collisions.first;
    double d_human = 2 * combined_human_radii[human_index];
    // Self-constrained collision check
    std::vector<int> robot_collisions_vec(robot_collisions.second.begin(), robot_collisions.second.end());
    if (selfConstrainedCollisionCheck(robot_collisions_vec, unclampable_enclosures_map, robot_capsules, d_human)) {
      return true;
    }
    // Environmentally-constrained collision check
    if (environment_collision_map.find(human_index) == environment_collision_map.end()) {
      // spdlog::info("Human element {} collides with robot but not with environment.", human_index);
      continue;
    }
    std::vector<int> environment_collisions_vec(environment_collision_map.at(human_index).begin(), environment_collision_map.at(human_index).end());
    if (environmentallyConstrainedCollisionCheck(robot_collisions_vec, robot_capsules,
        environment_collisions_vec, environment_elements, d_human,
        robot_capsule_velocities_it, robot_capsule_velocities_end, velocity_errors)) {
      return true;
    }
    // spdlog::info("Human element {} collides with robot and environment but is safe.", human_index);
  }
  return false;
}

std::vector<double> calculateRobotLinkEnergies(
  const std::vector<double>& robot_link_velocities,
  const std::vector<double>& robot_link_reflected_masses
) {
  std::vector<double> robot_link_energies;
  for (int i = 0; i < robot_link_velocities.size(); i++) {
    robot_link_energies.push_back(0.5 * robot_link_reflected_masses[i] * robot_link_velocities[i] * robot_link_velocities[i]);
  }
  return robot_link_energies;
}

bool checkContactEnergySafety(
  const std::map<int, std::vector<int>>& human_robot_contacts,
  const std::vector<double>& robot_link_energies,
  const std::vector<double>& maximal_contact_energies
) {
  for (const auto& contact : human_robot_contacts) {
    int human_capsule_index = contact.first;
    for (const auto& robot_link_index : contact.second) {
      if (robot_link_energies[robot_link_index] > maximal_contact_energies[human_capsule_index]) {
        // spdlog::warn("Robot link {} in contact with human body {} and robot energy of {} exceeded max allowed energy of {}.", robot_link_index, human_capsule_index, robot_link_energies[robot_link_index], maximal_contact_energies[human_capsule_index]);
        return false;
      }
    }
  }
  return true;
}

bool checkContactEnergySafetyIndividualLinks(
  const std::vector<std::pair<int, int>>& collisions,
  const std::vector<double>& robot_energies,
  const std::vector<std::vector<std::vector<double>>>& max_contact_energies,
  int human_motion_model_id
) {
  for (const auto& collision : collisions) {
    int human_capsule_index = collision.first;
    int robot_link_index = collision.second;
    if (robot_energies[robot_link_index] > max_contact_energies[robot_link_index][human_motion_model_id][human_capsule_index]) {
      return false;
    }
  }
  return true;
}

bool checkVelocitySafety(
  const std::map<int, std::vector<int>>& human_robot_contacts,
  const std::vector<double>& robot_link_velocities,
  const std::vector<double>& maximal_contact_velocities
) {
  for (const auto& contact : human_robot_contacts) {
    int human_capsule_index = contact.first;
    for (const auto& robot_link_index : contact.second) {
      if (robot_link_velocities[robot_link_index] > maximal_contact_velocities[human_capsule_index]) {
        return false;
      }
    }
  }
  return true;
}

std::vector<std::vector<double>> calculateMaxRobotLinkVelocitiesPerTimeInterval(
  const std::vector<Motion>& robot_motions,
  const std::vector<double>& velocity_error
) {
  if (robot_motions.size() == 0) {
    spdlog::error("No robot motions provided in calculateMaxRobotLinkVelocitiesPerTimeInterval.");
    return std::vector<std::vector<double>>();
  }
  int n_links = robot_motions[0].getMaximumCartesianVelocities().size();
  if (velocity_error.size() != n_links) {
    throw std::length_error("Velocity error has to have the same size as the number of robot links.");
  }
  std::vector<std::vector<double>> max_velocities;
  for (int i = 1; i < robot_motions.size(); i++) {
    std::vector<double> max_velocities_i;
    for (int j = 0; j < n_links; j++) {
      double prev_vel = robot_motions[i-1].getMaximumCartesianVelocities()[j];
      double vel = robot_motions[i].getMaximumCartesianVelocities()[j];
      if (prev_vel < 0 || vel < 0) {
        throw std::invalid_argument("Maximum Cartesian velocity of robot motion cannot be negative.");
      }
      double max_v = std::max(prev_vel, vel);
      max_velocities_i.push_back(max_v + velocity_error[j]);
    }
    max_velocities.push_back(max_velocities_i);
  }
  return max_velocities;
}

std::vector<std::vector<double>> calculateMaxRobotEnergiesFromReflectedMasses(
  const std::vector<std::vector<double>>& robot_link_velocities,
  const std::vector<std::vector<double>>& robot_link_reflected_masses
) {
  if (robot_link_velocities.size() != robot_link_reflected_masses.size()) {
    throw std::length_error("Robot link velocities and reflected masses have to have the same size.");
  }
  if (robot_link_velocities.size() == 0) {
    spdlog::error("Empty time intervals provided in verification_utils::calculateMaxRobotEnergiesFromReflectedMasses().");
    return std::vector<std::vector<double>>();
  }
  if (robot_link_velocities[0].size() != robot_link_reflected_masses[0].size()) {
    throw std::length_error("Number of links in Robot link velocities and reflected masses have to be the same.");
  }
  int n_time_intervals = robot_link_velocities.size();
  std::vector<std::vector<double>> robot_link_energies;
  for (int i = 0; i < n_time_intervals; i++) {
    robot_link_energies.push_back(calculateRobotLinkEnergies(robot_link_velocities[i], robot_link_reflected_masses[i]));
  }
  return robot_link_energies;
}

std::vector<std::vector<double>> calculateMaxRobotEnergiesFromInertiaMatrices(
  const std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>>& robot_inertia_matrices,
  std::vector<std::vector<double>> dq
) {
  if (robot_inertia_matrices.size() != dq.size()) {
    throw std::length_error("Robot inertia matrices and joint velocities have to have the same size.");
  }
  if (robot_inertia_matrices.size() == 0) {
    spdlog::error("Empty time intervals provided in verification_utils::calculateMaxRobotEnergiesFromInertiaMatrices().");
    return std::vector<std::vector<double>>();
  }
  if (robot_inertia_matrices[0].size() != dq[0].size()) {
    throw std::length_error("Number of links in Robot inertia matrices and joint velocities have to be the same.");
  }
  if (robot_inertia_matrices[0][0].rows() != dq[0].size() || robot_inertia_matrices[0][0].cols() != dq[0].size()) {
    throw std::length_error("Number of joints in Robot inertia matrices and joint velocities have to be the same.");
  }
  int n_time_intervals = robot_inertia_matrices.size();
  std::vector<std::vector<double>> robot_link_energies;
  for (int i = 0; i < n_time_intervals; i++) {
    Eigen::VectorXd dq_i = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dq[i].data(), dq[i].size());
    std::vector<double> robot_link_energies_i;
    for (int j = 0; j < dq[i].size(); j++) {
      robot_link_energies_i.push_back((0.5 * dq_i.transpose() * robot_inertia_matrices[i][j] * dq_i).array()[0]);
    }
    robot_link_energies.push_back(robot_link_energies_i);
  }
  return robot_link_energies;
}

}  // namespace safety_shield