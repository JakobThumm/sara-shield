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
  const std::vector<double>& link_radii,
  const std::vector<double>& velocity_error
) {
  std::vector<std::vector<double>> max_velocities;
  assert(robot_motions.size() > 0);
  int n_links = robot_motions[0].getMaximumCartesianVelocities().size();
  assert(velocity_error.size() == n_links);
  assert(link_radii.size() == n_links);
  for (int i = 1; i < robot_motions.size(); i++) {
    std::vector<double> max_velocities_i;
    for (int j = 0; j < n_links; j++) {
      double delta_s = robot_motions[i].getS()-robot_motions[i-1].getS();
      double max_v = std::max(robot_motions[i-1].getMaximumCartesianVelocities()[j], robot_motions[i].getMaximumCartesianVelocities()[j]);
      max_velocities_i.push_back(max_v + velocity_error[j]);
    }
    max_velocities.push_back(max_velocities_i);
  }
  return max_velocities;
}
}  // namespace safety_shield