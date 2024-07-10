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

std::vector<int> find_human_robot_contact(const reach_lib::Capsule& human_capsule,
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

std::map<int, std::vector<int>> find_all_human_robot_contacts(const std::vector<reach_lib::Capsule>& human_capsule,
      const std::vector<reach_lib::Capsule>& robot_capsules) {
  std::map<int, std::vector<int>> human_robot_contacts;
  for (int i = 0; i < human_capsule.size(); i++) {
    human_robot_contacts[i] = find_human_robot_contact(human_capsule[i], robot_capsules);
  }
  return human_robot_contacts;
}


}