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

std::vector<reach_lib::Capsule> VerifyISO::PFL_robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules,
                                        const std::vector<reach_lib::Capsule>& human_capsules, std::vector<reach_lib::Capsule>& collision_capsules) {

  // Check position capsules
  for (auto &human_capsule: human_capsules) {
    for (auto &robot_capsule: robot_capsules) {
      // If there is a collision, add to list
      if (capsuleCollisionCheck(robot_capsule, human_capsule)) {
        collision_capsules.push_back(human_capsule);
      }
    }
  }
  return collision_capsules;
}

// TODO: Kollisions Kapseln müssen menschliche Kapseln sein, wie unterscheide ich zwischen den 3 Menschenmodellen?
std::vector<reach_lib::Capsule> VerifyISO::PFL_verify_human_reach(const std::vector<reach_lib::Capsule>& robot_capsules,
                                                                  const std::vector<std::vector<reach_lib::Capsule>>& human_capsules) {
  try {
    std::vector<reach_lib::Capsule> collision_capsules;
    for (const auto &capsule_list: human_capsules) {
      PFL_robotHumanCollision(robot_capsules, capsule_list, collision_capsules);
    }
    return collision_capsules;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::PFL_verify_human_reach: {}", exc.what());
  }
}


}  // namespace safety_shield