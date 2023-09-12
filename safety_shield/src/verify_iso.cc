#include "safety_shield/verify_iso.h"

namespace safety_shield {

bool VerifyISO::robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules,
                                    const std::vector<reach_lib::Capsule>& human_capsules) {
  try {
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
  } catch (const std::exception& exc) {
    spdlog::error("Exception in VerifyISO::robotHumanCollision: {}", exc.what());
    return false;
  }
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

int VerifyISO::getIndexOfHead(const std::map<std::string, std::pair<int, double>>& body_to_index_and_velocity) {
  if(body_to_index_and_velocity.count("head") == 1) {
    return body_to_index_and_velocity.at("head").first;
  } else if(body_to_index_and_velocity.count("Head") == 1) {
    return body_to_index_and_velocity.at("Head").first;
  } else {
    spdlog::error("VerifyISO::getIndexOfHead: body_link_joints has no head value");
    return 0;
  }
}

// head_safe wenn bei mind. einem Model Kopf-Kapsel nicht mit dabei bei Kollisionsliste ist
bool VerifyISO::verify_human_reach_head(const std::vector<reach_lib::Capsule>& robot_capsules,
                             const std::vector<reach_lib::Capsule>& human_capsules_vel,
                             const std::vector<reach_lib::Capsule>& human_capsules_acc,
                             const std::map<std::string, reach_lib::jointPair>& body_link_joints,
                             const std::map<std::string, std::pair<int, double>>& body_to_index_and_velocity) {
  try {
    int index_head = getIndexOfHead(body_to_index_and_velocity);
    // only look at head capsule
    bool vel_model = !robotHumanCollision(robot_capsules, {human_capsules_vel.at(index_head)});
    bool acc_model = !robotHumanCollision(robot_capsules, {human_capsules_acc.at(index_head)});
    // safe if at least one model has no collision
    return vel_model || acc_model;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_human_reach_head: {}", exc.what());
    return false;
  }
}

// non_head_safe wenn bei mind. einem Model non-Kopf-Kapseln nicht mit dabei bei Kollisionsliste ist
bool VerifyISO::verify_human_reach_non_head(const std::vector<reach_lib::Capsule>& robot_capsules,
                                        std::vector<reach_lib::Capsule> human_capsules_vel,
                                        std::vector<reach_lib::Capsule> human_capsules_acc,
                                        const std::map<std::string, reach_lib::jointPair>& body_link_joints,
                                        const std::map<std::string, std::pair<int, double>>& body_to_index_and_velocity) {
  try {
    int index_head = getIndexOfHead(body_to_index_and_velocity);
    // look at all capsules except head
    human_capsules_vel.erase(human_capsules_vel.begin() + index_head);
    human_capsules_acc.erase(human_capsules_acc.begin() + index_head);
    // safe if at least one model has no collision
    bool vel_model = !robotHumanCollision(robot_capsules, human_capsules_vel);
    bool acc_model = !robotHumanCollision(robot_capsules, human_capsules_acc);
    return vel_model || acc_model;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_human_reach_non_head: {}", exc.what());
    return false;
  }
}

}  // namespace safety_shield