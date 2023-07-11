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

// TODO: try second because mujoco_mocap has neck as proximal joint?
int VerifyISO::getIndexOfHead(const std::map<std::string, reach_lib::jointPair>& body_link_joints) {
  if(body_link_joints.count("head") == 1) {
    return body_link_joints.at("head").first;
  } else if(body_link_joints.count("Head") == 1) {
    spdlog::info("index first is: {}", body_link_joints.at("Head").first);
    spdlog::info("index second is: {}", body_link_joints.at("Head").second);
    return body_link_joints.at("Head").first;
  } else {
    spdlog::error("VerifyISO::getIndexOfHead: body_link_joints has no head value");
    return 0;
  }
}

// TODO: head_safe wenn bei mind. einem Model Kopf nicht mit dabei bei Kollisionsliste ist
bool VerifyISO::verify_human_reach_head(const std::vector<reach_lib::Capsule>& robot_capsules,
                             const std::vector<reach_lib::Capsule>& human_capsules_vel,
                             const std::vector<reach_lib::Capsule>& human_capsules_acc,
                             const std::map<std::string, reach_lib::jointPair>& body_link_joints) {
  try {
    int index_head = getIndexOfHead(body_link_joints);
    // TODO: human_capsules_vel or acc hat index_head nicht?
    spdlog::info("capsules_vel length is: {}", human_capsules_vel.size());
    spdlog::info("capsules_acc length is: {}", human_capsules_acc.size());
    bool vel_model = !robotHumanCollision(robot_capsules, {human_capsules_vel.at(index_head)});
    bool acc_model = !robotHumanCollision(robot_capsules, {human_capsules_acc.at(index_head)});
    return vel_model || acc_model;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_human_reach_head: {}", exc.what());
    return false;
  }
}

// TODO: non_head_safe wenn bei mind. einem Model non-Kopf nicht mit dabei bei Kollisionsliste ist
bool VerifyISO::verify_human_reach_non_head(const std::vector<reach_lib::Capsule>& robot_capsules,
                                        std::vector<reach_lib::Capsule> human_capsules_vel,
                                        std::vector<reach_lib::Capsule> human_capsules_acc,
                                        const std::map<std::string, reach_lib::jointPair>& body_link_joints) {
  try {
    int index_head = getIndexOfHead(body_link_joints);
    human_capsules_vel.erase(human_capsules_vel.cbegin() + index_head);
    human_capsules_acc.erase(human_capsules_acc.cbegin() + index_head);
    bool vel_model = !robotHumanCollision(robot_capsules, human_capsules_vel);
    bool acc_model = !robotHumanCollision(robot_capsules, human_capsules_acc);
    return vel_model || acc_model;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_human_reach_non_head: {}", exc.what());
    return false;
  }
}

}  // namespace safety_shield