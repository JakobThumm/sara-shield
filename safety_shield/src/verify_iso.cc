#include "safety_shield/verify_iso.h"

namespace safety_shield {

bool VerifyISO::verifyHumanReach(const std::vector<reach_lib::Capsule>& robot_capsules,
                                   std::vector<std::vector<reach_lib::Capsule>> human_capsules) {
  for (const auto& capsule_list : human_capsules) {
    // If no collision occured, we are safe and don't have to check the rest.
    if (!robotHumanCollision(robot_capsules, capsule_list)) {
      return true;
    }
  }
  return false;
}

/// checks safety for each time interval
bool VerifyISO::verifyHumanReachTimeIntervals(
  const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
  const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
  int& collision_index
) {
  assert(robot_reachable_sets.size() == human_reachable_sets.size());
  for (int i = 0; i < robot_reachable_sets.size(); i++) {
    if (!verifyHumanReach(robot_reachable_sets[i], human_reachable_sets[i])) {
      // returns false if at least in one time step, there is a collision
      collision_index = i;
      return false;
    }
  }
  // returns true if it was safe in all time intervals
  collision_index = -1;
  return true;
}

bool VerifyISO::verifyHumanReachEnergyReflectedMasses(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
  const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
  const std::vector<std::vector<double>>& robot_link_velocities,
  const std::vector<std::vector<double>>& robot_link_reflected_masses,
  const std::vector<std::vector<double>>& maximal_contact_energies,
  int& collision_index) {
  std::vector<std::vector<double>> robot_link_energies = calculateMaxRobotEnergiesFromReflectedMasses(robot_link_velocities, robot_link_reflected_masses);
  return verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index);
}

bool VerifyISO::verifyHumanReachEnergyInertiaMatrices(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
  const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
  const std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>>& robot_inertia_matrices,
  const std::vector<Motion>& motions,
  const std::vector<std::vector<double>>& maximal_contact_energies,
  int& collision_index) {
  std::vector<std::vector<double>> dq;
  for (const auto& motion : motions) {
    dq.push_back(motion.getVelocity());
  }
  std::vector<std::vector<double>> robot_link_energies = calculateMaxRobotEnergiesFromInertiaMatrices(robot_inertia_matrices, dq);
  return verifyHumanReachEnergy(robot_reachable_sets, human_reachable_sets, robot_link_energies, maximal_contact_energies, collision_index);
}

bool VerifyISO::verifyHumanReachEnergy(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
  const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
  const std::vector<std::vector<double>>& robot_link_energies,
  const std::vector<std::vector<double>>& maximal_contact_energies,
  int& collision_index) {
  assert(robot_reachable_sets.size() == human_reachable_sets.size());  // same number of time intervals
  assert(robot_reachable_sets.size() == robot_link_energies.size());  // same number of time intervals
  assert(robot_reachable_sets[0].size() == robot_link_energies[0].size());  // same number of robot links
  assert(human_reachable_sets[0].size() == maximal_contact_energies.size());  // same number of human models
  assert(human_reachable_sets[0][0].size() == maximal_contact_energies[0].size());  // same number of human bodies
  int n_time_intervals = robot_reachable_sets.size();
  int n_robot_links = robot_reachable_sets[0].size();
  int n_human_models = human_reachable_sets[0].size();
  int n_human_bodies = human_reachable_sets[0][0].size();
  for (int i = 0; i < n_time_intervals; i++) {
    for (int m = 0; m < n_human_models; m++) {
      std::map<int, std::vector<int>> human_robot_contacts = findAllHumanRobotContacts(human_reachable_sets[i][m], robot_reachable_sets[i]);
      bool is_safe = checkContactEnergySafety(human_robot_contacts, robot_link_energies[i], maximal_contact_energies[m]);
      if (!is_safe) {
        collision_index = i;
        return false;
      }
    }
  }
  // returns true if it was safe in all time intervals
  collision_index = -1;
  return true;
}

bool VerifyISO::verifyHumanReachVelocity(const std::vector<std::vector<reach_lib::Capsule>>& robot_reachable_sets,
  const std::vector<std::vector<std::vector<reach_lib::Capsule>>>& human_reachable_sets,
  const std::vector<std::vector<double>>& robot_link_velocities,
  const std::vector<std::vector<double>>& maximal_contact_velocities,
  int& collision_index) {
  assert(robot_reachable_sets.size() == human_reachable_sets.size());  // same number of time intervals
  assert(robot_reachable_sets.size() == robot_link_velocities.size());  // same number of time intervals
  assert(robot_reachable_sets[0].size() == robot_link_velocities[0].size());  // same number of robot links
  assert(human_reachable_sets[0].size() == maximal_contact_velocities.size());  // same number of human models
  assert(human_reachable_sets[0][0].size() == maximal_contact_velocities[0].size());  // same number of human bodies
  int n_time_intervals = robot_reachable_sets.size();
  int n_robot_links = robot_reachable_sets[0].size();
  int n_human_models = human_reachable_sets[0].size();
  int n_human_bodies = human_reachable_sets[0][0].size();
  for (int i = 0; i < n_time_intervals; i++) {
    for (int m = 0; m < n_human_models; m++) {
      std::map<int, std::vector<int>> human_robot_contacts = findAllHumanRobotContacts(human_reachable_sets[i][m], robot_reachable_sets[i]);
      bool is_safe = checkVelocitySafety(human_robot_contacts, robot_link_velocities[i], maximal_contact_velocities[m]);
      if (!is_safe) {
        collision_index = i;
        return false;
      }
    }
  }
  // returns true if it was safe in all time intervals
  collision_index = -1;
  return true;
}

}  // namespace safety_shield