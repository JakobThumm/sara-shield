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
  if (robot_motions.size() == 0) {
    spdlog::error("No robot motions provided in calculateMaxRobotLinkVelocitiesPerTimeInterval.");
    return std::vector<std::vector<double>>();
  }
  int n_links = robot_motions[0].getMaximumCartesianVelocities().size();
  if (velocity_error.size() != n_links) {
    throw std::length_error("Velocity error has to have the same size as the number of robot links.");
  }
  if (link_radii.size() != n_links) {
    throw std::length_error("Link radii have to have the same size as the number of robot links.");
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