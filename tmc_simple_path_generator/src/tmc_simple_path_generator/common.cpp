/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief Common functions

#include "common.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <boost/range/adaptors.hpp>
#include <console_bridge/console.h>

#include <angles/angles.h>

#include <tmc_robot_local_planner_utils/converter.hpp>

using Eigen::VectorXd;

namespace {

template <typename T, typename U>
bool Extract(const tmc_manipulation_types::NameSeq& joint_names,
             const tmc_manipulation_types::NameSeq& target_names,
             const T& vector_in,
             U& vector_out) {
  vector_out.resize(target_names.size());
  for (const auto& target_name : target_names | boost::adaptors::indexed()) {
    auto index = tmc_manipulation_types::GetJointIndex(joint_names, target_name.value());
    if (index < vector_in.size()) {
      vector_out[target_name.index()] = vector_in[index];
    } else {
      CONSOLE_BRIDGE_logError("Index out of range.");
      return false;
    }
  }
  return true;
}

template <typename T, typename U>
void Overwrite(const tmc_manipulation_types::NameSeq& joint_names,
               const tmc_manipulation_types::NameSeq& target_names,
               const T& vector_in,
               U& vector_out) {
  for (const auto& target_name : target_names | boost::adaptors::indexed()) {
    auto index = tmc_manipulation_types::GetJointIndex(joint_names, target_name.value());
    if (index < vector_out.size()) {
      vector_out[index] = vector_in[target_name.index()];
    }
  }
}

}  // namespace

namespace tmc_simple_path_generator {

// Overwrite robot_state_in_out
bool OverwriteRobotState(
    tmc_manipulation_types::RobotState& robot_state_in_out,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    const VectorXd& joint_positions,
    const tmc_manipulation_types::PoseSeq& base_poses) {
  if ((joint_names.size() != joint_positions.size()) ||
      (base_names.size() != base_poses.size())) {
    CONSOLE_BRIDGE_logError("Size mismatch.");
    return false;
  }
  Overwrite(robot_state_in_out.joint_state.name, joint_names,
            joint_positions, robot_state_in_out.joint_state.position);
  Overwrite(robot_state_in_out.multi_dof_joint_state.names, base_names,
            base_poses, robot_state_in_out.multi_dof_joint_state.poses);
  return true;
}

// Extract information from robot_state
bool ExtractFromRobotState(
    const tmc_manipulation_types::RobotState& robot_state,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    VectorXd& joint_positions_out,
    tmc_manipulation_types::PoseSeq& base_poses_out) {
  if (!Extract(robot_state.joint_state.name, joint_names,
               robot_state.joint_state.position, joint_positions_out)) {
    return false;
  }
  if (!Extract(robot_state.multi_dof_joint_state.names, base_names,
               robot_state.multi_dof_joint_state.poses, base_poses_out)) {
    return false;
  }
  return true;
}

// Add uniform random noise with width delta to robot_state
bool AddRandomState(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    const std::vector<double>& joint_limit_upper,
    const std::vector<double>& joint_limit_lower,
    double delta,
    std::mt19937& generator,
    tmc_manipulation_types::RobotState& robot_state_out) {
  if (joint_names.size() != joint_limit_upper.size() ||
      joint_names.size() != joint_limit_lower.size()) {
    return false;
  }

  VectorXd joint_positions_in;
  joint_positions_in.resize(joint_names.size());
  tmc_manipulation_types::PoseSeq base_poses_in;
  base_poses_in.resize(base_names.size());
  if (!ExtractFromRobotState(robot_state_in, joint_names, base_names, joint_positions_in, base_poses_in)) {
    return false;
  }

  std::uniform_real_distribution<> dist(-1.0, 1.0);
  VectorXd joint_positions_out = joint_positions_in;
  for (uint32_t i = 0; i < joint_positions_out.size(); ++i) {
    joint_positions_out[i] += delta * dist(generator);
    joint_positions_out[i] = std::max(std::min(joint_positions_out[i], joint_limit_upper[i]), joint_limit_lower[i]);
  }
  tmc_manipulation_types::PoseSeq base_poses_out;
  for (uint32_t i = 0; i < base_names.size(); ++i) {
    Eigen::Vector3d pose_2d(dist(generator), dist(generator), dist(generator));
    base_poses_out.push_back(base_poses_in[i] * tmc_robot_local_planner_utils::GetTransform(delta * pose_2d));
  }
  robot_state_out = robot_state_in;
  return OverwriteRobotState(robot_state_out, joint_names, base_names,
                             joint_positions_out, base_poses_out);
}

// Replace robot_state with a random joint posture, uniform sampling within upper and lower limits
bool ReplaceWithUniformRandomJointPositions(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& joint_limit_upper,
    const std::vector<double>& joint_limit_lower,
    std::mt19937& generator,
    tmc_manipulation_types::RobotState& robot_state_out) {
  if (joint_names.size() != joint_limit_upper.size() ||
      joint_names.size() != joint_limit_lower.size()) {
    return false;
  }

  std::uniform_real_distribution<> dist(-1.0, 1.0);
  VectorXd joint_positions(robot_state_in.joint_state.position.size());
  for (uint32_t i = 0; i < joint_positions.size(); ++i) {
    const auto joint_name = robot_state_in.joint_state.name[i];
    auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
    if (it == joint_names.end()) {
      return false;
    }
    auto j = std::distance(joint_names.begin(), it);
    auto range = joint_limit_upper[j] - joint_limit_lower[j];
    auto sum = joint_limit_upper[j] + joint_limit_lower[j];
    joint_positions[i] = (sum + range * dist(generator)) / 2.0;
  }
  robot_state_out = robot_state_in;
  robot_state_out.joint_state.position = joint_positions;
  return true;
}

// Add a random offset to the cart position and posture, uniform sampling
bool AddUniformRandomBasePoseOffset(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const tmc_manipulation_types::NameSeq& base_names,
    double position_range,
    double rotation_range,
    std::mt19937& generator,
    tmc_manipulation_types::RobotState& robot_state_out) {
  VectorXd joint_positions;
  tmc_manipulation_types::PoseSeq base_poses;
  ExtractFromRobotState(robot_state_in, {}, base_names, joint_positions,
                        base_poses);

  std::uniform_real_distribution<> dist(-1.0, 1.0);
  Eigen::Vector3d pose_2d(dist(generator), dist(generator), dist(generator));
  pose_2d[0] *= position_range;
  pose_2d[1] *= position_range;
  pose_2d[2] *= rotation_range;
  auto offset_pose = tmc_robot_local_planner_utils::GetTransform(pose_2d);

  robot_state_out = robot_state_in;
  return OverwriteRobotState(robot_state_out, {}, base_names, {},
                             {base_poses[0] * offset_pose});
}

// Calculation of weighted distance
double CalculateLength(const Eigen::VectorXd& prev,
                       const Eigen::VectorXd& next,
                       const Eigen::VectorXd& weights) {
  return ((next - prev).array().abs() * weights.array()).maxCoeff();
}

// Calculation of weighted joint distance
double CalculateJointLength(const tmc_manipulation_types::RobotState& prev,
                            const tmc_manipulation_types::RobotState& next,
                            const Eigen::VectorXd& weights) {
  return CalculateLength(prev.joint_state.position, next.joint_state.position, weights);
}

// Calculation of weighted cart distance
double CalculateBaseLength(const Eigen::Affine3d& prev,
                           const Eigen::Affine3d& next,
                           const Eigen::VectorXd& weights) {
  const auto prev_vec = tmc_robot_local_planner_utils::Get2DPose(prev);
  auto next_vec = tmc_robot_local_planner_utils::Get2DPose(next);
  next_vec[2] = prev_vec[2] + angles::shortest_angular_distance(prev_vec[2], next_vec[2]);
  return CalculateLength(prev_vec, next_vec, weights);
}

// Connect RobotState group and convert to trajectory
bool Convert(const tmc_manipulation_types::RobotState& initial_state,
             const std::vector<tmc_manipulation_types::RobotState>& way_points,
             const std::vector<std::string>& joint_names,
             const Eigen::VectorXd& joint_weights,
             const Eigen::VectorXd& base_weights,
             tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  if (!initial_state.joint_state.name.empty()) {
    trajectory_out.joint_trajectory.joint_names = initial_state.joint_state.name;
    trajectory_out.joint_trajectory.points.resize(1 + way_points.size());
    trajectory_out.joint_trajectory.points[0].positions = initial_state.joint_state.position;
    trajectory_out.joint_trajectory.points[0].velocities = initial_state.joint_state.velocity;
  }

  trajectory_out.multi_dof_joint_trajectory.joint_names = initial_state.multi_dof_joint_state.names;
  trajectory_out.multi_dof_joint_trajectory.points.resize(1 + way_points.size());
  trajectory_out.multi_dof_joint_trajectory.points[0].transforms = initial_state.multi_dof_joint_state.poses;
  trajectory_out.multi_dof_joint_trajectory.points[0].velocities = initial_state.multi_dof_joint_state.twist;

  Eigen::VectorXd extracted_joint_weights = Eigen::VectorXd::Ones(initial_state.joint_state.position.size());
  if (!Extract(joint_names, initial_state.joint_state.name, joint_weights, extracted_joint_weights)) {
    return false;
  }

  for (auto i = 0; i < way_points.size(); ++i) {
    double length = CalculateBaseLength(trajectory_out.multi_dof_joint_trajectory.points[i].transforms[0],
                                        way_points[i].multi_dof_joint_state.poses[0], base_weights);
    if (!trajectory_out.joint_trajectory.joint_names.empty()) {
      double joint_length = CalculateLength(trajectory_out.joint_trajectory.points[i].positions,
                                            way_points[i].joint_state.position, extracted_joint_weights);
      length = std::max(length, joint_length);
    }

    if (!trajectory_out.joint_trajectory.joint_names.empty()) {
      trajectory_out.joint_trajectory.points[i + 1].positions = way_points[i].joint_state.position;
      trajectory_out.joint_trajectory.points[i + 1].velocities = Eigen::VectorXd::Zero(
          way_points[i].joint_state.position.size());
      trajectory_out.joint_trajectory.points[i + 1].time_from_start =
          trajectory_out.joint_trajectory.points[i].time_from_start + length;
    }

    trajectory_out.multi_dof_joint_trajectory.points[i + 1].transforms = way_points[i].multi_dof_joint_state.poses;
    trajectory_out.multi_dof_joint_trajectory.points[i + 1].velocities.resize(
        way_points[i].multi_dof_joint_state.poses.size(), tmc_manipulation_types::Twist::Zero());
    trajectory_out.multi_dof_joint_trajectory.points[i + 1].time_from_start =
        trajectory_out.multi_dof_joint_trajectory.points[i].time_from_start + length;
  }
  return true;
}

}  // namespace tmc_simple_path_generator
