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
/// @brief Parameters for trajectory generation
#include <tmc_simple_path_generator/sampling_parameters.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

#include <tmc_utils/parameters.hpp>

#include "common.hpp"

namespace {
template <typename T, typename U, typename V>
void GetMapToKeyValue(const T& map_in, U& keys, V& values) {
  for (auto it = map_in.begin(); it != map_in.end(); ++it) {
    keys.push_back(it->first);
    values.push_back(it->second);
  }
}
}  // namespace

namespace tmc_simple_path_generator {

SamplingParameters::SamplingParameters(const rclcpp::Node::SharedPtr& node) {
  this->joint_names = tmc_utils::GetParameter<std::vector<std::string>>(node, "joint_names", {});
  if (this->joint_names.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter joint_names isn't set.");
    exit(EXIT_FAILURE);
  }
  // Non-const because Map does not pass if const
  auto joint_weights_vec = tmc_utils::GetParameter<std::vector<double>>(node, "joint_weights", {});
  this->joint_weights.resize(joint_weights_vec.size());
  this->joint_weights = Eigen::Map<Eigen::VectorXd>(&joint_weights_vec[0], joint_weights_vec.size());

  this->base_names = tmc_utils::GetParameter<std::vector<std::string>>(node, "base_names", {});
  if (this->base_names.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter base_names isn't set.");
    exit(EXIT_FAILURE);
  }
  auto base_weights_vec = tmc_utils::GetParameter<std::vector<double>>(node, "base_weights", {});
  this->base_weights.resize(base_weights_vec.size());
  this->base_weights = Eigen::Map<Eigen::VectorXd>(&base_weights_vec[0], base_weights_vec.size());

  const auto urdf_str = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  auto urdf = urdf::Model();
  if (!urdf.initString(urdf_str)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to parse URDF contained in robot_description");
    exit(EXIT_FAILURE);
  }
  for (const auto& joint_name : this->joint_names) {
    if (urdf.getJoint(joint_name)) {
      auto urdf_joint_limits = urdf.getJoint(joint_name)->limits;
      this->joint_limit_upper.push_back(urdf_joint_limits->upper);
      this->joint_limit_lower.push_back(urdf_joint_limits->lower);
    } else {
      RCLCPP_FATAL(node->get_logger(), "No joint position limit information in robot_description");
      exit(EXIT_FAILURE);
    }
  }

  this->default_base_movement_type_ = tmc_utils::GetParameter(node, "base_movement_type", -1);
  if (default_base_movement_type_ < 0) {
    RCLCPP_FATAL(node->get_logger(), "Parameter base_movement_type isn't set");
    exit(EXIT_FAILURE);
  }
  UpdateBaseMovementType(true);

  const auto end_effectors = tmc_utils::GetParameter<std::vector<std::string>>(node, "end_effectors", {});
  if (end_effectors.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter end_effectors isn't set.");
    exit(EXIT_FAILURE);
  }
  for (const auto& end_effector : end_effectors) {
    const auto ik_joints_param_name = "ik_joints." + end_effector;
    const auto ik_joints = tmc_utils::GetParameter<std::vector<std::string>>(node, ik_joints_param_name, {});
    if (ik_joints.empty()) {
      RCLCPP_FATAL(node->get_logger(), "Parameter %s isn't set.", ik_joints_param_name.c_str());
      exit(EXIT_FAILURE);
    }
    this->ik_joints[end_effector] = ik_joints;
  }
}

void SamplingParameters::UpdateBaseMovementType(bool enable_base) {
  this->enable_base = enable_base;

  if (enable_base) {
    this->base_movement_type = this->default_base_movement_type_;
  } else {
    this->base_movement_type = tmc_manipulation_types::kNone;
  }
}

}  // namespace tmc_simple_path_generator

