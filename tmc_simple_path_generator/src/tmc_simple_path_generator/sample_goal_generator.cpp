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
/// @brief Sample Goal Generator Class

#include <tmc_simple_path_generator/sample_goal_generator.hpp>

#include <string>
#include <vector>

#include <console_bridge/console.h>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include "common.hpp"

namespace {

// Size check
bool CheckRobotState(const tmc_manipulation_types::RobotState& robot_state) {
  if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size()) {
    return false;
  }
  if (robot_state.multi_dof_joint_state.names.size() != robot_state.multi_dof_joint_state.poses.size()) {
    return false;
  }
  return true;
}

std::optional<tmc_robot_kinematics_model::IKRequest> GenerateIKRequest(
    const Eigen::Affine3d& origin_to_end,
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_simple_path_generator::SamplingParameters::Ptr& params,
    const std::string& end_frame) {
  if (params->ik_joints.find(end_frame) == params->ik_joints.end()) {
    CONSOLE_BRIDGE_logWarn("IK joints for end effector %s is not found", end_frame.c_str());
    return std::nullopt;
  }
  tmc_robot_kinematics_model::IKRequest req(
      static_cast<tmc_manipulation_types::BaseMovementType>(params->base_movement_type));
  req.initial_angle.name = initial_state.joint_state.name;
  req.initial_angle.position = initial_state.joint_state.position;
  req.use_joints = params->ik_joints.at(end_frame);

  Eigen::VectorXd joint_weight(req.use_joints.size());
  for (size_t i = 0; i < req.use_joints.size(); ++i) {
    auto it = std::find(params->joint_names.begin(), params->joint_names.end(), req.use_joints[i]);
    if (it != params->joint_names.end()) {
      const auto distance = std::distance(params->joint_names.begin(), it);
      joint_weight[i] = params->joint_weights[distance];
    } else {
      joint_weight[i] = 1.0;
    }
  }
  if (req.linear_base_movements.empty() && req.rotational_base_movements.empty()) {
    req.weight = joint_weight;
  } else {
    req.weight.resize(req.use_joints.size() + params->base_weights.size());
    req.weight << joint_weight, params->base_weights;
  }

  req.origin_to_base = initial_state.multi_dof_joint_state.poses[0];
  req.target_frames.emplace_back(end_frame, origin_to_end);
  return req;
}

}  // namespace

namespace tmc_simple_path_generator {

// Constructor
SampleGoalGeneratorBase::SampleGoalGeneratorBase()
    : ik_solver_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IKSolver"),
      is_from_random_initial_state_(true) {}

// Initialization to fix the seed
void SampleGoalGeneratorBase::Initialize(const rclcpp::Node::SharedPtr& node, uint32_t random_seed) {
  engine_.seed(random_seed);

  ik_initial_range_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "ik_initial_range", 0.5);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback, ik_initial_range_, std::placeholders::_1)));

  InitializeImpl(node);
}

// Initialization to randomly generate the seed
void SampleGoalGeneratorBase::Initialize(const rclcpp::Node::SharedPtr& node) {
  Initialize(node, std::random_device()());
}

// Sample goal posture from joint constraints
bool SampleGoalGeneratorBase::SampleFromJointConstraints(
    const std::vector<tmc_robot_local_planner::IJointConstraint::Ptr>& joint_constraints,
    tmc_manipulation_types::RobotState& goal_state_out) {
  if (!CheckRobotState(goal_state_out)) {
    return false;
  }
  if (joint_constraints.size() == 0) {
    return false;
  }
  auto choosen_constraint = SampleConstraints(joint_constraints, engine_);
  const auto sampled_state = choosen_constraint->Sample();
  goal_state_out.joint_state = sampled_state.joint_state;
  if (!sampled_state.multi_dof_joint_state.poses.empty()) {
    goal_state_out.multi_dof_joint_state.poses[0] = sampled_state.multi_dof_joint_state.poses[0];
  }
  return true;
}

// Sample goal posture from posture constraints
bool SampleGoalGeneratorBase::SampleFromLinkConstraints(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& link_constraints,
    const SamplingParameters::Ptr& params,
    tmc_manipulation_types::RobotState& goal_state_out) {
  if (!CheckRobotState(initial_state)) {
    return false;
  }
  if (link_constraints.size() == 0) {
    return false;
  }

  std::vector<std::string> base_names = {};
  if (params->enable_base) {
    base_names = params->base_names;
  }

  tmc_manipulation_types::RobotState varianced_initial_state;
  if (is_from_random_initial_state_) {
    if (!AddRandomState(initial_state, params->joint_names, base_names, params->joint_limit_upper,
                        params->joint_limit_lower, ik_initial_range_->value(), engine_, varianced_initial_state)) {
      return false;
    }
  } else {
    varianced_initial_state = initial_state;
  }

  auto choosen_constraint = SampleConstraints(link_constraints, engine_);
  auto origin_to_ref = choosen_constraint->Sample();
  return SolveIK(origin_to_ref, varianced_initial_state, params,
                 choosen_constraint->GetLinkName(), goal_state_out);
}


// Implementation of initialization
void SampleGoalGenerator::InitializeImpl(const rclcpp::Node::SharedPtr& node) {
  const auto solver_type = tmc_utils::GetParameter<std::string>(
      node, "solver_type", "tmc_robot_kinematics_model/NumericIKSolver");
  ik_solver_ = ik_solver_loader_.createSharedInstance(solver_type);

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
    robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  ik_solver_->set_robot_description(robot_description);
}

// Find the robot_state that achieves the terminal posture origin_to_end
bool SampleGoalGenerator::SolveIK(const Eigen::Affine3d& origin_to_end,
                                  const tmc_manipulation_types::RobotState& initial_state,
                                  const SamplingParameters::Ptr& params,
                                  const std::string& end_frame,
                                  tmc_manipulation_types::RobotState& solution_out) {
  const auto req = GenerateIKRequest(origin_to_end, initial_state, params, end_frame);
  if (!req.has_value()) {
    return false;
  }

  std::vector<tmc_robot_kinematics_model::IKResponse> res;
  tmc_robot_kinematics_model::IKResult result;
  try {
    result = ik_solver_->Solve(req.value(), res);
  } catch (std::exception& e) {
    CONSOLE_BRIDGE_logWarn("%s", e.what());
    return false;
  }
  if (result == tmc_robot_kinematics_model::kSuccess && !res.empty()) {
    solution_out = initial_state;
    solution_out.joint_state = res[0].solution_angle;
    solution_out.multi_dof_joint_state.poses[0] = res[0].origin_to_base;
    CONSOLE_BRIDGE_logDebug("IK success");
    return true;
  } else {
    CONSOLE_BRIDGE_logDebug("IK Failed %d", result);
    return false;
  }
}

}  // namespace tmc_simple_path_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::SampleGoalGenerator,
                       tmc_simple_path_generator::SampleGoalGeneratorBase);
