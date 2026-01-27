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
#ifndef TMC_SIMPLE_PATH_GENERATOR_SAMPLE_GOAL_GENERATOR_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_SAMPLE_GOAL_GENERATOR_HPP_

#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include <tmc_robot_local_planner/constraints.hpp>
#include <tmc_utils/parameters.hpp>

#include <tmc_simple_path_generator/sampling_parameters.hpp>

namespace tmc_simple_path_generator {

class SampleGoalGeneratorBase {
 public:
  using Ptr = std::shared_ptr<SampleGoalGeneratorBase>;

  /// @brief Constructor
  SampleGoalGeneratorBase();
  /// @brief Destructor
  virtual ~SampleGoalGeneratorBase() = default;

  /// @brief Initialization to fix the seed
  void Initialize(const rclcpp::Node::SharedPtr& node, uint32_t random_seed);
  /// @brief Initialization to randomly generate the seed
  void Initialize(const rclcpp::Node::SharedPtr& node);

  /// @brief Sample goal posture from joint constraints
  bool SampleFromJointConstraints(
      const std::vector<tmc_robot_local_planner::IJointConstraint::Ptr>& joint_constraints,
      tmc_manipulation_types::RobotState& goal_state_out);

  /// @brief Sample goal posture from posture constraints
  bool SampleFromLinkConstraints(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& link_constraints,
      const SamplingParameters::Ptr& params,
      tmc_manipulation_types::RobotState& goal_state_out);

  /// Set whether to add random numbers to the initial value of sampling from posture constraints
  void set_is_from_random_initial_state(bool value) { is_from_random_initial_state_ = value; }

  /// @brief Processing at termination
  virtual void Terminate() {
    is_from_random_initial_state_ = false;
  }

 protected:
  // IKSolver loader
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IKSolver> ik_solver_loader_;

  /// @brief Implementation of initialization for derived classes
  virtual void InitializeImpl(const rclcpp::Node::SharedPtr& node) = 0;

  /// @brief Find robot_state to achieve terminal posture origin_to_end
  virtual bool SolveIK(const Eigen::Affine3d& origin_to_end,
                       const tmc_manipulation_types::RobotState& initial_state,
                       const SamplingParameters::Ptr& params,
                       const std::string& end_frame,
                       tmc_manipulation_types::RobotState& solution_out) = 0;

 private:
  // Random number generator
  std::mt19937 engine_;
  // Handle for dynamic parameter setting
  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;
  // Width of random element of robot posture when solving IK [m]
  tmc_utils::DynamicParameter<double>::Ptr ik_initial_range_;

  // Whether to add random numbers to the initial value of sampling from posture constraints
  bool is_from_random_initial_state_;
};


class SampleGoalGenerator : public SampleGoalGeneratorBase {
 public:
  /// @brief Constructor
  SampleGoalGenerator() = default;
  /// @brief Destructor
  virtual ~SampleGoalGenerator() = default;

 protected:
  /// @brief Implementation of initialization
  void InitializeImpl(const rclcpp::Node::SharedPtr& node) override;

  /// @brief Find robot_state to achieve terminal posture origin_to_end
  bool SolveIK(const Eigen::Affine3d& origin_to_end,
               const tmc_manipulation_types::RobotState& initial_state,
               const SamplingParameters::Ptr& params,
               const std::string& end_frame,
               tmc_manipulation_types::RobotState& solution_out) override;

 private:
  // IKSolver instance
  std::shared_ptr<tmc_robot_kinematics_model::IKSolver> ik_solver_;
};

}  // namespace tmc_simple_path_generator

#endif  // TMC_SIMPLE_PATH_GENERATOR_SAMPLE_GOAL_GENERATOR_HPP

