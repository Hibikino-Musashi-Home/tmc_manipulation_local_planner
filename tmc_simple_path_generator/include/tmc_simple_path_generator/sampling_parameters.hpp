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
#ifndef TMC_SIMPLE_PATH_GENERATOR_SAMPLING_PARAMETERS_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_SAMPLING_PARAMETERS_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_simple_path_generator {

struct SamplingParameters {
  using Ptr = std::shared_ptr<SamplingParameters>;

  // Joint name
  tmc_manipulation_types::NameSeq joint_names;
  // Cart name
  tmc_manipulation_types::NameSeq base_names;
  // Joint weight
  Eigen::VectorXd joint_weights;
  // Cart weight
  Eigen::VectorXd base_weights;
  // Upper limit of joint
  std::vector<double> joint_limit_upper;
  // Lower limit of joint
  std::vector<double> joint_limit_lower;
  // Cart movement type
  int base_movement_type;
  // Whether to use cart movement or not
  bool enable_base;

  std::map<std::string, std::vector<std::string>> ik_joints;  // end_effector -> ik_joints

  SamplingParameters() {}
  explicit SamplingParameters(const rclcpp::Node::SharedPtr& node);

  void UpdateBaseMovementType(bool enable_base);
  void SetDefaultBaseMovementType(int movement_type) {
    default_base_movement_type_ = movement_type;
  }

 private:
  int default_base_movement_type_;
};

}  // namespace tmc_simple_path_generator

#endif  // TMC_SIMPLE_PATH_GENERATOR_SAMPLING_PARAMETERS_HPP_
