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
/// @brief Converter functions

#include <tmc_robot_local_planner_utils/converter.hpp>

#include <string>
#include <vector>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/logging.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>

using Eigen::Affine3d;
using Eigen::Vector3d;

namespace {

const rclcpp::Logger kLogger = rclcpp::get_logger("tmc_robot_local_planner_utils");

// Since tmc_planning_msgs 3.0.0, linear constraints and TSRs are arrays,
// but the planner holds only one of each. Use the first element.
template <typename T>
const T* GetFirstElement(const std::vector<T>& elements, const char* name) {
  if (elements.empty()) {
    return nullptr;
  }
  if (elements.size() > 1) {
    RCLCPP_WARN(kLogger, "Only the first element of %s is used (%zu given).", name, elements.size());
  }
  return &elements.front();
}

template <typename T>
bool TryTransform(const T& in,
                  const std::string& origin_frame,
                  const double& timeout,
                  const tf2_ros::Buffer& buffer,
                  T& out) {
  try {
    out = buffer.transform(in, origin_frame, tf2::durationFromSec(timeout));
  } catch (tf2::TransformException& ex) {
    return false;
  }
  return true;
}

bool TransformWithLoop(const std_msgs::msg::Header& header,
                       const std::string& origin_frame,
                       const double& timeout,
                       const tf2_ros::Buffer& buffer,
                       std::vector<geometry_msgs::msg::Transform>& in) {
  for (auto& transform : in) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = header;
    transform_stamped.transform = transform;
    geometry_msgs::msg::TransformStamped transformed;
    if (TryTransform(transform_stamped, origin_frame, timeout, buffer, transformed)) {
      transform = transformed.transform;
    } else {
      return false;
    }
  }
  return true;
}

template <typename T, typename Fn>
bool BatchTransformWithLoop(const Fn& fn,
                            const std::string& origin_frame,
                            const double& timeout,
                            const tf2_ros::Buffer& buffer,
                            T& constraints) {
  for (auto& constraint : constraints) {
    if (!fn(origin_frame, timeout, buffer, constraint)) {
      return false;
    }
  }
  return true;
}

template <typename T, typename U, typename Fn>
void BatchConvertConstMsgConstToWithLoop(const Fn& fn,
                                         const std::vector<T>& constraints,
                                         std::vector<U>& out) {
  for (auto& constraint : constraints) {
    U converted_constraint;
    fn(constraint, converted_constraint);
    if (converted_constraint) {
      out.push_back(converted_constraint);
    }
  }
}

}  // namespace

namespace tmc_robot_local_planner_utils {

// Extract x, y, yaw from the transformation matrix
Vector3d Get2DPose(const Affine3d& transform) {
  return Eigen::Vector3d(transform.translation().x(),
                         transform.translation().y(),
                         transform.rotation().eulerAngles(0, 1, 2).z());
}

// Output the transformation matrix with input as x, y, yaw
Affine3d GetTransform(const Vector3d& pose2d) {
  return Eigen::Translation3d(pose2d.x(), pose2d.y(), 0.0) *
      Eigen::AngleAxisd(pose2d.z(), Eigen::Vector3d::UnitZ());
}

// Return linear.x linear.y angular.z from Twist
Vector3d Get2DTwist(const tmc_manipulation_types::Twist& twist) {
  return Vector3d(twist[0], twist[1], twist[5]);
}

// Output Twist with input as x, y, yaw
tmc_manipulation_types::Twist GetTwist(const Vector3d& twist2d) {
  tmc_manipulation_types::Twist twist;
  twist << twist2d[0], twist2d[1], 0.0, 0.0, 0.0, twist2d[2];
  return twist;
}

bool TransformConstraints(
    const std::string& origin_frame,
    const double timeout,
    const tf2_ros::Buffer& buffer,
    tmc_planning_msgs::msg::Constraints& msg) {
  if (!BatchTransformWithLoop(TransformTSRConstraint, origin_frame,
                              timeout, buffer, msg.soft_link_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformTSRConstraint, origin_frame,
                              timeout, buffer, msg.hard_link_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformJointConstraint, origin_frame,
                              timeout, buffer, msg.soft_joint_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformJointConstraint, origin_frame,
                              timeout, buffer, msg.hard_joint_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformTSRConstraint, origin_frame,
                              timeout, buffer, msg.hard_path_link_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformJointConstraint, origin_frame,
                              timeout, buffer, msg.soft_path_joint_constraints)) {
    return false;
  }
  for (auto& constraint : msg.initial_motion_linear_constraints) {
    if (!constraint.reference.header.frame_id.empty() &&
        !TryTransform(constraint.reference, origin_frame, timeout, buffer, constraint.reference)) {
      return false;
    }
  }
  return true;
}

void ConvertConstraints(const tmc_planning_msgs::msg::Constraints& msg,
                        tmc_robot_local_planner::Constraints& constraints) {
  BatchConvertConstMsgConstToWithLoop(ConvertLinkConstraintMsgToLinkConstraint,
                                      msg.soft_link_constraints,
                                      constraints.soft_link_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertLinkConstraintMsgToLinkConstraint,
                                      msg.hard_link_constraints,
                                      constraints.hard_link_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertJointConstraintMsgToJointConstraint,
                                      msg.soft_joint_constraints,
                                      constraints.soft_joint_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertJointConstraintMsgToJointConstraint,
                                      msg.hard_joint_constraints,
                                      constraints.hard_joint_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertLinkConstraintMsgToLinkConstraint,
                                      msg.hard_path_link_constraints,
                                      constraints.hard_path_link_constraints);
  const auto goal_relative_linear_constraint = GetFirstElement(
      msg.goal_relative_linear_constraints, "goal_relative_linear_constraints");
  if (goal_relative_linear_constraint) {
    ConvertLinearConstraintMsgToLinearConstraint(*goal_relative_linear_constraint,
                                                 constraints.goal_relative_linear_constraint);
  }
  const auto initial_motion_linear_constraint = GetFirstElement(
      msg.initial_motion_linear_constraints, "initial_motion_linear_constraints");
  if (initial_motion_linear_constraint) {
    ConvertLinearConstraintMsgToLinearConstraint(*initial_motion_linear_constraint,
                                                 constraints.initial_motion_linear_constraint);
  }
  BatchConvertConstMsgConstToWithLoop(ConvertJointConstraintMsgToJointConstraint,
                                      msg.soft_path_joint_constraints,
                                      constraints.soft_path_joint_constraints);
}

bool TransformTSRConstraint(
    const std::string& origin_frame,
    const double timeout,
    const tf2_ros::Buffer& buffer,
    tmc_planning_msgs::msg::TsrLinkConstraint& constraint) {
  for (auto& tsr : constraint.tsrs) {
    geometry_msgs::msg::PoseStamped origin_to_tsr;
    origin_to_tsr.header = constraint.header;
    origin_to_tsr.pose = tsr.origin_to_tsr;
    geometry_msgs::msg::PoseStamped transformed;
    tsr.origin_to_tsr = transformed.pose;
    if (TryTransform(origin_to_tsr, origin_frame, timeout, buffer, transformed)) {
      tsr.origin_to_tsr = transformed.pose;
    } else {
      return false;
    }
  }
  return true;
}

void ConvertLinkConstraintMsgToLinkConstraint(
    const tmc_planning_msgs::msg::TsrLinkConstraint& msg,
    tmc_robot_local_planner::ILinkConstraint::Ptr& constraint_out) {
  const auto tsr_msg = GetFirstElement(msg.tsrs, "tsrs");
  if (!tsr_msg) {
    RCLCPP_WARN(kLogger, "TsrLinkConstraint without tsrs is ignored.");
    constraint_out = nullptr;
    return;
  }
  tmc_manipulation_types::TaskSpaceRegion tsr;
  tmc_manipulation_types_bridge::TaskSpaceRegionMsgToTaskSpaceRegion(*tsr_msg, tsr);
  if (tsr_msg->rotation_first) {
    constraint_out = std::make_shared<tmc_robot_local_planner::RotationFirstTsrLinkConstraint>(
        tsr, 0, std::chrono::system_clock::now().time_since_epoch().count());
  } else {
    constraint_out = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(
        tsr, 0, std::chrono::system_clock::now().time_since_epoch().count());
  }
}

bool TransformJointConstraint(
    const std::string& origin_frame,
    const double timeout,
    const tf2_ros::Buffer& buffer,
    tmc_planning_msgs::msg::RangeJointConstraint& constraint) {
  if (!TransformWithLoop(constraint.header, origin_frame, timeout, buffer,
                         constraint.min.multi_dof_joint_state.transforms)) {
    return false;
  }
  if (!TransformWithLoop(constraint.header, origin_frame, timeout, buffer,
                         constraint.max.multi_dof_joint_state.transforms)) {
    return false;
  }
  return true;
}

void ConvertJointConstraintMsgToJointConstraint(
    const tmc_planning_msgs::msg::RangeJointConstraint& msg,
    tmc_robot_local_planner::IJointConstraint::Ptr& constraint_out) {
  tmc_manipulation_types::RobotState min;
  tmc_manipulation_types::RobotState max;
  tmc_manipulation_types_bridge::RobotStateMsgToRobotState(msg.min, min);
  tmc_manipulation_types_bridge::RobotStateMsgToRobotState(msg.max, max);
  constraint_out = std::make_shared<tmc_robot_local_planner::RangeJointConstraint>(min, max, 0, 0);
}

void ConvertLinearConstraintMsgToLinearConstraint(
    const tmc_planning_msgs::msg::LinearConstraint& msg,
    tmc_robot_local_planner::LinearConstraint& constraint_out) {
  if (msg.local_origin_of_axis) {
    RCLCPP_WARN(kLogger, "local_origin_of_axis is not supported. The axis is treated as global.");
  }
  constraint_out.end_frame_id = msg.end_frame_id;
  // Zero determination can be done here, but the conversion function is dedicated to conversion, determination is done by the user
  tf2::fromMsg(msg.axis, constraint_out.axis);
  constraint_out.distance = msg.distance;
}

void ConvertLinearConstraintMsgToLinearConstraint(
    const tmc_planning_msgs::msg::LinearConstraintWithPose& msg,
    tmc_robot_local_planner::LinearConstraintWithPose& constraint_out) {
  constraint_out.end_frame_id = msg.end_frame_id;
  // Assumes it has been transformed
  tf2::fromMsg(msg.reference.pose, constraint_out.reference);
  tf2::fromMsg(msg.axis, constraint_out.axis);
  constraint_out.distance = msg.distance;
}

}  // namespace tmc_robot_local_planner_utils
