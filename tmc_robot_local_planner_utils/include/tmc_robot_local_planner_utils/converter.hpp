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
#ifndef TMC_ROBOT_LOCAL_PLANNER_UTILS_CONVERTER_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_UTILS_CONVERTER_HPP_

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/buffer.h>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_planning_msgs/msg/constraints.hpp>
#include <tmc_planning_msgs/msg/tsr_link_constraint.hpp>
#include <tmc_robot_local_planner/constraints.hpp>
#include <tmc_robot_local_planner/range_joint_constraint.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

namespace tmc_robot_local_planner_utils {

/// @brief Returns 3d(x, y, theta)
/// @param transform Affine coordinates
/// @return 3d vector
Eigen::Vector3d Get2DPose(const Eigen::Affine3d& transform);

/// @brief Returns the transformation matrix
/// @param 3d(x, y, theta) vector
/// @return transform Affine coordinates
Eigen::Affine3d GetTransform(const Eigen::Vector3d& pose2d);

/// @brief Returns linear.x linear.y angular.z
/// @param twist Velocity of linear and angular components
/// @return 3d vector
Eigen::Vector3d Get2DTwist(const tmc_manipulation_types::Twist& twist);

/// @brief Returns Twist
/// @param 3d(linear.x linear.y angular.z) vector
/// @return twist Velocity of linear and angular components
tmc_manipulation_types::Twist GetTwist(const Eigen::Vector3d& twist2d);

/// @brief Transforms each constraint
/// @param origin_frame Reference frame
/// @param timeout tf timeout [sec]
/// @param buffer tf2 buffer
/// @param msg Message type constraint
/// @return Function success determination
bool TransformConstraints(const std::string& origin_frame,
                          const double timeout,
                          const tf2_ros::Buffer& buffer,
                          tmc_planning_msgs::msg::Constraints& msg);

/// @brief Converts ROS message type constraint to non-ROS type
/// @param msg ROS message type constraint
/// @param constraints Conversion result
void ConvertConstraints(const tmc_planning_msgs::msg::Constraints& msg,
                        tmc_robot_local_planner::Constraints& constraints);

/// @brief Transforms link constraint
/// @param origin_frame Origin
/// @param timeout tf timeout [sec]
/// @param buffer tf2 buffer
/// @param msg Message type link constraint
/// @return Function success determination
bool TransformTSRConstraint(const std::string& origin_frame,
                            const double timeout,
                            const tf2_ros::Buffer& buffer,
                            tmc_planning_msgs::msg::TsrLinkConstraint& msg);

/// @brief Converts ROS message type link constraint to non-ROS type
/// @param msg ROS message type constraint
/// @param const_out Conversion result
void ConvertLinkConstraintMsgToLinkConstraint(
    const tmc_planning_msgs::msg::TsrLinkConstraint& msg,
    tmc_robot_local_planner::ILinkConstraint::Ptr& constraint_out);

/// @brief Transforms joint constraint
/// @param origin_frame Origin
/// @param timeout tf timeout [sec]
/// @param buffer tf2 buffer
/// @param msg Message type joint constraint
/// @return Function success determination
bool TransformJointConstraint(const std::string& origin_frame,
                              const double timeout,
                              const tf2_ros::Buffer& buffer,
                              tmc_planning_msgs::msg::RangeJointConstraint& msg);

/// @brief Converts ROS message type joint constraint to non-ROS type
/// @param msg ROS message type constraint
/// @param const_out Conversion result
void ConvertJointConstraintMsgToJointConstraint(
    const tmc_planning_msgs::msg::RangeJointConstraint& msg,
    tmc_robot_local_planner::IJointConstraint::Ptr& constraint_out);

/// @brief Converts ROS message type linear constraint to non-ROS type
/// @param msg ROS message type constraint
/// @param const_out Conversion result
void ConvertLinearConstraintMsgToLinearConstraint(
    const tmc_planning_msgs::msg::LinearConstraint& msg,
    tmc_robot_local_planner::LinearConstraint& constraint_out);

/// @brief Converts ROS message type linear constraint to non-ROS type
/// @param msg ROS message type constraint
/// @param const_out Conversion result
void ConvertLinearConstraintMsgToLinearConstraint(
    const tmc_planning_msgs::msg::LinearConstraintWithPose& msg,
    tmc_robot_local_planner::LinearConstraintWithPose& constraint_out);

}  // namespace tmc_robot_local_planner_utils
#endif  // TMC_ROBOT_LOCAL_PLANNER_UTILS_CONVERTER_HPP_
