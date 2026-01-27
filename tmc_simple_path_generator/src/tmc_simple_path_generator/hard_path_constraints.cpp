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

#include <tmc_simple_path_generator/hard_path_constraints.hpp>

#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

#include "common.hpp"

namespace {
constexpr double kStepThresholdRate = std::sqrt(5.0) / 2.0;
constexpr double kDistanceThreshold = 1.0e-3;

tmc_manipulation_types::RobotState ExtractRobotState(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory, uint32_t index) {
  tmc_manipulation_types::RobotState robot_state;
  robot_state.joint_state.name = trajectory.joint_trajectory.joint_names;
  robot_state.multi_dof_joint_state.names = trajectory.multi_dof_joint_trajectory.joint_names;
  robot_state.joint_state.position = trajectory.joint_trajectory.points[index].positions;
  robot_state.multi_dof_joint_state.poses = trajectory.multi_dof_joint_trajectory.points[index].transforms;
  return robot_state;
}

void UpdateTrajectoryPoint(const tmc_manipulation_types::RobotState& robot_state, uint32_t index,
                           tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory) {
  dst_trajectory.joint_trajectory.points[index].positions = robot_state.joint_state.position;
  dst_trajectory.multi_dof_joint_trajectory.points[index].transforms = robot_state.multi_dof_joint_state.poses;
}

void InsertTrajectoryPoint(const tmc_manipulation_types::RobotState& robot_state, uint32_t index,
                           tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory) {
  tmc_manipulation_types::TimedJointTrajectoryPoint joint_trajectory_point;
  joint_trajectory_point.positions = robot_state.joint_state.position;
  dst_trajectory.joint_trajectory.points.insert(
      dst_trajectory.joint_trajectory.points.begin() + index, joint_trajectory_point);

  tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint multi_dof_joint_trajectory_point;
  multi_dof_joint_trajectory_point.transforms = robot_state.multi_dof_joint_state.poses;
  dst_trajectory.multi_dof_joint_trajectory.points.insert(
      dst_trajectory.multi_dof_joint_trajectory.points.begin() + index, multi_dof_joint_trajectory_point);
}

std::optional<double> CalculateDistance(const Eigen::Vector3d& line_start,
                                        const Eigen::Vector3d& line_end,
                                        const Eigen::Vector3d& point) {
  const auto start_to_point = point - line_start;
  const auto start_to_end = line_end - line_start;
  const auto end_to_start = line_start - line_end;
  const auto end_to_point = point - line_end;
  if (start_to_point.dot(start_to_end) <= 0.0) {
    return start_to_point.norm();
  } else if (end_to_point.dot(end_to_start) <= 0.0) {
    // Ignore the distance on the endpoint side
    return std::nullopt;
  } else {
    return (start_to_point.cross(start_to_end)).norm() / start_to_end.norm();
  }
}

std::optional<tmc_manipulation_types::RobotState> CalculateIK(
    const tmc_manipulation_types::RobotState& initial_state,
    const Eigen::Affine3d& origin_to_waypoint,
    const std::string& end_effector_frame,
    const tmc_simple_path_generator::SampleGoalGenerator::Ptr& sampler,
    const tmc_simple_path_generator::SamplingParameters::Ptr& params) {
  const auto tsr = tmc_manipulation_types::TaskSpaceRegion(origin_to_waypoint,
                                                           Eigen::Affine3d::Identity(),
                                                           tmc_manipulation_types::RegionValues::Zero(),
                                                           tmc_manipulation_types::RegionValues::Zero(),
                                                           "", end_effector_frame);
  const auto constraint = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0);
  tmc_manipulation_types::RobotState waypoint_state;
  if (!sampler->SampleFromLinkConstraints(initial_state, {constraint}, params, waypoint_state)) {
    return std::nullopt;
  }
  return waypoint_state;
}

}  // namespace

namespace tmc_simple_path_generator {

HardPathLinkConstraints::HardPathLinkConstraints(const rclcpp::Node::SharedPtr& node)
    : fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel"),
      engine_(std::random_device()()) {
  // Initialization of FK
  const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
  if (kinematics_type.empty()) {
    robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>();
  } else {
    robot_ = fk_loader_.createSharedInstance(kinematics_type);
  }

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
      robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  robot_->Initialize(robot_description);

  // Initialization of IK
  closest_sampler_ = std::make_shared<SampleGoalGenerator>();
  closest_sampler_->Initialize(node);
  closest_sampler_->set_is_from_random_initial_state(false);
}

bool HardPathLinkConstraints::ConstrainTrajectory(
    const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& link_constraints,
    const SamplingParameters::Ptr& params,
    tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory) {
  return ConstrainTrajectory(link_constraints, params, 0, 0, dst_trajectory);
}

bool HardPathLinkConstraints::ConstrainTrajectory(
    const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& link_constraints,
    const SamplingParameters::Ptr& params,
    uint32_t initial_state_offset,
    uint32_t goal_state_offset,
    tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory) {
  uint32_t points_threshold = 2 + initial_state_offset + goal_state_offset;
  if (dst_trajectory.joint_trajectory.points.size() < points_threshold ||
      dst_trajectory.multi_dof_joint_trajectory.points.size() < points_threshold) {
    return false;
  }
  if (dst_trajectory.joint_trajectory.points.size() != dst_trajectory.multi_dof_joint_trajectory.points.size()) {
    return false;
  }
  if (link_constraints.empty()) {
    return true;
  }

  auto constraint = SampleConstraints(link_constraints, engine_);

  // Modify the midpoint to satisfy the forcibly selected constraint
  for (auto i = 1 + initial_state_offset;
       i < dst_trajectory.joint_trajectory.points.size() - 1 - goal_state_offset; ++i) {
    const auto robot_state = ExtractRobotState(dst_trajectory, i);
    // Only calculate when it needs to be corrected
    const auto origin_to_closest = CalculateOriginToClosest(constraint, robot_state);
    if (!origin_to_closest) {
      continue;
    }
    const auto closest_state = ConstrainImpl(origin_to_closest.value(), constraint, robot_state, params);
    if (!closest_state) {
      return false;
    }
    UpdateTrajectoryPoint(closest_state.value(), i, dst_trajectory);
  }
  // Add trajectory points that satisfy the selected constraint after initial and before goal
  {
    const auto robot_state = ExtractRobotState(dst_trajectory, initial_state_offset);
    const auto origin_to_closest = CalculateOriginToClosest(constraint, robot_state);
    if (origin_to_closest) {
      const auto closest_state = ConstrainImpl(origin_to_closest.value(), constraint, robot_state, params);
      if (!closest_state) {
        return false;
      }
      InsertTrajectoryPoint(closest_state.value(), 1 + initial_state_offset, dst_trajectory);
    }
  }
  {
    const auto robot_state = ExtractRobotState(dst_trajectory,
                                               dst_trajectory.joint_trajectory.points.size() - 1 - goal_state_offset);
    const auto origin_to_closest = CalculateOriginToClosest(constraint, robot_state);
    if (origin_to_closest) {
      const auto closest_state = ConstrainImpl(origin_to_closest.value(), constraint, robot_state, params);
      if (!closest_state) {
        return false;
      }
      InsertTrajectoryPoint(closest_state.value(),
                            dst_trajectory.joint_trajectory.points.size() - 1 - goal_state_offset, dst_trajectory);
    }
  }
  return true;
}

std::optional<Eigen::Affine3d> HardPathLinkConstraints::CalculateOriginToClosest(
    const tmc_robot_local_planner::ILinkConstraint::Ptr& link_constraint,
    const tmc_manipulation_types::RobotState& robot_state) {
  robot_->SetNamedAngle(robot_state.joint_state);
  robot_->SetRobotTransform(robot_state.multi_dof_joint_state.poses[0]);

  const auto origin_to_link = robot_->GetObjectTransform(link_constraint->GetLinkName());
  const auto origin_to_closest = link_constraint->CalcClosest(origin_to_link);

  // There is no particular basis for the number 1mm
  if ((origin_to_closest.translation() - origin_to_link.translation()).norm() > kDistanceThreshold) {
    return origin_to_closest;
  } else {
    return std::nullopt;
  }
}

std::optional<tmc_manipulation_types::RobotState> HardPathLinkConstraints::ConstrainImpl(
    const Eigen::Affine3d& origin_to_closest,
    const tmc_robot_local_planner::ILinkConstraint::Ptr& link_constraint,
    const tmc_manipulation_types::RobotState& robot_state,
    const SamplingParameters::Ptr& params) {
  tmc_manipulation_types::TaskSpaceRegion tsr(
      origin_to_closest, Eigen::Affine3d::Identity(),
      tmc_manipulation_types::RegionValues::Zero(), tmc_manipulation_types::RegionValues::Zero(),
      link_constraint->GetOriginName(), link_constraint->GetLinkName());
  const auto closest_pose_constraint = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0);

  tmc_manipulation_types::RobotState closest_state;
  if (closest_sampler_->SampleFromLinkConstraints(robot_state, {closest_pose_constraint}, params, closest_state)) {
    return closest_state;
  } else {
    return std::nullopt;
  }
}


GoalRelativeLinearConstraint::GoalRelativeLinearConstraint(const rclcpp::Node::SharedPtr& node)
    : logger_(node->get_logger()),
      clock_(node->get_clock()),
      fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel") {
  linear_step_ = tmc_utils::GetParameter<double>(node, "linear_constraint_step", 0.02);

  const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
  if (kinematics_type.empty()) {
    robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>();
  } else {
    robot_ = fk_loader_.createSharedInstance(kinematics_type);
  }

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
      robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  robot_->Initialize(robot_description);

  sampler_ = std::make_shared<SampleGoalGenerator>();
  sampler_->Initialize(node);
  sampler_->set_is_from_random_initial_state(false);
}

std::optional<std::vector<tmc_manipulation_types::RobotState>> GoalRelativeLinearConstraint::SampleLinearPath(
    const tmc_robot_local_planner::LinearConstraint linear_constraint,
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::RobotState& goal_state,
    const SamplingParameters::Ptr& params) {
  if (linear_constraint.end_frame_id.empty()) {
    return std::vector<tmc_manipulation_types::RobotState>();
  }
  if (linear_constraint.axis.isZero()) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: axis is zero");
    return std::nullopt;
  }
  if (linear_constraint.distance < std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: distance is zero or negative");
    return std::nullopt;
  }
  // Checking if the frame name is valid is only possible with this method, which is somewhat computationally wasteful
  try {
    robot_->GetObjectTransform(linear_constraint.end_frame_id);
  } catch (const std::domain_error& e) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: %s", e.what());
    return std::nullopt;
  }

  // Fixing at index 0 is not good, but since the entire generator is implemented that way, no consideration is given
  robot_->SetNamedAngle(initial_state.joint_state);
  robot_->SetRobotTransform(initial_state.multi_dof_joint_state.poses[0]);
  const auto origin_to_initial = robot_->GetObjectTransform(linear_constraint.end_frame_id);

  robot_->SetNamedAngle(goal_state.joint_state);
  robot_->SetRobotTransform(goal_state.multi_dof_joint_state.poses[0]);
  const auto origin_to_goal = robot_->GetObjectTransform(linear_constraint.end_frame_id);

  const auto axis = linear_constraint.axis.normalized();

  std::vector<double> distances;
  for (auto distance = linear_step_; distance < linear_constraint.distance; distance += linear_step_) {
    distances.push_back(distance);
  }
  distances.push_back(linear_constraint.distance);

  // Reverse later, goal_state is unnecessary but added for the implementation of using the previous solution as the initial value for IK, will be removed later
  std::vector<tmc_manipulation_types::RobotState> middle_states_out = {goal_state};
  for (const auto distance : distances) {
    const auto goal_to_waypoint = Eigen::Translation3d(distance * axis) * Eigen::AngleAxisd::Identity();
    const auto origin_to_waypoint = origin_to_goal * goal_to_waypoint;

    // If the initial end-effector position is near the line, terminate the calculation there
    // Strictly speaking, it's the distance between a point and a line segment, but it's considered acceptable if it's at a reasonable distance proportional to the step
    const auto distance_from_initial = (origin_to_waypoint.translation() - origin_to_initial.translation()).norm();
    if (distance_from_initial < kStepThresholdRate * linear_step_) {
      break;
    }

    tmc_manipulation_types::TaskSpaceRegion tsr(origin_to_waypoint,
                                                Eigen::Affine3d::Identity(),
                                                tmc_manipulation_types::RegionValues::Zero(),
                                                tmc_manipulation_types::RegionValues::Zero(),
                                                "", linear_constraint.end_frame_id);
    const auto constraint = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0);

    tmc_manipulation_types::RobotState waypoint_state;
    if (!sampler_->SampleFromLinkConstraints(middle_states_out.back(), {constraint}, params, waypoint_state)) {
      // Return failure if IK cannot be solved, the idea is that it's bad to throw command values that make IK unsolvable
      return std::nullopt;
    }
    middle_states_out.push_back(waypoint_state);
  }

  // Correct the order and delete the goal_state added for calculation
  std::reverse(middle_states_out.begin(), middle_states_out.end());
  middle_states_out.pop_back();

  return middle_states_out;
}


InitialMotionLinearConstraint::InitialMotionLinearConstraint(const rclcpp::Node::SharedPtr& node)
    : logger_(node->get_logger()),
      clock_(node->get_clock()),
      is_from_random_state_(false),
      fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel") {
  linear_step_ = tmc_utils::GetParameter<double>(node, "linear_constraint_step", 0.02);
  distance_threshold_ = tmc_utils::GetParameter<double>(node, "distance_threshold", linear_step_);

  const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
  if (kinematics_type.empty()) {
    robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>();
  } else {
    robot_ = fk_loader_.createSharedInstance(kinematics_type);
  }
  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
      robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  robot_->Initialize(robot_description);

  sampler_ = std::make_shared<SampleGoalGenerator>();
  sampler_->Initialize(node);
  sampler_->set_is_from_random_initial_state(false);
}

std::optional<std::vector<tmc_manipulation_types::RobotState>> InitialMotionLinearConstraint::SampleLinearPath(
    const tmc_robot_local_planner::LinearConstraintWithPose& linear_constraint,
    const tmc_manipulation_types::RobotState& initial_state,
    const SamplingParameters::Ptr& params) {
  if (linear_constraint.end_frame_id.empty()) {
    return std::vector<tmc_manipulation_types::RobotState>();
  }
  if (linear_constraint.axis.isZero()) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: axis is zero");
    return std::nullopt;
  }
  if (linear_constraint.distance < std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: distance is zero or negative");
    return std::nullopt;
  }

  // Fixing at index 0 is not good, but since the entire generator is implemented that way, no consideration is given
  robot_->SetNamedAngle(initial_state.joint_state);
  robot_->SetRobotTransform(initial_state.multi_dof_joint_state.poses[0]);
  Eigen::Affine3d origin_to_initial;
  try {
    origin_to_initial = robot_->GetObjectTransform(linear_constraint.end_frame_id);
  } catch (const std::domain_error& e) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: %s", e.what());
    return std::nullopt;
  }

  // Determine the end-effector position to be passed through
  std::vector<double> distances;
  for (auto distance = linear_step_; distance < linear_constraint.distance; distance += linear_step_) {
    distances.push_back(distance);
  }
  if (distances.empty() || std::abs(distances.back() - linear_constraint.distance) > kDistanceThreshold) {
    distances.push_back(linear_constraint.distance);
  }

  const auto axis = linear_constraint.axis.normalized();
  std::vector<Eigen::Affine3d> waypoints;
  for (const auto distance : distances) {
    const auto ref_to_waypoint = Eigen::Translation3d(distance * axis) * Eigen::AngleAxisd::Identity();
    const auto origin_to_waypoint = linear_constraint.reference * ref_to_waypoint;
    waypoints.push_back(origin_to_waypoint);
  }

  // Capsule with the constraint line segment as the axis, but no linear constraint is applied outside the cylinder on the endpoint side
  const auto distance_from_line = CalculateDistance(linear_constraint.reference.translation(),
                                                    waypoints.back().translation(),
                                                    origin_to_initial.translation());
  if (!distance_from_line || distance_from_line.value() > distance_threshold_) {
    return std::vector<tmc_manipulation_types::RobotState>();
  }

  // Determine the merging point
  int32_t linear_index = static_cast<int32_t>(waypoints.size() - 1);
  while (linear_index > 0) {
    const auto distance_from_initial = (waypoints[linear_index].translation() - origin_to_initial.translation()).norm();
    if (distance_from_initial < kStepThresholdRate * linear_step_) {
      if (distance_from_initial < kDistanceThreshold) {
        linear_index += 1;
      }
      break;
    }
    --linear_index;
  }
  if (linear_index < 0 || linear_index >= static_cast<int32_t>(waypoints.size())) {
    return std::vector<tmc_manipulation_types::RobotState>();
  }

  // Determine the robot state at the waypoint using inverse kinematics
  if (is_from_random_state_) {
    // Solve in reverse order up to linear_index, solve with random initial value IK only at the beginning
    std::vector<tmc_manipulation_types::RobotState> middle_states_out;
    {
      sampler_->set_is_from_random_initial_state(true);
      const auto waypoint_state = CalculateIK(
          initial_state, waypoints.back(), linear_constraint.end_frame_id, sampler_, params);
      if (!waypoint_state) {
        // Return failure if IK cannot be solved, the idea is that it's bad to throw command values that make IK unsolvable
        return std::nullopt;
      }
      middle_states_out.push_back(waypoint_state.value());
    }
    sampler_->set_is_from_random_initial_state(false);
    for (int32_t i = waypoints.size() - 2; i >= linear_index; --i) {
      const auto waypoint_state = CalculateIK(
          middle_states_out.back(), waypoints[i], linear_constraint.end_frame_id, sampler_, params);
      if (!waypoint_state) {
        return std::nullopt;
      }
      middle_states_out.push_back(waypoint_state.value());
    }
    std::reverse(middle_states_out.begin(), middle_states_out.end());
    return middle_states_out;
  } else {
    // Solve sequentially from the initial state without randomness
    sampler_->set_is_from_random_initial_state(false);
    std::vector<tmc_manipulation_types::RobotState> middle_states_out = {initial_state};
    for ( ; linear_index < static_cast<int32_t>(waypoints.size()); ++linear_index) {
      const auto waypoint_state = CalculateIK(
          middle_states_out.back(), waypoints[linear_index], linear_constraint.end_frame_id, sampler_, params);
      if (!waypoint_state) {
        return std::nullopt;
      }
      middle_states_out.push_back(waypoint_state.value());
    }
    middle_states_out.erase(middle_states_out.begin());
    return middle_states_out;
  }
}


}  // namespace tmc_simple_path_generator
