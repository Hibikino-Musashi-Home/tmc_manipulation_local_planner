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
#include "robot_collision_checker.hpp"

#include <string>
#include <utility>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_utils/qos.hpp>

namespace {
// world_joint: A virtual joint between the robot's odometry origin (odom) and the robot's trajectory
const char* const kBaseJointName = "world_joint";
}  // anonymous namespace

namespace tmc_collision_detecting_validator {

RobotCollisionChecker::RobotCollisionChecker(const rclcpp::Node::SharedPtr& node)
    : RobotCollisionChecker(node, nullptr) {}

RobotCollisionChecker::RobotCollisionChecker(const rclcpp::Node::SharedPtr& node,
                                             const tmc_utils::DynamicParameter<bool>::Ptr& verbose)
    : fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel"),
      logger_(node->get_logger()), verbose_(verbose) {
  auto robot_model = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_model.empty()) {
    robot_model = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  if (robot_model.empty()) {
    RCLCPP_FATAL(node->get_logger(), "cannot get paramter robot_description and robot_description_kinematics");
    exit(EXIT_FAILURE);
  }
  const auto robot_collision_pair = tmc_utils::GetParameter<std::string>(node, "robot_collision_pair", "");
  if (robot_collision_pair.empty()) {
    RCLCPP_FATAL(node->get_logger(), "cannot get paramter robot_collision_pair");
    exit(EXIT_FAILURE);
  }
  const auto collision_engine = tmc_utils::GetParameter<std::string>(node, "collision_engine", "fcl");

  const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
  if (kinematics_type.empty()) {
    collision_detector_ = std::make_shared<tmc_robot_collision_detector::RobotCollisionDetector>(
        robot_model, robot_collision_pair, collision_engine);
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), "Kinematics type is " << kinematics_type);
    const auto robot = fk_loader_.createSharedInstance(kinematics_type);
    robot->Initialize(robot_model);

    collision_detector_ = std::make_shared<tmc_robot_collision_detector::RobotCollisionDetector>(
        robot, robot_model, robot_collision_pair, collision_engine);
  }

  // TODO(Takeshita) プラグイン化したほうが設計としてはキレイだが，そこまで頑張る必要が現時点ではないので頑張らない
  const auto trajectory_points_collision_check_order_type = tmc_utils::GetParameter<std::string>(
      node, "trajectory_points_collision_check_order_type", "TrajectoryPointsCollisionCheckSkippingOrder");
  if (trajectory_points_collision_check_order_type == "TrajectoryPointsCollisionCheckBothEnds") {
    collision_check_order_ = std::make_shared<TrajectoryPointsCollisionCheckBothEnds>();
  } else {
    collision_check_order_ = std::make_shared<TrajectoryPointsCollisionCheckSkippingOrder>();
  }
  collision_check_order_->Init(node);
}

RobotCollisionChecker::Result RobotCollisionChecker::IsFeasible(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    std::function<bool()> interrupt,
    std::string& error_message_out) {
  // Returns false if the size of name is 0
  if (trajectory.joint_trajectory.joint_names.size() == 0 &&
      trajectory.multi_dof_joint_trajectory.joint_names.size() == 0) {
    return Result::kInvalid;
  }

  uint32_t base_index = GetTargetJointIndex(trajectory.multi_dof_joint_trajectory.joint_names, kBaseJointName);
  if (base_index == trajectory.multi_dof_joint_trajectory.joint_names.size()) {
    return Result::kInvalid;
  }

  const auto first_point_result = CheckCollisionImpl(trajectory, base_index, 0, error_message_out);
  if (first_point_result == Result::kFeasible) {
    const auto indices = collision_check_order_->GenerateIndices(trajectory);
    for (auto i : indices) {
      if (interrupt()) {
        return Result::kInterrupted;
      }
      if (i == 0) {
        continue;
      }
      const auto result = CheckCollisionImpl(trajectory, base_index, i, error_message_out);
      if (result != Result::kFeasible) {
        return result;
      }
    }
    return Result::kFeasible;
  } else {
    if (trajectory.multi_dof_joint_trajectory.points.size() == 1) {
      return first_point_result;
    }

    std::vector<Collisions> collisions;
    {
      // Skip Update since it comes here right after the first point check
      std::vector<std::pair<std::string, std::string>> contact_pair;
      if (!collision_detector_->CheckCollision(false, contact_pair)) {
        // It's strange to be here even though it should have interfered at the first point
        error_message_out = "Inconsistent state: first point was in collision, but now not in collision";
        return Result::kInvalid;
      }
      collisions = ComputeCollisions(contact_pair);
  }

    auto last_result = first_point_result;
    for (auto i = 1u; i < trajectory.multi_dof_joint_trajectory.points.size(); ++i) {
      if (interrupt()) {
        return Result::kInterrupted;
      }
      const auto update_result = UpdateCollisionDetector(trajectory, base_index, i);
      if (update_result != Result::kFeasible) {
        return update_result;
      }
      std::vector<std::pair<std::string, std::string>> contact_pair;
      if (collision_detector_->CheckCollision(false, contact_pair)) {
        // It's not good if it collides after already exiting the collision
        if (collisions.empty()) {
          return Result::kCollision;
        }

        // It's okay if it's not a new collision and the collision becomes shallower
        const auto new_collisions = ComputeCollisions(contact_pair);
        for (const auto& new_collision : new_collisions) {
          bool found = false;
          for (const auto& prev_collision : collisions) {
            if (new_collision.IsSameObjects(prev_collision)) {
              if (new_collision.depth <= prev_collision.depth) {
                found = true;
              }
              break;
            }
          }
          if (!found) {
            return Result::kCollision;
          }
        }
        collisions = new_collisions;
        last_result = Result::kCollision;
      } else {
        collisions.clear();
        last_result = Result::kFeasible;
      }
    }
    return last_result;
  }
}

RobotCollisionChecker::Result RobotCollisionChecker::UpdateCollisionDetector(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    uint32_t base_index,
    uint32_t point_index) {
  // Returns false if the sizes of name and point are different
  if ((point_index < trajectory.joint_trajectory.points.size()) &&
      (trajectory.joint_trajectory.joint_names.size() !=
           trajectory.joint_trajectory.points[point_index].positions.size())) {
    return Result::kInvalid;
  }
  // Assume there is no base trajectory to judge and return false
  if (base_index >= trajectory.multi_dof_joint_trajectory.points[point_index].transforms.size()) {
    return Result::kInvalid;
  }

  // Setting the trajectory
  if (point_index < trajectory.joint_trajectory.points.size()) {
    tmc_manipulation_types::JointState state =
        {trajectory.joint_trajectory.joint_names,
         trajectory.joint_trajectory.points[point_index].positions,
         Eigen::VectorXd(),
         Eigen::VectorXd()};
    collision_detector_->SetRobotNamedAngle(state);
  }
  collision_detector_->SetRobotTransform(
      trajectory.multi_dof_joint_trajectory.points[point_index].transforms[base_index]);
  return Result::kFeasible;
}

RobotCollisionChecker::Result RobotCollisionChecker::CheckCollisionImpl(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    uint32_t base_index,
    uint32_t point_index,
    std::string& error_message_out) {
  const auto update_result = UpdateCollisionDetector(trajectory, base_index, point_index);
  if (update_result != Result::kFeasible) {
    return update_result;
  }

  // Interference check
  std::vector<std::pair<std::string, std::string>> contact_pair;
  if (collision_detector_->CheckCollision(true, contact_pair)) {
    if (verbose_ && verbose_->value()) {
      std::stringstream stream;
      stream << "Collision: " << contact_pair.front().first << " - " << contact_pair.front().second << std::endl;
      for (auto j = 0; j < trajectory.joint_trajectory.joint_names.size(); ++j) {
        stream << "  " << trajectory.joint_trajectory.joint_names[j] << ": "
                << trajectory.joint_trajectory.points[point_index].positions[j] << std::endl;
      }
      stream << "  base_pos_x: "
             << trajectory.multi_dof_joint_trajectory.points[point_index].transforms[base_index].translation().x()
             << std::endl;
      stream << "  base_pos_y: "
             << trajectory.multi_dof_joint_trajectory.points[point_index].transforms[base_index].translation().y()
             << std::endl;
      const auto euler_angles =
          trajectory.multi_dof_joint_trajectory.points[point_index].transforms[base_index].linear().eulerAngles(0, 1, 2);  // NOLINT
      stream << "  base_ori_yaw: " << euler_angles[2] << std::endl;
      error_message_out = stream.str();
    }
    return Result::kCollision;
  }
  return Result::kFeasible;
}


void RobotCollisionChecker::CreateObstacleObjects(const moveit_msgs::msg::PlanningSceneWorld& environment) {
  for (const auto& collision_object : environment.collision_objects) {
    // No coordinate transformation is performed, assuming objects with transformed coordinates are coming in
    tmc_manipulation_types::OuterObjectParameters params;
    tmc_manipulation_types_bridge::CollisionObjectToOuterObjectParameters(collision_object, params);
    collision_detector_->CreateOuterObject(params);
  }
}

void RobotCollisionChecker::AttachObjectToRobot(const moveit_msgs::msg::AttachedCollisionObject& collision_object) {
  // If not included in the obstacle environment, add it
  // Utilize the specification of RobotCollisionDetector that external obstacles are included in the group "OUTER"
  const auto object_names = collision_detector_->GetObjectNameListByGroup("OUTER");
  if (std::find(object_names.begin(), object_names.end(), collision_object.object.id) == object_names.end()) {
    // When the grasping setting is performed, the position, etc., are overwritten, so just add the object
    tmc_manipulation_types::OuterObjectParameters params;
    tmc_manipulation_types_bridge::CollisionObjectToOuterObjectParameters(collision_object.object, params);
    collision_detector_->CreateOuterObject(params);
  }
  // Grasping setting
  Eigen::Affine3d link_to_object;
  tf2::fromMsg(collision_object.object.pose, link_to_object);
  collision_detector_->HoldObject(collision_object.object.id, collision_object.link_name, link_to_object);
}

void RobotCollisionChecker::DisableCollisionCheckObjectToObject(const std::string& object_name1,
                                                                const std::string& object_name2) {
  try {
    collision_detector_->DisableCollisionCheckObjectToObject(object_name1, object_name2);
  } catch(const std::domain_error& error) {
    RCLCPP_WARN_STREAM(logger_, "DisableCollisionCheck failure: " <<object_name1 << " - " << object_name2);
    RCLCPP_DEBUG_STREAM(logger_, error.what());
  }
}

void RobotCollisionChecker::DestroyAllObstacleObject() {
  collision_detector_->DestroyAllOuterObject();
}

void RobotCollisionChecker::UpdateJointState(const tmc_manipulation_types::JointState& joint_state) {
  collision_detector_->SetRobotNamedAngle(joint_state);
}

std::vector<RobotCollisionChecker::Collisions> RobotCollisionChecker::ComputeCollisions(
    const std::vector<std::pair<std::string, std::string>>& contact_pair) const {
  // Since contact_pair is known, assume collision_detector is updated
  std::vector<Collisions> collisions;
  for (const auto& pair : contact_pair) {
    Collisions collision;
    collision.name_1 = pair.first;
    collision.name_2 = pair.second;
    collision_detector_->CheckCollisionPair(pair.first, pair.second, collision.depth);
    collisions.push_back(collision);
  }
  return collisions;
}


void CollisionEnvironment::UpdateCollisionDetector(const RobotCollisionChecker::Ptr& collision_checker) const {
  collision_checker->DestroyAllObstacleObject();
  collision_checker->CreateObstacleObjects(environment);
  for (const auto& collision_object : attached_objects) {
    collision_checker->AttachObjectToRobot(collision_object);
  }
  for (const auto& pair : disabled_object_pairs.pairs) {
    collision_checker->DisableCollisionCheckObjectToObject(pair.names[0], pair.names[1]);
  }
}

void CollisionEnvironment::DumpEnvironment(const tmc_utils::MessageLogger::Ptr& logger) const {
  if (logger) {
    logger->SaveMessage(".env", environment);
    moveit_msgs::msg::RobotState msg;
    msg.attached_collision_objects = attached_objects;
    logger->SaveMessage(".attached", msg);
    logger->SaveMessage(".disabled", disabled_object_pairs);
  }
}

CollisionEnvironmentCache::CollisionEnvironmentCache(const rclcpp::Node::SharedPtr& node) {
  environment_ = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>>(
      node, "collision_environment_server/transformed_environment", tmc_utils::BestEffortQoS());

  const auto attached_object_topics = tmc_utils::GetParameter(
      node, "attached_object_topics", std::vector<std::string>({"attached_object_publisher/attached_object"}));
  for (const auto& topic_name : attached_object_topics) {
    attached_objects_.emplace_back(std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
        node, topic_name, tmc_utils::BestEffortQoS()));
  }

  disabled_object_pairs_ = std::make_shared<tmc_utils::CachingSubscriber<tmc_planning_msgs::msg::ObjectPairsList>>(
      node, "~/collisions_disabled_object_pairs", tmc_utils::ReliableVolatileQoS());
}

CollisionEnvironment CollisionEnvironmentCache::GetCollisionEnvironment() const {
  CollisionEnvironment env;
  if (environment_->IsSubscribed()) {
    env.environment = environment_->GetValue();
  }

  for (const auto& attached_object : attached_objects_) {
    if (!attached_object->IsSubscribed()) {
      continue;
    }
    const auto value = attached_object->GetValue();
    env.attached_objects.insert(
        env.attached_objects.end(), value.attached_collision_objects.begin(), value.attached_collision_objects.end());
  }

  if (disabled_object_pairs_->IsSubscribed()) {
    env.disabled_object_pairs = disabled_object_pairs_->GetValue();
  }
  return env;
}

}  // namespace tmc_collision_detecting_validator
