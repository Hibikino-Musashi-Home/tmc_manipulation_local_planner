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
#ifndef TMC_COLLISION_DETECTING_VALIDATOR_ROBOT_COLLISION_CHECKER_HPP_
#define TMC_COLLISION_DETECTING_VALIDATOR_ROBOT_COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <pluginlib/class_loader.hpp>

#include <tmc_robot_collision_detector/robot_collision_detector.hpp>

// It is necessary to include the ones mixed with pinocchio first
#include <rclcpp/rclcpp.hpp>  // NOLINT
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_planning_msgs/msg/object_pairs_list.hpp>
#include <tmc_utils/caching_subscriber.hpp>
#include <tmc_utils/msg_io.hpp>
#include <tmc_utils/parameters.hpp>

#include "utils.hpp"

namespace tmc_collision_detecting_validator {

class RobotCollisionChecker {
 public:
  using Ptr = std::shared_ptr<RobotCollisionChecker>;

  explicit RobotCollisionChecker(const rclcpp::Node::SharedPtr& node);
  RobotCollisionChecker(const rclcpp::Node::SharedPtr& node, const tmc_utils::DynamicParameter<bool>::Ptr& verbose);

  enum class Result {
    kFeasible = 0,
    kCollision = 1,
    kInterrupted = 2,
    kInvalid = 3,
  };

  /// @brief Feasibility Check Stop immediately after checking collision
  /// @param trajectory Timed Trajectory
  /// @param interrupt Interruption function
  /// @param error_message_out Error message
  /// @return Result Validation result
  Result IsFeasible(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                    std::function<bool()> interrupt,
                    std::string& error_message_out);

  /// @brief Create obstacle objects from the environment
  /// @param environment Collision environment
  void CreateObstacleObjects(const moveit_msgs::msg::PlanningSceneWorld& environment);

  /// @brief Attach object to robot
  /// @param collision_object Attached collision object
  void AttachObjectToRobot(const moveit_msgs::msg::AttachedCollisionObject& collision_object);

  /// @brief Disable collision check between object and object
  /// @param object_name1 Object name 1
  /// @param object_name2 Object name 2
  void DisableCollisionCheckObjectToObject(const std::string& object_name1, const std::string& object_name2);

  /// @brief Clear robot outside obstacle objects
  void DestroyAllObstacleObject();

  /// @brief Update joint state
  /// @param joint_state Joint state
  void UpdateJointState(const tmc_manipulation_types::JointState& joint_state);

 private:
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IRobotKinematicsModel> fk_loader_;

  tmc_robot_collision_detector::RobotCollisionDetector::Ptr collision_detector_;

  rclcpp::Logger logger_;
  tmc_utils::DynamicParameter<bool>::Ptr verbose_;

  std::shared_ptr<ITrajectoryPointsCollisionCheckOrder> collision_check_order_;

  Result UpdateCollisionDetector(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                                 uint32_t base_index,
                                 uint32_t point_index);
  Result CheckCollisionImpl(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                            uint32_t base_index,
                            uint32_t point_index,
                            std::string& error_message_out);

  /// Collision depth information
  struct Collisions {
    std::string name_1;
    std::string name_2;
    double depth;

    bool IsSameObjects(const Collisions& other) const {
      return ((name_1 == other.name_1) && (name_2 == other.name_2)) ||
          ((name_1 == other.name_2) && (name_2 == other.name_1));
    }
  };
  std::vector<Collisions> ComputeCollisions(const std::vector<std::pair<std::string, std::string>>& contact_pair) const;
};


struct CollisionEnvironment {
  moveit_msgs::msg::PlanningSceneWorld environment;
  std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_objects;
  tmc_planning_msgs::msg::ObjectPairsList disabled_object_pairs;

  void UpdateCollisionDetector(const RobotCollisionChecker::Ptr& collision_checker) const;

  /// @brief Dump collision objects
  /// @param logger logger pointer
  void DumpEnvironment(const tmc_utils::MessageLogger::Ptr& logger) const;
};


class CollisionEnvironmentCache {
 public:
  explicit CollisionEnvironmentCache(const rclcpp::Node::SharedPtr& node);

  /// @brief Get the latest collision environment
  /// @return CollisionEnvironment
  CollisionEnvironment GetCollisionEnvironment() const;

 private:
  tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>::Ptr environment_;
  std::vector<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>::Ptr> attached_objects_;
  tmc_utils::CachingSubscriber<tmc_planning_msgs::msg::ObjectPairsList>::Ptr disabled_object_pairs_;
};


}  // namespace tmc_collision_detecting_validator
#endif  // TMC_COLLISION_DETECTING_VALIDATOR_ROBOT_COLLISION_CHECKER_HPP_
