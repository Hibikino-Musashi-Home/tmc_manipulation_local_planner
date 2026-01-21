/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
/// @brief Test of SimplePathGenerator

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_planning_msgs/action/generate_robot_trajectories.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_local_planner/range_joint_constraint.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

#include <tmc_simple_path_generator/simple_path_generator.hpp>

namespace {
const tmc_manipulation_types::NameSeq kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
const tmc_manipulation_types::NameSeq kBaseNames = {"world_joint"};
const char* const kEndFrame = "link7";
const std::vector<double> kJointWeights = {5.0, 1.0, 0.5, 0.5, 0.5, 1.0};
const std::vector<double> kBaseWeights = {5.0, 5.0, 1.0};
constexpr int32_t kMaxSimpleTrajectoryNum = 6;
constexpr int32_t kMaxTrajectoryNum = 30;
constexpr int32_t kGenerationThreadNum = 2;

constexpr double kEpsilon = 0.005;



void DeclareTestParameters(rclcpp::Node::SharedPtr node) {
  node->declare_parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("robot_description_kinematics", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("base_movement_type", 1);
  node->declare_parameter("joint_names", kJointNames);
  node->declare_parameter("joint_weights", kJointWeights);
  node->declare_parameter("base_names", kBaseNames);
  node->declare_parameter("base_weights", kBaseWeights);
  node->declare_parameter("max_simple_trajectory_num", kMaxSimpleTrajectoryNum);
  node->declare_parameter("max_trajectory_num", kMaxTrajectoryNum);
  node->declare_parameter("generation_thread_num", kGenerationThreadNum);
  node->declare_parameter("generator_timeout", 1.0);
  node->declare_parameter("interpolation_type", "tmc_simple_path_generator/PlanarTrajectoryInterpolatorPlain");
  node->declare_parameter("end_effectors", std::vector<std::string>({kEndFrame}));
  node->declare_parameter(std::string("ik_joints.") + kEndFrame, kJointNames);
}

Eigen::Affine3d CalculateEndFramePose(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const tmc_manipulation_types::RobotState& state) {
  robot->SetRobotTransform(state.multi_dof_joint_state.poses[0]);
  robot->SetNamedAngle(state.joint_state);
  return robot->GetObjectTransform(kEndFrame);
}

Eigen::Affine3d CalculateEndFramePose(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    uint32_t point_index) {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = trajectory.joint_trajectory.joint_names;
  state.joint_state.position = trajectory.joint_trajectory.points[point_index].positions;
  state.multi_dof_joint_state.poses = trajectory.multi_dof_joint_trajectory.points[point_index].transforms;
  return CalculateEndFramePose(robot, state);
}

tmc_manipulation_types::RobotState GenerateInitialState() {
  tmc_manipulation_types::RobotState initial_state;
  initial_state.joint_state.name = kJointNames;
  initial_state.joint_state.position = Eigen::VectorXd::Zero(kJointNames.size());
  initial_state.joint_state.velocity = Eigen::VectorXd::Zero(kJointNames.size());
  initial_state.multi_dof_joint_state.names = kBaseNames;
  initial_state.multi_dof_joint_state.poses = {Eigen::Affine3d::Identity()};
  initial_state.multi_dof_joint_state.twist = {tmc_manipulation_types::Twist::Zero()};
  return initial_state;
}

std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> GenerateLinkConstraints(
    const Eigen::Affine3d& origin_to_end) {
  tmc_manipulation_types::TaskSpaceRegion tsr;
  tsr.origin_frame_id = "odom";
  tsr.end_frame_id = kEndFrame;
  tsr.origin_to_tsr = origin_to_end;
  tsr.tsr_to_end = Eigen::Affine3d::Identity();
  tsr.min_bounds = tmc_manipulation_types::RegionValues::Zero();
  tsr.max_bounds = tmc_manipulation_types::RegionValues::Zero();
  // Since priority is not particularly used, just put 0
  return {std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0)};
}

std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> GenerateLinkConstraints(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const tmc_manipulation_types::RobotState& state) {
  return GenerateLinkConstraints(CalculateEndFramePose(robot, state));
}

tmc_manipulation_types::RobotState CalculateStateFromInitial(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& constraints) {
  auto sample_goal_generator = std::make_shared<tmc_simple_path_generator::SampleGoalGenerator>();
  sample_goal_generator->Initialize(node);
  sample_goal_generator->set_is_from_random_initial_state(false);

  auto params = std::make_shared<tmc_simple_path_generator::SamplingParameters>();
  params->joint_names = kJointNames;
  params->joint_weights.resize(kJointNames.size());
  params->joint_weights = Eigen::Map<Eigen::VectorXd>(const_cast<double*>(kJointWeights.data()), kJointWeights.size());
  params->base_names = {"world_joint"};
  params->base_weights = Eigen::Map<Eigen::VectorXd>(const_cast<double*>(kBaseWeights.data()), kBaseWeights.size());
  params->joint_limit_upper.resize(kJointNames.size(), 3.0);
  params->joint_limit_lower.resize(kJointNames.size(), -3.0);
  params->ik_joints[kEndFrame] = kJointNames;
  params->SetDefaultBaseMovementType(tmc_manipulation_types::kPlanar);
  params->UpdateBaseMovementType(true);

  tmc_manipulation_types::RobotState goal_state_from_initial;
  sample_goal_generator->SampleFromLinkConstraints(GenerateInitialState(), constraints, params,
                                                   goal_state_from_initial);
  return goal_state_from_initial;
}

tmc_manipulation_types::RobotState CalculateStateFromInitial(
    const rclcpp::Node::SharedPtr& node, const Eigen::Affine3d& origin_to_end) {
  return CalculateStateFromInitial(node, GenerateLinkConstraints(origin_to_end));
}


void AddHardLinkConstraints(const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
                            tmc_robot_local_planner::Constraints& constraints_out) {
  auto goal_state = GenerateInitialState();
  goal_state.joint_state.position[2] = 0.3;
  goal_state.joint_state.position[3] = 0.4;
  goal_state.multi_dof_joint_state.poses[0].translation().x() = 0.5;
  constraints_out.hard_link_constraints = GenerateLinkConstraints(robot, goal_state);
}

void AddGoalRelativeLinearConstraint(tmc_robot_local_planner::Constraints& constraints_out) {
  constraints_out.goal_relative_linear_constraint.end_frame_id = kEndFrame;
  constraints_out.goal_relative_linear_constraint.axis = -Eigen::Vector3d::UnitX();
  constraints_out.goal_relative_linear_constraint.distance = 0.02;
}

void AddHardPathLinkConstraints(tmc_robot_local_planner::Constraints& constraints_out) {
  tmc_manipulation_types::TaskSpaceRegion tsr;
  tsr.origin_frame_id = "odom";
  tsr.end_frame_id = kEndFrame;
  tsr.origin_to_tsr = Eigen::Affine3d::Identity();
  tsr.tsr_to_end = Eigen::Affine3d::Identity();
  // Constrained on a horizontal plane at a height of 1.0m
  tsr.min_bounds << -1.0e10, -1.0e10, 1.0, -M_PI, -M_PI, -M_PI;
  tsr.max_bounds << 1.0e10, 1.0e10, 1.0, M_PI, M_PI, M_PI;
  constraints_out.hard_path_link_constraints = {std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0)};
}

void AddInitialMotionLinearConstraint(const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
                                      tmc_robot_local_planner::Constraints& constraints_out) {
  constraints_out.initial_motion_linear_constraint.end_frame_id = kEndFrame;
  constraints_out.initial_motion_linear_constraint.reference = CalculateEndFramePose(robot, GenerateInitialState());
  constraints_out.initial_motion_linear_constraint.axis = Eigen::Vector3d::UnitZ();
  constraints_out.initial_motion_linear_constraint.distance = 0.02;
}


Eigen::VectorXd Serialize(const tmc_manipulation_types::TimedRobotTrajectory& trajectory, uint32_t point_index) {
  Eigen::VectorXd serialized_state(trajectory.joint_trajectory.joint_names.size() + 3);
  serialized_state.head(trajectory.joint_trajectory.joint_names.size()) =
      trajectory.joint_trajectory.points[point_index].positions;
  serialized_state.tail(3)[0] =
      trajectory.multi_dof_joint_trajectory.points[point_index].transforms[0].translation().x();
  serialized_state.tail(3)[1] =
      trajectory.multi_dof_joint_trajectory.points[point_index].transforms[0].translation().y();
  serialized_state.tail(3)[2] =
      trajectory.multi_dof_joint_trajectory.points[point_index].transforms[0].linear().eulerAngles(0, 1, 2)[2];
  return serialized_state;
}

Eigen::VectorXd Serialize(const tmc_manipulation_types::RobotState& state) {
  Eigen::VectorXd serialized_state(state.joint_state.position.size() + 3);
  serialized_state.head(state.joint_state.position.size()) = state.joint_state.position;
  serialized_state.tail(3)[0] = state.multi_dof_joint_state.poses[0].translation().x();
  serialized_state.tail(3)[1] = state.multi_dof_joint_state.poses[0].translation().y();
  serialized_state.tail(3)[2] = state.multi_dof_joint_state.poses[0].linear().eulerAngles(0, 1, 2)[2];
  return serialized_state;
}


// Extract trajectories with N points
std::vector<tmc_manipulation_types::TimedRobotTrajectory> ExtractTrajectories(
    const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
    uint32_t target_points_num) {
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> extracted_trajectories;
  for (const auto& trajectory : trajectories) {
    if (trajectory.joint_trajectory.points.size() == target_points_num) {
      extracted_trajectories.push_back(trajectory);
    }
  }
  return extracted_trajectories;
}

// Extract trajectories with matching poses
std::vector<tmc_manipulation_types::TimedRobotTrajectory> ExtractTrajectories(
    const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
    const tmc_manipulation_types::RobotState& target_state,
    uint32_t point_index,
    bool positive) {
  const Eigen::VectorXd target_state_serialized = Serialize(target_state);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> extracted_trajectories;
  for (const auto& trajectory : trajectories) {
    const Eigen::VectorXd point_state_serialized = Serialize(trajectory, point_index);
    if ((target_state_serialized - point_state_serialized).isZero() == positive) {
      extracted_trajectories.push_back(trajectory);
    }
  }
  return extracted_trajectories;
}

// Extract trajectories with matching final poses
std::vector<tmc_manipulation_types::TimedRobotTrajectory> ExtractTrajectories(
    const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
    const tmc_manipulation_types::RobotState& goal_state,
    bool positive) {
  // When this function is called, it is assumed that the trajectory group is not zero and all trajectories have the same number of points
  return ExtractTrajectories(trajectories, goal_state, trajectories[0].joint_trajectory.points.size() - 1, positive);
}


void AppendState(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
                 uint32_t target_point_index,
                 std::vector<Eigen::VectorXd>& middle_states_out) {
  for (const auto& trajectory : trajectories) {
    middle_states_out.push_back(Serialize(trajectory, target_point_index));
  }
}

void TestRandomStates(const std::vector<Eigen::VectorXd>& middle_states) {
  // Used to determine randomness, where the increase and decrease in values are roughly equal
  uint32_t total_num = 0;
  uint32_t positive_num = 0;
  for (auto i = 0; i < middle_states.size() - 1; ++i) {
    const Eigen::VectorXd current_to_next = middle_states[i + 1] - middle_states[i];
    for (auto j = 0; j < current_to_next.size(); ++j) {
      ++total_num;
      if (current_to_next[j] > 0) {
        ++positive_num;
      }
    }
  }

  // Falls within the 99% confidence interval
  // At 95%, the test fails about once every 20 times, so it was relaxed
  constexpr double kZ = 2.58;
  EXPECT_GT(static_cast<double>(positive_num), static_cast<double>(total_num) / 2.0 - kZ * std::sqrt(total_num) / 2);
  EXPECT_LT(static_cast<double>(positive_num), static_cast<double>(total_num) / 2.0 + kZ * std::sqrt(total_num) / 2);
}

void TestSameStates(const std::vector<Eigen::VectorXd>& middle_states) {
  for (auto i = 0; i < middle_states.size() - 1; ++i) {
    EXPECT_TRUE((middle_states[i] - middle_states[i + 1]).isZero());
  }
}

void TestEndEffectorPose(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                         uint32_t target_point_index,
                         const Eigen::Affine3d& origin_to_goal,
                         const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot) {
    const auto origin_to_end = CalculateEndFramePose(robot, trajectory, target_point_index);
    const auto end_to_goal = origin_to_end.inverse() * origin_to_goal;
    EXPECT_NEAR(end_to_goal.translation().norm(), 0.0, kEpsilon);
    EXPECT_NEAR(end_to_goal.linear().trace(), 3.0, kEpsilon);
}

void TestGoalStates(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
                    const Eigen::Affine3d& origin_to_goal,
                    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot) {
  for (const auto& trajectory : trajectories) {
    if (trajectory.joint_trajectory.points.size() < 2) {
      ADD_FAILURE();
      return;
    }
    TestEndEffectorPose(trajectory, trajectory.joint_trajectory.points.size() - 1, origin_to_goal, robot);
  }
}

void TestGoalRelativeLinearConstraint(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
                                      const Eigen::Affine3d& origin_to_goal,
                                      const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot) {
  SCOPED_TRACE("GoalRelativeLinearConstraint");
  for (const auto& trajectory : trajectories) {
    if (trajectory.joint_trajectory.points.size() < 3) {
      ADD_FAILURE();
      return;
    }
    TestEndEffectorPose(trajectory, trajectory.joint_trajectory.points.size() - 2,
                        origin_to_goal * Eigen::Translation3d(-0.02, 0.0, 0.0), robot);
  }
}

void TestHardPathLinkConstraint(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
                                const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
                                const uint32_t initial_state_offset = 0,
                                const uint32_t goal_state_offset = 0) {
  for (const auto& trajectory : trajectories) {
    if (trajectory.joint_trajectory.points.size() < 4) {
      ADD_FAILURE();
      return;
    }
    for (auto i = 1 + initial_state_offset;
         i < trajectory.joint_trajectory.points.size() - 1 - goal_state_offset; ++i) {
      const Eigen::Affine3d origin_to_end = CalculateEndFramePose(robot, trajectory, i);
      EXPECT_NEAR(origin_to_end.translation().z(), 1.0, kEpsilon);
    }
  }
}

void TestInitialMotionLinearConstraint(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories,
                                       const tmc_manipulation_types::RobotState& initial_motion_linear_constraint_state,
                                       const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot) {
  SCOPED_TRACE("InitialMotionLinearConstraint");
  const auto origin_to_reference = CalculateEndFramePose(robot, initial_motion_linear_constraint_state);
  for (const auto& trajectory : trajectories) {
    if (trajectory.joint_trajectory.points.size() < 2) {
      ADD_FAILURE();
      return;
    }
    TestEndEffectorPose(trajectory, 1, origin_to_reference, robot);
  }
}

}  // namespace

namespace tmc_simple_path_generator {

class SimplePathGeneratorConstraintsTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  std::shared_ptr<tmc_robot_local_planner::IGenerator> generator_;
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;

  tmc_robot_local_planner::Constraints constraints_;

  tmc_manipulation_types::RobotState initial_state_;
  std::function<bool()> interrupt_func_;

  Eigen::Affine3d origin_to_goal_;
  tmc_manipulation_types::RobotState goal_state_from_initial_;
  tmc_manipulation_types::RobotState initial_motion_linear_constraint_state_;
};

void SimplePathGeneratorConstraintsTest::SetUp() {
  auto node = rclcpp::Node::make_shared("generator");
  DeclareTestParameters(node);

  generator_ = std::make_shared<SimplePathGeneratorPlugin>();
  generator_->Initialize(node);

  robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(
      tmc_manipulation_tests::stanford_manipulator::GetUrdf());

  AddHardLinkConstraints(robot_, constraints_);

  initial_state_ = GenerateInitialState();
  interrupt_func_ = []() -> bool { return false; };

  goal_state_from_initial_ = CalculateStateFromInitial(node, constraints_.hard_link_constraints);
  origin_to_goal_ = CalculateEndFramePose(robot_, goal_state_from_initial_);
  initial_motion_linear_constraint_state_ = CalculateStateFromInitial(
      node, CalculateEndFramePose(robot_, GenerateInitialState()) * Eigen::Translation3d(0.0, 0.0, 0.02));
}

TEST_F(SimplePathGeneratorConstraintsTest, NoOptionalConstraints) {
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  // The number of generated trajectories is at least max_trajectory_num and less than max_trajectory_num + generation_thread_num
  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);

  std::vector<Eigen::VectorXd> middle_states;
  std::vector<Eigen::VectorXd> goal_states_from_initial;
  std::vector<Eigen::VectorXd> goal_states_random;
  // From initial value IK/without intermediate pose 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 2);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, goal_states_from_initial);
  }
  // Random IK/without intermediate pose max_simple_trajectory_num - 1 trajectories
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 2);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    EXPECT_EQ(target_trajectories.size(), kMaxSimpleTrajectoryNum - 1);

    AppendState(target_trajectories, 1, goal_states_random);
  }
  // From initial value IK/with intermediate pose 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, middle_states);
    AppendState(target_trajectories, 2, goal_states_from_initial);
  }
  // Random IK/with intermediate pose remaining
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    EXPECT_EQ(target_trajectories.size(), trajectories_out.size() - kMaxSimpleTrajectoryNum - 1);

    AppendState(target_trajectories, 1, middle_states);
    AppendState(target_trajectories, 2, goal_states_random);
  }

  TestRandomStates(middle_states);
  TestSameStates(goal_states_from_initial);
  TestRandomStates(goal_states_random);
}

TEST_F(SimplePathGeneratorConstraintsTest, WithGoalRelativeLinearConstraint) {
  AddGoalRelativeLinearConstraint(constraints_);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);
  TestGoalRelativeLinearConstraint(trajectories_out, origin_to_goal_, robot_);

  std::vector<Eigen::VectorXd> middle_states;
  std::vector<Eigen::VectorXd> goal_relative_states_from_initial;
  std::vector<Eigen::VectorXd> goal_relative_states_random;
  std::vector<Eigen::VectorXd> goal_states_from_initial;
  std::vector<Eigen::VectorXd> goal_states_random;
  // From initial value IK/without intermediate pose 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, goal_relative_states_from_initial);
    AppendState(target_trajectories, 2, goal_states_from_initial);
  }
  // Random IK/without intermediate pose max_simple_trajectory_num - 1 trajectories
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    EXPECT_EQ(target_trajectories.size(), kMaxSimpleTrajectoryNum - 1);

    AppendState(target_trajectories, 1, goal_relative_states_random);
    AppendState(target_trajectories, 2, goal_states_random);
  }
  // From initial value IK/with intermediate pose 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 4);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, middle_states);
    AppendState(target_trajectories, 2, goal_relative_states_from_initial);
    AppendState(target_trajectories, 3, goal_states_from_initial);
  }
  // Random IK/with intermediate pose remaining
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 4);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    EXPECT_EQ(target_trajectories.size(), trajectories_out.size() - kMaxSimpleTrajectoryNum - 1);

    AppendState(target_trajectories, 1, middle_states);
    AppendState(target_trajectories, 2, goal_relative_states_random);
    AppendState(target_trajectories, 3, goal_states_random);
  }

  TestRandomStates(middle_states);
  TestSameStates(goal_relative_states_from_initial);
  TestRandomStates(goal_relative_states_random);
  TestSameStates(goal_states_from_initial);
  TestRandomStates(goal_states_random);
}

TEST_F(SimplePathGeneratorConstraintsTest, WithHardPathLinkConstraint) {
  AddHardPathLinkConstraints(constraints_);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);
  TestHardPathLinkConstraint(trajectories_out, robot_);

  std::vector<Eigen::VectorXd> initial_constrainted_states;
  std::vector<Eigen::VectorXd> middle_states;
  std::vector<Eigen::VectorXd> goal_constrainted_states_from_initial;
  std::vector<Eigen::VectorXd> goal_constrainted_states_random;
  std::vector<Eigen::VectorXd> goal_states_from_initial;
  std::vector<Eigen::VectorXd> goal_states_random;
  // From initial value IK/without intermediate pose 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 4);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_constrainted_states);
    AppendState(target_trajectories, 2, goal_constrainted_states_from_initial);
    AppendState(target_trajectories, 3, goal_states_from_initial);
  }
  // Random IK/without intermediate pose max_simple_trajectory_num - 1 trajectories
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 4);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    EXPECT_EQ(target_trajectories.size(), kMaxSimpleTrajectoryNum - 1);

    AppendState(target_trajectories, 1, initial_constrainted_states);
    AppendState(target_trajectories, 2, goal_constrainted_states_random);
    AppendState(target_trajectories, 3, goal_states_random);
  }
  // From initial value IK/with intermediate pose 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 5);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_constrainted_states);
    AppendState(target_trajectories, 2, middle_states);
    AppendState(target_trajectories, 3, goal_constrainted_states_from_initial);
    AppendState(target_trajectories, 4, goal_states_from_initial);
  }
  // Random IK/with intermediate pose remaining
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 5);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    EXPECT_EQ(target_trajectories.size(), trajectories_out.size() - kMaxSimpleTrajectoryNum - 1);

    AppendState(target_trajectories, 1, initial_constrainted_states);
    AppendState(target_trajectories, 2, middle_states);
    AppendState(target_trajectories, 3, goal_constrainted_states_random);
    AppendState(target_trajectories, 4, goal_states_random);
  }

  TestSameStates(initial_constrainted_states);
  TestRandomStates(middle_states);
  TestSameStates(goal_constrainted_states_from_initial);
  TestRandomStates(goal_constrainted_states_random);
  TestSameStates(goal_states_from_initial);
  TestRandomStates(goal_states_random);
}

TEST_F(SimplePathGeneratorConstraintsTest, WithInitialMotionLinearConstraint) {
  AddInitialMotionLinearConstraint(robot_, constraints_);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);
  TestInitialMotionLinearConstraint(trajectories_out, initial_motion_linear_constraint_state_, robot_);

  std::vector<Eigen::VectorXd> initial_relative_states_from_initial;
  std::vector<Eigen::VectorXd> initial_relative_states_random;
  std::vector<Eigen::VectorXd> middle_states;
  std::vector<Eigen::VectorXd> goal_states_from_initial;
  std::vector<Eigen::VectorXd> goal_states_random;
  // From initial value IK/without intermediate pose/straight constraint from initial value 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_from_initial);
    AppendState(target_trajectories, 2, goal_states_from_initial);
  }
  // Random IK/without intermediate pose/straight constraint from initial value (max_simple_trajectory_num - 2) / 2 trajectories
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, true);
    EXPECT_EQ(target_trajectories.size(), (kMaxSimpleTrajectoryNum - 2) / 2);

    AppendState(target_trajectories, 1, initial_relative_states_from_initial);
    AppendState(target_trajectories, 2, goal_states_random);
  }
  // From initial value IK/without intermediate pose/straight constraint from exit 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, goal_states_from_initial);
  }
  // Random IK/without intermediate pose/straight constraint from exit (max_simple_trajectory_num - 2) / 2 trajectories
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 3);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), (kMaxSimpleTrajectoryNum - 2) / 2);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, goal_states_random);
  }
  // From initial value IK/with intermediate pose/straight constraint from initial value 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 4);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_from_initial);
    AppendState(target_trajectories, 2, middle_states);
    AppendState(target_trajectories, 3, goal_states_from_initial);
  }
  // From initial value IK/with intermediate pose/straight constraint from exit 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 4);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, middle_states);
    AppendState(target_trajectories, 3, goal_states_from_initial);
  }
  // Random IK/with intermediate pose/straight constraint from initial value 0 trajectories
  // Random IK/with intermediate pose/straight constraint from exit remaining
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 4);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), trajectories_out.size() - kMaxSimpleTrajectoryNum - 2);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, middle_states);
    AppendState(target_trajectories, 3, goal_states_random);
  }

  TestSameStates(initial_relative_states_from_initial);
  TestRandomStates(initial_relative_states_random);
  TestRandomStates(middle_states);
  TestSameStates(goal_states_from_initial);
  TestRandomStates(goal_states_random);
}

// Testing all combinations of presence and absence is difficult, so only check if conditions are met

TEST_F(SimplePathGeneratorConstraintsTest, WithGoalRelativeLinearAndHardPathLink) {
  AddGoalRelativeLinearConstraint(constraints_);
  AddHardPathLinkConstraints(constraints_);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);
  TestGoalRelativeLinearConstraint(trajectories_out, origin_to_goal_, robot_);
  TestHardPathLinkConstraint(trajectories_out, robot_, 0, 1);
}

TEST_F(SimplePathGeneratorConstraintsTest, WithGoalRelativeLinearAndInitialMotionLinear) {
  AddGoalRelativeLinearConstraint(constraints_);
  AddInitialMotionLinearConstraint(robot_, constraints_);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);
  TestGoalRelativeLinearConstraint(trajectories_out, origin_to_goal_, robot_);
  TestInitialMotionLinearConstraint(trajectories_out, initial_motion_linear_constraint_state_, robot_);
}

TEST_F(SimplePathGeneratorConstraintsTest, WithHardPathLinkAndInitialMotionLinear) {
  AddHardPathLinkConstraints(constraints_);
  AddInitialMotionLinearConstraint(robot_, constraints_);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);
  TestHardPathLinkConstraint(trajectories_out, robot_, 1, 0);
  TestInitialMotionLinearConstraint(trajectories_out, initial_motion_linear_constraint_state_, robot_);
}

TEST_F(SimplePathGeneratorConstraintsTest, WithAllConstrints) {
  AddGoalRelativeLinearConstraint(constraints_);
  AddHardPathLinkConstraints(constraints_);
  AddInitialMotionLinearConstraint(robot_, constraints_);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  EXPECT_TRUE(generator_->Generate(constraints_, initial_state_, 0.2, true, interrupt_func_, trajectories_out));

  EXPECT_GE(trajectories_out.size(), kMaxTrajectoryNum);
  EXPECT_LT(trajectories_out.size(), kMaxTrajectoryNum + kGenerationThreadNum);

  TestGoalStates(trajectories_out, origin_to_goal_, robot_);
  TestGoalRelativeLinearConstraint(trajectories_out, origin_to_goal_, robot_);
  TestHardPathLinkConstraint(trajectories_out, robot_, 1, 1);
  TestInitialMotionLinearConstraint(trajectories_out, initial_motion_linear_constraint_state_, robot_);

  std::vector<Eigen::VectorXd> initial_relative_states_from_initial;
  std::vector<Eigen::VectorXd> initial_relative_states_random;
  std::vector<Eigen::VectorXd> initial_constrainted_states_from_initial;
  std::vector<Eigen::VectorXd> initial_constrainted_states_random;
  std::vector<Eigen::VectorXd> middle_states;
  std::vector<Eigen::VectorXd> goal_constrainted_states_from_initial;
  std::vector<Eigen::VectorXd> goal_constrainted_states_random;
  std::vector<Eigen::VectorXd> goal_relative_states_from_initial;
  std::vector<Eigen::VectorXd> goal_relative_states_random;
  std::vector<Eigen::VectorXd> goal_states_from_initial;
  std::vector<Eigen::VectorXd> goal_states_random;
  // From initial value IK/without intermediate pose/straight constraint from initial value 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 6);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_from_initial);
    AppendState(target_trajectories, 2, initial_constrainted_states_from_initial);
    AppendState(target_trajectories, 3, goal_constrainted_states_from_initial);
    AppendState(target_trajectories, 4, goal_relative_states_from_initial);
    AppendState(target_trajectories, 5, goal_states_from_initial);
  }
  // Random IK/without intermediate pose/straight constraint from initial value (max_simple_trajectory_num - 2) / 2 trajectories
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 6);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, true);
    EXPECT_EQ(target_trajectories.size(), (kMaxSimpleTrajectoryNum - 2) / 2);

    AppendState(target_trajectories, 1, initial_relative_states_from_initial);
    AppendState(target_trajectories, 2, initial_constrainted_states_from_initial);
    AppendState(target_trajectories, 3, goal_constrainted_states_random);
    AppendState(target_trajectories, 4, goal_relative_states_random);
    AppendState(target_trajectories, 5, goal_states_random);
  }
  // From initial value IK/without intermediate pose/straight constraint from exit 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 6);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, initial_constrainted_states_random);
    AppendState(target_trajectories, 3, goal_constrainted_states_from_initial);
    AppendState(target_trajectories, 4, goal_relative_states_from_initial);
    AppendState(target_trajectories, 5, goal_states_from_initial);
  }
  // Random IK/without intermediate pose/straight constraint from exit (max_simple_trajectory_num - 2) / 2 trajectories
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 6);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), (kMaxSimpleTrajectoryNum - 2) / 2);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, initial_constrainted_states_random);
    AppendState(target_trajectories, 3, goal_constrainted_states_random);
    AppendState(target_trajectories, 4, goal_relative_states_random);
    AppendState(target_trajectories, 5, goal_states_random);
  }
  // From initial value IK/with intermediate pose/straight constraint from initial value 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 7);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, true);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_from_initial);
    AppendState(target_trajectories, 2, initial_constrainted_states_from_initial);
    AppendState(target_trajectories, 3, middle_states);
    AppendState(target_trajectories, 4, goal_constrainted_states_from_initial);
    AppendState(target_trajectories, 5, goal_relative_states_from_initial);
    AppendState(target_trajectories, 6, goal_states_from_initial);
  }
  // From initial value IK/with intermediate pose/straight constraint from exit 1 trajectory
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 7);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, true);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), 1);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, initial_constrainted_states_random);
    AppendState(target_trajectories, 3, middle_states);
    AppendState(target_trajectories, 4, goal_constrainted_states_from_initial);
    AppendState(target_trajectories, 5, goal_relative_states_from_initial);
    AppendState(target_trajectories, 6, goal_states_from_initial);
  }
  // Random IK/with intermediate pose/straight constraint from initial value 0 trajectories
  // Random IK/with intermediate pose/straight constraint from exit remaining
  {
    auto target_trajectories = ExtractTrajectories(trajectories_out, 7);
    target_trajectories = ExtractTrajectories(target_trajectories, goal_state_from_initial_, false);
    target_trajectories = ExtractTrajectories(target_trajectories, initial_motion_linear_constraint_state_, 1, false);
    EXPECT_EQ(target_trajectories.size(), trajectories_out.size() - kMaxSimpleTrajectoryNum - 2);

    AppendState(target_trajectories, 1, initial_relative_states_random);
    AppendState(target_trajectories, 2, initial_constrainted_states_random);
    AppendState(target_trajectories, 3, middle_states);
    AppendState(target_trajectories, 4, goal_constrainted_states_random);
    AppendState(target_trajectories, 5, goal_relative_states_random);
    AppendState(target_trajectories, 6, goal_states_random);
  }

  TestSameStates(initial_relative_states_from_initial);
  TestRandomStates(initial_relative_states_random);
  TestSameStates(initial_constrainted_states_from_initial);
  TestRandomStates(initial_constrainted_states_random);
  TestRandomStates(middle_states);
  TestSameStates(goal_constrainted_states_from_initial);
  TestRandomStates(goal_constrainted_states_random);
  TestSameStates(goal_relative_states_from_initial);
  TestRandomStates(goal_relative_states_random);
  TestSameStates(goal_states_from_initial);
  TestRandomStates(goal_states_random);
}

}  // namespace tmc_simple_path_generator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
