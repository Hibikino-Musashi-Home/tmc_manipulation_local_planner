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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <tmc_simple_path_generator/hard_path_constraints.hpp>

namespace {

const std::vector<std::string> kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
const std::vector<std::string> kBaseNames = {"world_joint"};
const char* const kEndFrame = "link7";

constexpr double kEpsilon = 1.0e-3;
constexpr double kLinearStep = 0.01;

tmc_manipulation_types::RobotState GenerateDefaultState(double x = 0.0, double y = 0.0) {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = kJointNames;
  state.joint_state.position = Eigen::VectorXd::Zero(6);
  state.multi_dof_joint_state.names = kBaseNames;
  state.multi_dof_joint_state.poses = {Eigen::Translation3d(x, y, 0.0) * Eigen::AngleAxisd::Identity()};
  return state;
}

tmc_simple_path_generator::SamplingParameters::Ptr GenerateSamplingParameters() {
  auto params = std::make_shared<tmc_simple_path_generator::SamplingParameters>();
  params->joint_names = kJointNames;
  params->joint_weights.resize(kJointNames.size());
  params->joint_weights << 5.0, 1.0, 0.5, 0.5, 0.5, 1.0;
  params->base_names = {"world_joint"};
  params->base_weights = Eigen::Vector3d(5.0, 5.0, 1.0);
  params->joint_limit_upper.resize(kJointNames.size(), 3.0);
  params->joint_limit_lower.resize(kJointNames.size(), -3.0);
  params->ik_joints[kEndFrame] = kJointNames;
  params->SetDefaultBaseMovementType(tmc_manipulation_types::kPlanar);
  params->UpdateBaseMovementType(true);
  return params;
}

Eigen::VectorXd Serialize(const tmc_manipulation_types::RobotState& state) {
  Eigen::VectorXd serialized_state(state.joint_state.position.size() + 3);
  serialized_state.head(state.joint_state.position.size()) = state.joint_state.position;
  serialized_state.tail(3)[0] = state.multi_dof_joint_state.poses[0].translation().x();
  serialized_state.tail(3)[1] = state.multi_dof_joint_state.poses[0].translation().y();
  serialized_state.tail(3)[2] = state.multi_dof_joint_state.poses[0].linear().eulerAngles(0, 1, 2)[2];
  return serialized_state;
}

bool IsMonotonically(const tmc_manipulation_types::RobotState& initial_state,
                     const tmc_manipulation_types::RobotState& end_state,
                     const tmc_manipulation_types::RobotState& current_state) {
  const Eigen::VectorXd initial_to_current = Serialize(current_state) - Serialize(initial_state);
  const Eigen::VectorXd current_to_end = Serialize(end_state) - Serialize(current_state);
  for (auto i = 0u; i < initial_to_current.size(); ++i) {
    if (initial_to_current[i] * current_to_end[i] < 0.0) {
      return false;
    }
  }
  return true;
}

bool IsMonotonically(const tmc_manipulation_types::RobotState& initial_state,
                     const tmc_manipulation_types::RobotState& end_state,
                     const std::vector<tmc_manipulation_types::RobotState>& path) {
  for (const auto& current_state : path) {
    if (!IsMonotonically(initial_state, end_state, current_state)) {
      return false;
    }
  }
  return true;
}

bool IsEqual(const tmc_manipulation_types::RobotState& state_1,
             const tmc_manipulation_types::RobotState& state_2) {
  if (state_1.joint_state.position.size() != state_2.joint_state.position.size()) {
    return false;
  }
  const auto serialized_state_1 = Serialize(state_1);
  const auto serialized_state_2 = Serialize(state_2);
  for (auto i = 0u; i < serialized_state_1.size(); ++i) {
    if (std::abs(serialized_state_1[i] - serialized_state_2[i]) > kEpsilon) {
      return false;
    }
  }
  return true;
}

bool IsEqual(const std::vector<tmc_manipulation_types::RobotState>& path_1,
             const std::vector<tmc_manipulation_types::RobotState>& path_2) {
  if (path_1.size() != path_2.size()) {
    return false;
  }
  for (auto i = 0u; i < path_1.size(); ++i) {
    if (!IsEqual(path_1[i], path_2[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace tmc_simple_path_generator {

class InitialMotionLinearConstraintTest : public ::testing::Test {
 protected:
  void SetUp() override;

  InitialMotionLinearConstraint::Ptr path_constraints_;
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;

  Eigen::Affine3d CalculateEndFramePose(const tmc_manipulation_types::RobotState& state);
};

void InitialMotionLinearConstraintTest::SetUp() {
  const auto robot_description = tmc_manipulation_tests::stanford_manipulator::GetUrdf();
  robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);

  auto node = rclcpp::Node::make_shared("test_node");
  node->declare_parameter<std::string>("robot_description_kinematics", robot_description);
  node->declare_parameter<double>("linear_constraint_step", kLinearStep);

  path_constraints_ = std::make_shared<InitialMotionLinearConstraint>(node);
}

Eigen::Affine3d InitialMotionLinearConstraintTest::CalculateEndFramePose(
    const tmc_manipulation_types::RobotState& state) {
  robot_->SetRobotTransform(state.multi_dof_joint_state.poses[0]);
  robot_->SetNamedAngle(state.joint_state);
  return robot_->GetObjectTransform(kEndFrame);
}

TEST_F(InitialMotionLinearConstraintTest, SampleLinearPathFromInitial) {
  const auto initial_state = GenerateDefaultState();

  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.reference = CalculateEndFramePose(initial_state);
  linear_constraint.axis = Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, GenerateSamplingParameters());
  ASSERT_TRUE(linear_path.has_value());

  // It is possible to become +1 due to decimal calculation errors
  const uint32_t expected_size = static_cast<uint32_t>(linear_constraint.distance / kLinearStep);
  EXPECT_GE(linear_path.value().size(), expected_size);
  EXPECT_LE(linear_path.value().size(), expected_size + 1);

  double prev_distance = 0.0;
  for (const auto& way_point : linear_path.value()) {
    const auto origin_to_way_point = CalculateEndFramePose(way_point);
    const auto displacement = origin_to_way_point.translation() - linear_constraint.reference.translation();
    // Since there is a range for IK convergence determination, exact matching cannot be expected
    EXPECT_NEAR(displacement.normalized().dot(linear_constraint.axis), 1.0, kEpsilon);

    const auto distance = displacement.norm();
    EXPECT_NEAR(distance - prev_distance, kLinearStep, kEpsilon);
    prev_distance = distance;
  }
  // Since IK is solved sequentially from the initial posture, it should be monotonically increasing
  IsMonotonically(initial_state, linear_path.value().back(), linear_path.value());

  // There is no randomness
  const auto linear_path_2 = path_constraints_->SampleLinearPath(
        linear_constraint, initial_state, GenerateSamplingParameters());
  ASSERT_TRUE(linear_path_2.has_value());
  EXPECT_TRUE(IsEqual(linear_path.value(), linear_path_2.value()));
}

TEST_F(InitialMotionLinearConstraintTest, SampleRandomLinearPath) {
  const auto initial_state = GenerateDefaultState();

  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.reference = CalculateEndFramePose(initial_state);
  linear_constraint.axis = Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  // Check if the linear constraint seems to be effective
  path_constraints_->set_is_from_random_state(true);
  {
    const auto random_linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, initial_state, GenerateSamplingParameters());
    ASSERT_TRUE(random_linear_path.has_value());

    // It is possible to become +1 due to decimal calculation errors
    const uint32_t expected_size = static_cast<uint32_t>(linear_constraint.distance / kLinearStep);
    EXPECT_GE(random_linear_path.value().size(), expected_size);
    EXPECT_LE(random_linear_path.value().size(), expected_size + 1);

    double prev_distance = 0.0;
    for (const auto& way_point : random_linear_path.value()) {
      const auto origin_to_way_point = CalculateEndFramePose(way_point);
      const auto displacement = origin_to_way_point.translation() - linear_constraint.reference.translation();
      // Since there is a range for IK convergence determination, exact matching cannot be expected
      EXPECT_NEAR(displacement.normalized().dot(linear_constraint.axis), 1.0, kEpsilon);

      const auto distance = displacement.norm();
      EXPECT_NEAR(distance - prev_distance, kLinearStep, kEpsilon);
      prev_distance = distance;
    }
  }

  // Check for randomness
  path_constraints_->set_is_from_random_state(false);
  const auto linear_path_from_initial = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, GenerateSamplingParameters());
  ASSERT_TRUE(linear_path_from_initial.has_value());

  bool are_all_equal = true;
  bool are_all_monotonically = true;
  path_constraints_->set_is_from_random_state(true);
  for (auto i = 0; i < 10; ++i) {
    const auto random_linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, initial_state, GenerateSamplingParameters());
    ASSERT_TRUE(random_linear_path.has_value());

    if (IsEqual(linear_path_from_initial.value(), random_linear_path.value())) {
      continue;
    }
    are_all_equal = false;

    if (IsMonotonically(initial_state, random_linear_path.value().back(), random_linear_path.value())) {
      continue;
    }
    are_all_monotonically = false;
    break;
  }
  EXPECT_FALSE(are_all_equal);
  EXPECT_FALSE(are_all_monotonically);
}

TEST_F(InitialMotionLinearConstraintTest, InitialStateOnLinearConstraint) {
  const auto initial_state = GenerateDefaultState();

  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.reference = CalculateEndFramePose(initial_state)
                              * Eigen::Translation3d(0.058 * Eigen::Vector3d::UnitX());
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, GenerateSamplingParameters());
  ASSERT_TRUE(linear_path.has_value());

  // Since it merges from 0.06, there are 5 points up to 0.10
  EXPECT_EQ(linear_path.value().size(), 5);

  double prev_distance = 0.05;
  for (const auto& way_point : linear_path.value()) {
    const auto origin_to_way_point = CalculateEndFramePose(way_point);
    const auto displacement = origin_to_way_point.translation() - linear_constraint.reference.translation();
    EXPECT_NEAR(displacement.normalized().dot(linear_constraint.axis), 1.0, kEpsilon);

    const auto distance = displacement.norm();
    EXPECT_NEAR(distance - prev_distance, kLinearStep, kEpsilon);
    prev_distance = distance;
  }
  // Since IK is solved sequentially from the initial posture, it should be monotonically increasing
  IsMonotonically(initial_state, linear_path.value().back(), linear_path.value());
}


class InitialMotionLinearConstraintBoundaryTest : public InitialMotionLinearConstraintTest,
                                                  public ::testing::WithParamInterface<bool> {};

TEST_P(InitialMotionLinearConstraintBoundaryTest, ConstraintsBoundaryTest) {
  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.reference = CalculateEndFramePose(GenerateDefaultState());
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  path_constraints_->set_is_from_random_state(GetParam());
  {
    // Since it is on the constraint line segment, it is constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-0.05, 0.0), GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    // Since it merges from 0.06, there are 5 points up to 0.10
    EXPECT_EQ(linear_path.value().size(), 5);
  }
  {
    // Since it is close to the constraint line segment, it is constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-0.05, kLinearStep * 0.9), GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    // Since it merges from 0.05, there are 6 points up to 0.10
    EXPECT_EQ(linear_path.value().size(), 6);
  }
  {
    // Since it is far from the constraint line segment, it is not constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-0.05, kLinearStep * 1.1), GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 0);
  }
  {
    // Since it is close to the starting point of the constraint line segment, it is constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(kLinearStep * 0.9, 0.0), GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 10);
  }
  {
    // Since it is close to the starting point of the constraint line segment, it is constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(0.0, kLinearStep * 0.9), GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 10);
  }
  {
    // Since it is far from the starting point of the constraint line segment, it is not constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(kLinearStep * 1.1, 0.0), GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 0);
  }
  {
    // Since it is beyond the endpoint of the constraint line segment, it is not constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-linear_constraint.distance - kLinearStep * 0.9, 0.0),
        GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 0);
  }
  {
    // Since it is the same as the endpoint of the constraint line segment, it is not constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-linear_constraint.distance, 0.0),
        GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 0);
  }
  {
    // Since it is almost the same as the endpoint of the constraint line segment, it is not constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-linear_constraint.distance + 1.0e-6, 0.0),
        GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 0);
  }
  {
    // Since it is on the plane of the endpoint of the constraint line segment, it is not constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-linear_constraint.distance, kLinearStep * 0.9),
        GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 0);
  }
  {
    // Since it is inside the endpoint of the constraint line segment, it is constrained
    const auto linear_path = path_constraints_->SampleLinearPath(
        linear_constraint, GenerateDefaultState(-linear_constraint.distance + kLinearStep * 0.5, kLinearStep * 0.5),
        GenerateSamplingParameters());
    ASSERT_TRUE(linear_path.has_value());
    EXPECT_EQ(linear_path.value().size(), 1);
  }
}
INSTANTIATE_TEST_CASE_P(BoundaryTest,
                        InitialMotionLinearConstraintBoundaryTest,
                        ::testing::Values(true, false));

TEST_F(InitialMotionLinearConstraintTest, LinearConstraintNotUsed) {
  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = "";
  linear_constraint.reference = Eigen::Affine3d::Identity();
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, GenerateDefaultState(), GenerateSamplingParameters());
  // If end_frame_id is empty, it is exceptionally considered successful
  ASSERT_TRUE(linear_path.has_value());
  EXPECT_TRUE(linear_path.value().empty());
}

TEST_F(InitialMotionLinearConstraintTest, InvalidEndFrame) {
  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = "invalid_frame";
  linear_constraint.reference = Eigen::Affine3d::Identity();
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto initial_state = GenerateDefaultState();

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, GenerateDefaultState(), GenerateSamplingParameters());
  EXPECT_FALSE(linear_path.has_value());
}

TEST_F(InitialMotionLinearConstraintTest, ZeroAxis) {
  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.reference = Eigen::Affine3d::Identity();
  linear_constraint.axis = Eigen::Vector3d::Zero();
  linear_constraint.distance = 0.1;

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, GenerateDefaultState(), GenerateSamplingParameters());
  EXPECT_FALSE(linear_path.has_value());
}

TEST_F(InitialMotionLinearConstraintTest, ZeroDistance) {
  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.reference = Eigen::Affine3d::Identity();
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.0;

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, GenerateDefaultState(), GenerateSamplingParameters());
  EXPECT_FALSE(linear_path.has_value());
}

TEST_F(InitialMotionLinearConstraintTest, NegativeDistance) {
  tmc_robot_local_planner::LinearConstraintWithPose linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.reference = Eigen::Affine3d::Identity();
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = -0.1;

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, GenerateDefaultState(), GenerateSamplingParameters());
  EXPECT_FALSE(linear_path.has_value());
}

}  // namespace tmc_simple_path_generator

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
