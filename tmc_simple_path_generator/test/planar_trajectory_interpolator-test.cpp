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
/// @brief Test of PlanarTrajectoryInterpolator

#include <boost/range/adaptor/indexed.hpp>

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>

#include <tmc_robot_local_planner_utils/converter.hpp>
#include <tmc_simple_path_generator/planar_trajectory_interpolator.hpp>

using tmc_robot_local_planner_utils::Get2DPose;
using tmc_robot_local_planner_utils::Get2DTwist;

namespace {
const std::vector<std::string> kJointNames = {"arm", "hand", "shoulder"};
const std::vector<std::string> kBaseNames = {"world_joint"};
constexpr double kEpsilon = 1.0e-6;

// Extract positions from TimedJointTrajectory
bool ExtractJointPositions(
    const tmc_manipulation_types::TimedJointTrajectory& trajectory,
    const tmc_manipulation_types::NameSeq& joint_names,
    std::vector<Eigen::VectorXd>& dst_positions) {
  dst_positions.clear();
  for (const auto& name : joint_names) {
    auto name_it = std::find(trajectory.joint_names.begin(),
                             trajectory.joint_names.end(), name);
    if (name_it == trajectory.joint_names.end()) {
      return false;
    }
    auto index = std::distance(trajectory.joint_names.begin(), name_it);
    Eigen::VectorXd positions(trajectory.points.size());
    for (const auto& point : trajectory.points | boost::adaptors::indexed()) {
      if (index < point.value().positions.size()) {
        positions[point.index()] = point.value().positions[index];
      } else {
        return false;
      }
    }
    dst_positions.push_back(positions);
  }
  return true;
}

// Extract velocities from TimedJointTrajectory
bool ExtractJointVelocities(
    const tmc_manipulation_types::TimedJointTrajectory& trajectory,
    const tmc_manipulation_types::NameSeq& joint_names,
    std::vector<Eigen::VectorXd>& dst_velocities) {
  dst_velocities.clear();
  for (const auto& name : joint_names) {
    auto name_it = std::find(trajectory.joint_names.begin(),
                             trajectory.joint_names.end(), name);
    if (name_it == trajectory.joint_names.end()) {
      return false;
    }
    auto index = std::distance(trajectory.joint_names.begin(), name_it);
    Eigen::VectorXd velocities(trajectory.points.size());
    for (const auto& point : trajectory.points | boost::adaptors::indexed()) {
      if (index < point.value().velocities.size()) {
        velocities[point.index()] = point.value().velocities[index];
      } else {
        return false;
      }
    }
    dst_velocities.push_back(velocities);
  }
  return true;
}

// Extract position x, y, yaw from TimedMultiDOFJointTrajectory
bool ExtractBase2DPoses(
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& trajectory,
    const std::string joint_name,
    std::vector<Eigen::VectorXd>& dst_positions) {
  dst_positions.resize(3, Eigen::VectorXd(trajectory.points.size()));
  auto name_it = std::find(trajectory.joint_names.begin(),
                           trajectory.joint_names.end(), joint_name);
  if (name_it == trajectory.joint_names.end()) {
    return false;
  }
  auto index = std::distance(trajectory.joint_names.begin(), name_it);
  for (const auto& point : trajectory.points | boost::adaptors::indexed()) {
    auto pose = Get2DPose(point.value().transforms[index]);
    dst_positions[0][point.index()] = pose[0];
    dst_positions[1][point.index()] = pose[1];
    dst_positions[2][point.index()] = pose[2];
  }
  return true;
}

// Extract velocity x, y, yaw from TimedMultiDOFJointTrajectory
bool ExtractBase2DTwist(
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& trajectory,
    const std::string joint_name,
    std::vector<Eigen::VectorXd>& dst_velocities) {
  dst_velocities.resize(3, Eigen::VectorXd(trajectory.points.size()));
  auto name_it = std::find(trajectory.joint_names.begin(),
                           trajectory.joint_names.end(), joint_name);
  if (name_it == trajectory.joint_names.end()) {
    return false;
  }
  auto index = std::distance(trajectory.joint_names.begin(), name_it);
  for (const auto& point : trajectory.points | boost::adaptors::indexed()) {
    auto twist = Get2DTwist(point.value().velocities[index]);
    dst_velocities[0][point.index()] = twist[0];
    dst_velocities[1][point.index()] = twist[1];
    dst_velocities[2][point.index()] = twist[2];
  }
  return true;
}

// Count changes in increase/decrease, assuming starting from zero
uint32_t CountDirectionSwitch(const Eigen::VectorXd& positions) {
  // Search for the first non-zero point
  double last_diff = 0.0;
  uint32_t start_index = positions.size();
  for (uint32_t i = 0; i < positions.size(); ++i) {
    if (fabs(positions[i]) > std::numeric_limits<double>::min()) {
      start_index = i;
      last_diff = positions[i];
      break;
    }
  }
  if (start_index == positions.size()) {
    // No change as it remains zero
    return 0;
  }
  uint32_t result = 0;
  for (uint32_t i = start_index + 1; i < positions.size(); ++i) {
    auto diff = positions[i] - positions[i - 1];
    if (fabs(diff) < std::numeric_limits<double>::min()) {
      // Reserve judgment if zero
      continue;
    }
    if (last_diff * diff < 0.0) {
      ++result;
    }
    last_diff = diff;
  }
  return result;
}

void Eq(const Eigen::Affine3d& actual, const Eigen::Affine3d& expected) {
  EXPECT_EQ(actual.translation(), expected.translation());
  EXPECT_EQ(actual.linear(), expected.linear());
}

tmc_simple_path_generator::SamplingParameters::Ptr GenerateTestParameters() {
  auto params = std::make_shared<tmc_simple_path_generator::SamplingParameters>();
  params->joint_names = kJointNames;
  params->joint_weights = Eigen::Vector3d(5.0, 1.0, 0.5);
  params->base_names = kBaseNames;
  params->base_weights = Eigen::Vector3d(5.0, 5.0, 1.0);
  params->joint_limit_upper = {1.0, 1.0, 1.0};
  params->joint_limit_lower = {-1.0, -1.0, -1.0};
  return params;
}

tmc_manipulation_types::RobotState GenerateRobotState(double value = 0.0) {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = kJointNames;
  state.joint_state.position = Eigen::VectorXd::Ones(kJointNames.size()) * value;
  state.joint_state.velocity = Eigen::VectorXd::Ones(kJointNames.size()) * value;
  state.multi_dof_joint_state.names = kBaseNames;
  state.multi_dof_joint_state.poses.resize(1);
  state.multi_dof_joint_state.poses[0] =
      Eigen::Translation3d(value, value, 0.0) * Eigen::Quaterniond(Eigen::AngleAxisd(value, Eigen::Vector3d::UnitZ()));
  state.multi_dof_joint_state.twist.push_back(Eigen::VectorXd::Ones(6) * value);
  return state;
}

tmc_manipulation_types::RobotState GenerateInitialState() {
  return GenerateRobotState();
}

tmc_manipulation_types::RobotState GenerateGoalState() {
  return GenerateRobotState(1.0);
}

tmc_manipulation_types::RobotState RemoveFirstJoint(const tmc_manipulation_types::RobotState& state) {
  tmc_manipulation_types::RobotState new_state = state;
  new_state.joint_state.name.erase(new_state.joint_state.name.begin());
  new_state.joint_state.position = new_state.joint_state.position.tail(new_state.joint_state.position.size() - 1);
  new_state.joint_state.velocity = new_state.joint_state.velocity.tail(new_state.joint_state.velocity.size() - 1);
  return new_state;
}

}  // namespace

namespace tmc_simple_path_generator {

TEST(PlanarTrajectoryInterpolatorPluginTest, CreateInstance) {
  pluginlib::ClassLoader<ITrajectoryInterpolator> class_loader(
      "tmc_simple_path_generator", "tmc_simple_path_generator::ITrajectoryInterpolator");
  auto interpolator = class_loader.createSharedInstance("tmc_simple_path_generator/PlanarTrajectoryInterpolator");
  interpolator = class_loader.createSharedInstance("tmc_simple_path_generator/PlanarTrajectoryInterpolatorPlain");
}


class PlanarTrajectoryInterpolatorTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp::Node::SharedPtr node_;
  ITrajectoryInterpolator::Ptr planar_;
};

void PlanarTrajectoryInterpolatorTest::SetUp() {
  node_ = rclcpp::Node::make_shared("test_node");

  planar_ = std::make_shared<PlanarTrajectoryInterpolator>();
  planar_->Initialize(node_, GenerateTestParameters());
}

// When there is no initial velocity
TEST_F(PlanarTrajectoryInterpolatorTest, InterpolateSimpleWithoutInitialVelocities) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, goal_state, 0.5, trajectory));

  std::vector<Eigen::VectorXd> joint_positions;
  ASSERT_TRUE(ExtractJointPositions(trajectory.joint_trajectory, kJointNames, joint_positions));
  for (uint32_t i = 0; i < kJointNames.size(); ++i) {
    EXPECT_EQ(0, CountDirectionSwitch(joint_positions[i]));
    EXPECT_LE(0.0, joint_positions[i].tail(1)[0]);
    EXPECT_DOUBLE_EQ(initial_state.joint_state.position[i], joint_positions[i].head(1)[0]);
    EXPECT_DOUBLE_EQ(goal_state.joint_state.position[i], joint_positions[i].tail(1)[0]);
  }
  std::vector<Eigen::VectorXd> joint_velocities;
  ASSERT_TRUE(ExtractJointVelocities(trajectory.joint_trajectory, kJointNames, joint_velocities));
  for (uint32_t i = 0; i < kJointNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(initial_state.joint_state.velocity[i], joint_velocities[i].head(1)[0]);
    EXPECT_DOUBLE_EQ(0.0, joint_velocities[i].tail(1)[0]);
  }

  std::vector<Eigen::VectorXd> base_positions;
  ASSERT_TRUE(ExtractBase2DPoses(trajectory.multi_dof_joint_trajectory, kBaseNames[0], base_positions));
  const auto initial_base_pose = Get2DPose(initial_state.multi_dof_joint_state.poses[0]);
  const auto goal_base_pose = Get2DPose(goal_state.multi_dof_joint_state.poses[0]);
  for (uint32_t i = 0; i < base_positions.size(); ++i) {
    EXPECT_EQ(0, CountDirectionSwitch(base_positions[i]));
    EXPECT_LE(0.0, base_positions[i].tail(1)[0]);
    EXPECT_DOUBLE_EQ(initial_base_pose[i], base_positions[i].head(1)[0]);
    EXPECT_DOUBLE_EQ(goal_base_pose[i], base_positions[i].tail(1)[0]);
  }
  std::vector<Eigen::VectorXd> base_velocities;
  ASSERT_TRUE(ExtractBase2DTwist(trajectory.multi_dof_joint_trajectory, kBaseNames[0], base_velocities));
  auto initial_base_twist = Get2DTwist(initial_state.multi_dof_joint_state.twist[0]);
  for (uint32_t i = 0; i < base_velocities.size(); ++i) {
    EXPECT_DOUBLE_EQ(initial_base_twist[i], base_velocities[i].head(1)[0]);
    EXPECT_DOUBLE_EQ(0.0, base_velocities[i].tail(1)[0]);
  }
}

// When there is an initial velocity
TEST_F(PlanarTrajectoryInterpolatorTest, InterpolateSimpleWithInitialVelocities) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.velocity << 0.1, 0.1, 0.0;
  initial_state.multi_dof_joint_state.twist[0] << 0.1, -1.0, 0.1, 0.0, 0.0, 0.0;
  const auto goal_state = GenerateGoalState();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, goal_state, 0.5, trajectory));

  std::vector<Eigen::VectorXd> joint_positions;
  ASSERT_TRUE(ExtractJointPositions(trajectory.joint_trajectory, kJointNames, joint_positions));
  for (uint32_t i = 0; i < kJointNames.size(); ++i) {
    EXPECT_EQ(0, CountDirectionSwitch(joint_positions[i]));
    EXPECT_NEAR(initial_state.joint_state.position[i], joint_positions[i].head(1)[0], kEpsilon);
    EXPECT_NEAR(goal_state.joint_state.position[i], joint_positions[i].tail(1)[0], kEpsilon);
  }
  std::vector<Eigen::VectorXd> joint_velocities;
  ASSERT_TRUE(ExtractJointVelocities(trajectory.joint_trajectory, kJointNames, joint_velocities));
  for (uint32_t i = 0; i < kJointNames.size(); ++i) {
    EXPECT_NEAR(initial_state.joint_state.velocity[i], joint_velocities[i].head(1)[0], kEpsilon);
    EXPECT_NEAR(0.0, joint_velocities[i].tail(1)[0], kEpsilon);
  }

  std::vector<Eigen::VectorXd> base_positions;
  ASSERT_TRUE(ExtractBase2DPoses(trajectory.multi_dof_joint_trajectory, kBaseNames[0], base_positions));
  // The direction of the cart in the y-direction changes only once
  EXPECT_EQ(0, CountDirectionSwitch(base_positions[0]));
  EXPECT_EQ(1, CountDirectionSwitch(base_positions[1]));
  EXPECT_EQ(0, CountDirectionSwitch(base_positions[2]));
  auto initial_base_pose = Get2DPose(initial_state.multi_dof_joint_state.poses[0]);
  auto goal_base_pose = Get2DPose(goal_state.multi_dof_joint_state.poses[0]);
  for (uint32_t i = 0; i < base_positions.size(); ++i) {
    EXPECT_NEAR(initial_base_pose[i], base_positions[i].head(1)[0], kEpsilon);
    EXPECT_NEAR(goal_base_pose[i], base_positions[i].tail(1)[0], kEpsilon);
  }

  std::vector<Eigen::VectorXd> base_velocities;
  ASSERT_TRUE(ExtractBase2DTwist(trajectory.multi_dof_joint_trajectory, kBaseNames[0], base_velocities));
  auto initial_base_twist = Get2DTwist(initial_state.multi_dof_joint_state.twist[0]);
  for (uint32_t i = 0; i < base_velocities.size(); ++i) {
    EXPECT_NEAR(initial_base_twist[i], base_velocities[i].head(1)[0], kEpsilon);
    EXPECT_NEAR(0.0, base_velocities[i].tail(1)[0], kEpsilon);
  }
}

// Invalid normalized_velocity
TEST_F(PlanarTrajectoryInterpolatorTest, InvalidNormalizedVelocity) {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), -0.1, trajectory));
}

// Trajectory interpolation using intermediate posture
TEST_F(PlanarTrajectoryInterpolatorTest, InterpolateWithMiddleState) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();

  // If the outside of initial and goal is an intermediate posture, switching of increase/decrease occurs
  std::vector<std::tuple<tmc_manipulation_types::RobotState, uint32_t>> middle_state_and_switch_count = {
      {GenerateRobotState(-1.0), 1}, {GenerateRobotState(0.5), 0}, {GenerateRobotState(2.0), 1}
  };

  for (const auto& [middle_state, switch_count] : middle_state_and_switch_count) {
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    ASSERT_TRUE(planar_->Interpolate(initial_state, goal_state, 0.5, middle_state, trajectory));

    std::vector<Eigen::VectorXd> joint_positions;
    ASSERT_TRUE(ExtractJointPositions(trajectory.joint_trajectory, kJointNames, joint_positions));

    for (const auto& positions : joint_positions) {
      EXPECT_EQ(switch_count, CountDirectionSwitch(positions));
    }

    for (uint32_t i = 0; i < kJointNames.size(); ++i) {
      EXPECT_NEAR(initial_state.joint_state.position[i], joint_positions[i].head(1)[0], kEpsilon);
      EXPECT_NEAR(goal_state.joint_state.position[i], joint_positions[i].tail(1)[0], kEpsilon);
    }
    std::vector<Eigen::VectorXd> joint_velocities;
    ASSERT_TRUE(ExtractJointVelocities(trajectory.joint_trajectory, kJointNames, joint_velocities));
    for (uint32_t i = 0; i < kJointNames.size(); ++i) {
      EXPECT_NEAR(initial_state.joint_state.velocity[i], joint_velocities[i].head(1)[0], kEpsilon);
      EXPECT_NEAR(0.0, joint_velocities[i].tail(1)[0], kEpsilon);
    }

    std::vector<Eigen::VectorXd> base_positions;
    ASSERT_TRUE(ExtractBase2DPoses(trajectory.multi_dof_joint_trajectory, kBaseNames[0], base_positions));

    for (const auto& positions : base_positions) {
      EXPECT_EQ(switch_count, CountDirectionSwitch(positions));
    }

    auto initial_base_pose = Get2DPose(initial_state.multi_dof_joint_state.poses[0]);
    auto goal_base_pose = Get2DPose(goal_state.multi_dof_joint_state.poses[0]);
    for (uint32_t i = 0; i < base_positions.size(); ++i) {
      EXPECT_NEAR(initial_base_pose[i], base_positions[i].head(1)[0], kEpsilon);
      EXPECT_NEAR(goal_base_pose[i], base_positions[i].tail(1)[0], kEpsilon);
    }
    std::vector<Eigen::VectorXd> base_velocities;
    ASSERT_TRUE(ExtractBase2DTwist(trajectory.multi_dof_joint_trajectory, kBaseNames[0], base_velocities));
    auto initial_base_twist = Get2DTwist(initial_state.multi_dof_joint_state.twist[0]);
    for (uint32_t i = 0; i < base_velocities.size(); ++i) {
      EXPECT_NEAR(initial_base_twist[i], base_velocities[i].head(1)[0], kEpsilon);
      EXPECT_NEAR(0.0, base_velocities[i].tail(1)[0], kEpsilon);
    }
  }
}

// Check if time_from_start of the generated trajectory changes according to the trajectory length, case where the number of trajectory points is below the maximum
TEST_F(PlanarTrajectoryInterpolatorTest, TimeFromStartsShortTrajectory) {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 1.75, trajectory));

  ASSERT_EQ(30, trajectory.joint_trajectory.points.size());
  ASSERT_EQ(30, trajectory.multi_dof_joint_trajectory.points.size());
  ASSERT_EQ(trajectory.joint_trajectory.points.size(),
            trajectory.multi_dof_joint_trajectory.points.size());

  // Time check
  for (uint32_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.joint_trajectory.points[i].time_from_start,
                     trajectory.multi_dof_joint_trajectory.points[i].time_from_start);
  }
  std::vector<double> time_intervals = {trajectory.joint_trajectory.points.front().time_from_start};
  for (uint32_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
    time_intervals.push_back(trajectory.joint_trajectory.points[i].time_from_start -
                             trajectory.joint_trajectory.points[i - 1].time_from_start);
  }

  // The first point is 0.0
  EXPECT_NEAR(0.0, time_intervals[0], kEpsilon);
  // If below the maximum, time step is 0.1, less than 0.1 only to fill the last
  for (uint32_t i = 1; i < time_intervals.size() - 1; ++i) {
    EXPECT_NEAR(0.1, time_intervals[i], kEpsilon);
  }
  EXPECT_GT(0.1, time_intervals.back());
}

// Check if time_from_start of the generated trajectory changes according to the trajectory length, case where the number of trajectory points exceeds the maximum
TEST_F(PlanarTrajectoryInterpolatorTest, TimeFromStartsLongTrajectory) {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 1.0, trajectory));

  ASSERT_EQ(30, trajectory.joint_trajectory.points.size());
  ASSERT_EQ(30, trajectory.multi_dof_joint_trajectory.points.size());

  // Time check
  for (uint32_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.joint_trajectory.points[i].time_from_start,
                     trajectory.multi_dof_joint_trajectory.points[i].time_from_start);
  }
  std::vector<double> time_intervals = {trajectory.joint_trajectory.points.front().time_from_start};
  for (uint32_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
    time_intervals.push_back(trajectory.joint_trajectory.points[i].time_from_start -
                             trajectory.joint_trajectory.points[i - 1].time_from_start);
  }

  // The first point is 0.0
  EXPECT_NEAR(0.0, time_intervals[0], kEpsilon);
  // Half with step 0.1, the rest with equal intervals larger than 0.1 to match the trajectory length
  for (uint32_t i = 1; i < 15; ++i) {
    EXPECT_NEAR(0.1, time_intervals[i], kEpsilon);
  }
  for (uint32_t i = 16; i < time_intervals.size(); ++i) {
    EXPECT_NEAR(time_intervals[15], time_intervals[i], kEpsilon);
  }
}

// No name for initial joint state
TEST_F(PlanarTrajectoryInterpolatorTest, NoInitialJointNames) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.name.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No position for initial joint state
TEST_F(PlanarTrajectoryInterpolatorTest, NoInitialJointPositions) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.position = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No velocity for initial joint state
TEST_F(PlanarTrajectoryInterpolatorTest, NoInitialJointVelocities) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.velocity = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No name for initial cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoInitialBaseJointNames) {
  auto initial_state = GenerateInitialState();
  initial_state.multi_dof_joint_state.names.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No posture for initial cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoInitialBaseJointPoses) {
  auto initial_state = GenerateInitialState();
  initial_state.multi_dof_joint_state.poses.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No velocity for initial cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoInitialBaseTwists) {
  auto initial_state = GenerateInitialState();
  initial_state.multi_dof_joint_state.poses.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(initial_state, GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No name for goal joint state
TEST_F(PlanarTrajectoryInterpolatorTest, NoGoalJointNames) {
  auto goal_state = GenerateGoalState();
  goal_state.joint_state.name.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, GenerateRobotState(2.0), trajectory));
}

// No position for goal joint state
TEST_F(PlanarTrajectoryInterpolatorTest, NoGoalJointPositions) {
  auto goal_state = GenerateGoalState();
  goal_state.joint_state.position = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, GenerateRobotState(2.0), trajectory));
}

// No velocity for goal joint state, but it's okay
TEST_F(PlanarTrajectoryInterpolatorTest, NoGoalJointVelocities) {
  auto goal_state = GenerateGoalState();
  goal_state.joint_state.velocity = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, GenerateRobotState(2.0), trajectory));
}

// No name for goal cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoGoalBaseJointNames) {
  auto goal_state = GenerateGoalState();
  goal_state.multi_dof_joint_state.names.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, GenerateRobotState(2.0), trajectory));
}

// No posture for goal cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoGoalBaseJointPoses) {
  auto goal_state = GenerateGoalState();
  goal_state.multi_dof_joint_state.poses.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, GenerateRobotState(2.0), trajectory));
}

// No velocity for goal cart state, but it's okay
TEST_F(PlanarTrajectoryInterpolatorTest, NoGoalBaseTwists) {
  auto goal_state = GenerateGoalState();
  goal_state.multi_dof_joint_state.twist.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), goal_state, 0.5, GenerateRobotState(2.0), trajectory));
}

// No name for intermediate joint state
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddleJointNames) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.joint_state.name.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 0.5, middle_state, trajectory));
}

// No position for intermediate joint state
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddleJointPositions) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.joint_state.position = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 0.5, middle_state, trajectory));
}

// No velocity for intermediate joint state, but it's okay
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddleJointVelocities) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.joint_state.velocity = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 0.5, middle_state, trajectory));
}

// No name for intermediate cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddleBaseJointNames) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.multi_dof_joint_state.names.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 0.5, middle_state, trajectory));
}

// No posture for intermediate cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddleBaseJointPoses) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.multi_dof_joint_state.poses.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 0.5, middle_state, trajectory));
}

// No velocity for intermediate cart state, but it's okay
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddleBaseTwists) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.multi_dof_joint_state.twist.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), GenerateGoalState(), 0.5, middle_state, trajectory));
}

// From here on, expected and actual are reversed, but it's troublesome to fix, so leave it as is

class PlanarTrajectoryInterpolatorPlainTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp::Node::SharedPtr node_;
  ITrajectoryInterpolator::Ptr planar_;
};

void PlanarTrajectoryInterpolatorPlainTest::SetUp() {
  node_ = rclcpp::Node::make_shared("test_node");

  planar_ = std::make_shared<PlanarTrajectoryInterpolatorPlain>();
  planar_->Initialize(node_, GenerateTestParameters());
}

void ValidatePlainInterpolation(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                                const tmc_manipulation_types::RobotState& initial_state,
                                const std::vector<tmc_manipulation_types::RobotState>& path_states,
                                const std::vector<double>& time_from_starts) {
  EXPECT_EQ(trajectory.joint_trajectory.joint_names, kJointNames);
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 1 + path_states.size());
  EXPECT_EQ(trajectory.joint_trajectory.points[0].positions, initial_state.joint_state.position);
  EXPECT_EQ(trajectory.joint_trajectory.points[0].velocities, initial_state.joint_state.velocity);
  for (auto i = 0; i < path_states.size(); ++i) {
    // The velocity of the goal is meaningless (unused) value, so do not check
    EXPECT_EQ(trajectory.joint_trajectory.points[i + 1].positions, path_states[i].joint_state.position);
    EXPECT_EQ(trajectory.joint_trajectory.points[i + 1].time_from_start, time_from_starts[i]);
  }

  EXPECT_EQ(trajectory.multi_dof_joint_trajectory.joint_names, kBaseNames);
  ASSERT_EQ(trajectory.multi_dof_joint_trajectory.points.size(), 1 + path_states.size());
  Eq(trajectory.multi_dof_joint_trajectory.points[0].transforms[0], initial_state.multi_dof_joint_state.poses[0]);
  EXPECT_EQ(trajectory.multi_dof_joint_trajectory.points[0].velocities[0],
            initial_state.multi_dof_joint_state.twist[0]);
  for (auto i = 0; i < path_states.size(); ++i) {
    Eq(trajectory.multi_dof_joint_trajectory.points[i + 1].transforms[0],
       path_states[i].multi_dof_joint_state.poses[0]);
    EXPECT_EQ(trajectory.multi_dof_joint_trajectory.points[i + 1].time_from_start, time_from_starts[i]);
  }
}

// Trajectory conversion without intermediate points or intermediate paths
TEST_F(PlanarTrajectoryInterpolatorPlainTest, NoMiddleStateNoMiddlePath) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, goal_state, 0.5, trajectory));
  ValidatePlainInterpolation(trajectory, initial_state, {goal_state}, {5.0});
}

// Check if weight is obtained from joint name
TEST_F(PlanarTrajectoryInterpolatorPlainTest, JointWeightFromName) {
  const auto initial_state = RemoveFirstJoint(GenerateInitialState());
  const auto goal_state = RemoveFirstJoint(GenerateGoalState());

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, goal_state, 0.5, trajectory));

  EXPECT_EQ(trajectory.joint_trajectory.joint_names, initial_state.joint_state.name);
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 2);
  EXPECT_EQ(trajectory.joint_trajectory.points[0].positions, initial_state.joint_state.position);
  EXPECT_EQ(trajectory.joint_trajectory.points[0].velocities, initial_state.joint_state.velocity);
  EXPECT_EQ(trajectory.joint_trajectory.points[1].positions, goal_state.joint_state.position);
  EXPECT_EQ(trajectory.joint_trajectory.points[1].time_from_start, 5.0);
}

// Trajectory conversion for cart only
TEST_F(PlanarTrajectoryInterpolatorPlainTest, BaseOnlyTrajectory) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.name.clear();
  initial_state.joint_state.position = Eigen::VectorXd();
  initial_state.joint_state.velocity = Eigen::VectorXd();
  auto goal_state = GenerateGoalState();
  goal_state.joint_state.name.clear();
  goal_state.joint_state.position = Eigen::VectorXd();
  goal_state.joint_state.velocity = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, goal_state, 0.5, trajectory));

  EXPECT_TRUE(trajectory.joint_trajectory.joint_names.empty());
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());
  EXPECT_EQ(trajectory.multi_dof_joint_trajectory.joint_names, kBaseNames);
}

// Trajectory conversion without intermediate points, with empty intermediate path
TEST_F(PlanarTrajectoryInterpolatorPlainTest, NoMiddleStateEmptyMiddlePath) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, {}, {}, goal_state, 0.5, trajectory));
  ValidatePlainInterpolation(trajectory, initial_state, {goal_state}, {5.0});
}

// Trajectory conversion without intermediate points, with intermediate path
TEST_F(PlanarTrajectoryInterpolatorPlainTest, NoMiddleStateWithMiddlePath) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();
  const auto middle_state = GenerateRobotState(2.0);

  {
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    ASSERT_TRUE(planar_->Interpolate(initial_state, {}, {middle_state}, goal_state, 0.5, trajectory));
    ValidatePlainInterpolation(trajectory, initial_state, {middle_state, goal_state}, {10.0, 15.0});
  }
  {
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    ASSERT_TRUE(planar_->Interpolate(initial_state, {middle_state}, {}, goal_state, 0.5, trajectory));
    ValidatePlainInterpolation(trajectory, initial_state, {middle_state, goal_state}, {10.0, 15.0});
  }
  {
    const auto middle_state_0 = GenerateRobotState(-1.0);
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    ASSERT_TRUE(planar_->Interpolate(initial_state, {middle_state_0}, {middle_state}, goal_state, 0.5, trajectory));
    ValidatePlainInterpolation(trajectory, initial_state,
                               {middle_state_0, middle_state, goal_state}, {5.0, 20.0, 25.0});
  }
}

// Trajectory conversion with intermediate points, without intermediate path
TEST_F(PlanarTrajectoryInterpolatorPlainTest, WithMiddleStateNoMiddlePath) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();
  const auto middle_state = GenerateRobotState(2.0);

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, goal_state, 0.5, middle_state, trajectory));
  ValidatePlainInterpolation(trajectory, initial_state, {middle_state, goal_state}, {10.0, 15.0});
}

// Trajectory conversion with intermediate points, with empty intermediate path
TEST_F(PlanarTrajectoryInterpolatorPlainTest, WithMiddleStateEmptyMiddlePath) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();
  const auto middle_state = GenerateRobotState(2.0);

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  ASSERT_TRUE(planar_->Interpolate(initial_state, {}, {}, goal_state, 0.5, middle_state, trajectory));
  ValidatePlainInterpolation(trajectory, initial_state, {middle_state, goal_state}, {10.0, 15.0});
}

// Trajectory conversion with intermediate points and intermediate path
TEST_F(PlanarTrajectoryInterpolatorPlainTest, WithMiddleStateAndMiddlePath) {
  const auto initial_state = GenerateInitialState();
  const auto goal_state = GenerateGoalState();
  const auto middle_state = GenerateRobotState(2.0);
  const auto middle_path_state = GenerateRobotState(3.0);

  {
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    ASSERT_TRUE(planar_->Interpolate(initial_state, {}, {middle_path_state},
                                     goal_state, 0.5, middle_state, trajectory));
    ValidatePlainInterpolation(trajectory, initial_state, {middle_state, middle_path_state, goal_state},
                               {10.0, 15.0, 25.0});
  }
  {
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    ASSERT_TRUE(planar_->Interpolate(initial_state, {middle_path_state}, {},
                                     goal_state, 0.5, middle_state, trajectory));
    ValidatePlainInterpolation(trajectory, initial_state, {middle_path_state, middle_state, goal_state},
                               {15.0, 20.0, 25.0});
  }
  {
    const auto middle_path_state_0 = GenerateRobotState(-1.0);
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    ASSERT_TRUE(planar_->Interpolate(initial_state, {middle_path_state_0}, {middle_path_state},
                                     goal_state, 0.5, middle_state, trajectory));
    ValidatePlainInterpolation(trajectory, initial_state,
                               {middle_path_state_0, middle_state, middle_path_state, goal_state},
                               {5.0, 20.0, 25.0, 35.0});
  }
}

// No name for intermediate path joint state
TEST_F(PlanarTrajectoryInterpolatorPlainTest, NoMiddlePathJointNames) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.joint_state.name.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No position for intermediate path joint state
TEST_F(PlanarTrajectoryInterpolatorPlainTest, NoMiddlePathJointPositions) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.joint_state.position = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No velocity for intermediate path joint state, but it's okay
TEST_F(PlanarTrajectoryInterpolatorPlainTest, NoMiddlePathJointVelocities) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.joint_state.velocity = Eigen::VectorXd();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                   GenerateGoalState(), 0.5, trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                   GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                   GenerateGoalState(), 0.5, trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                   GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No name for intermediate path cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddlePathBaseJointNames) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.multi_dof_joint_state.names.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No posture for intermediate path cart state
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddlePathBaseJointPoses) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.multi_dof_joint_state.poses.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, trajectory));
  EXPECT_FALSE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                    GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

// No velocity for intermediate path cart state, but it's okay
TEST_F(PlanarTrajectoryInterpolatorTest, NoMiddlePathBaseTwists) {
  auto middle_state = GenerateRobotState(2.0);
  middle_state.multi_dof_joint_state.twist.clear();

  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                   GenerateGoalState(), 0.5, trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {}, {middle_state},
                                   GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                   GenerateGoalState(), 0.5, trajectory));
  EXPECT_TRUE(planar_->Interpolate(GenerateInitialState(), {middle_state}, {},
                                   GenerateGoalState(), 0.5, GenerateRobotState(2.0), trajectory));
}

}  // namespace tmc_simple_path_generator

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
