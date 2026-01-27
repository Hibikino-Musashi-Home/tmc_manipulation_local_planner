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
/// @brief Test of ArrivalRateCalculator

#include <gtest/gtest.h>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_local_planner/range_joint_constraint.hpp>
// #include <tmc_robot_local_planner/tsr_link_constraint.hpp>
#include <tmc_robot_local_planner_utils/arrival_rate_calculator.hpp>

namespace tmc_robot_local_planner_utils {

class ArrivalRateCalculatorTest : public ::testing::Test {
 protected:
  ArrivalRateCalculatorTest() {}

  void SetUp() override;
  void TearDown() override;

  ArrivalRateCalculator::Ptr arrival_rate_calculator_;
  std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3",
                                           "joint4", "joint5", "joint6"};
  tmc_robot_local_planner::Constraints constraints_;
  tmc_manipulation_types::RobotState min_;
  tmc_manipulation_types::RobotState max_;
  // tmc_manipulation_types::TaskSpaceRegion tsr_;
  tmc_manipulation_types::RobotState robot_state_;
};

void ArrivalRateCalculatorTest::SetUp() {
  // Instance creation
  arrival_rate_calculator_ = std::make_shared<ArrivalRateCalculator>();

  // Setting of RangeJointConstraint
  min_.joint_state.name = joint_names_;
  min_.joint_state.position.resize(joint_names_.size());
  min_.joint_state.position << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
  max_ = min_;

  // // Setting of TsrLinkConstraint
  // tsr_.end_frame_id = "link7";
  // Eigen::Translation3d translation(0.0, 0.0, 0.0);
  // Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);
  // tsr_.tsr_to_end = translation * quaternion;
  // Eigen::VectorXd bounds;
  // bounds.resize(6);
  // bounds << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // tsr_.min_bounds = bounds;
  // tsr_.max_bounds = tsr_.min_bounds;

  robot_state_.joint_state.name = joint_names_;
  robot_state_.joint_state.position.resize(joint_names_.size());
  robot_state_.joint_state.position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  robot_state_.multi_dof_joint_state.names.push_back("world_joint");
  robot_state_.multi_dof_joint_state.poses.push_back(Eigen::Affine3d::Identity());
}

void ArrivalRateCalculatorTest::TearDown() {
  constraints_.hard_joint_constraints.clear();
  constraints_.hard_link_constraints.clear();
}

// ArrivalRate can be calculated with hard_joint_constraints
TEST_F(ArrivalRateCalculatorTest, JointConstraintCalculate) {
  // setup
  constraints_.hard_joint_constraints.push_back(
      std::make_shared<tmc_robot_local_planner::RangeJointConstraint>(min_, max_, 0, 0));

  // ArrivalRate when receiving a constraint
  arrival_rate_calculator_->SetConstraints(constraints_);
  auto arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
  ASSERT_TRUE(static_cast<bool>(arrival_rate));
  // The initial arrival rate when a constraint is set should be 0%
  EXPECT_NEAR(arrival_rate.get(), 0.0, 0.01);

  // ArrivalRate when not receiving a constraint
  robot_state_.joint_state.position << 0.6, 0.7, 0.8, 0.9, 1.0, 1.1;
  arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
  ASSERT_TRUE(static_cast<bool>(arrival_rate));
  // If it moves halfway, the arrival rate should be 50%
  EXPECT_NEAR(arrival_rate.get(), 50.0, 0.01);

  robot_state_.joint_state.position << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
  arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
  ASSERT_TRUE(static_cast<bool>(arrival_rate));
  // If it reaches the goal, it should be 100%
  EXPECT_NEAR(arrival_rate.get(), 100.0, 0.01);
}

// // ArrivalRate can be calculated with hard_link_constraints
// TEST_F(ArrivalRateCalculatorTest, LinkConstraintCalculate) {
//   // setup
//   // Position of link7 when each joint is 0.0
//   Eigen::Translation3d translation(0.0, -0.25, 0.95);
//   Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);
//   tsr_.origin_to_tsr = translation * quaternion;
//   constraints_.hard_link_constraints.push_back(
//       std::make_shared<TsrLinkConstraint>(tsr_, 0, 0));

//   // ArrivalRate when receiving a constraint
//   arrival_rate_calculator_->SetConstraints(constraints_);
//   // Move only the linear axis joinr3
//   robot_state_.joint_state.position << 0.0, 0.0, 0.2, 0.0, 0.0, 0.0;
//   auto arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   // The initial arrival rate when a constraint is set should be 0%
//   EXPECT_NEAR(arrival_rate.get(), 0.0, 0.01);

//   // ArrivalRate when not receiving a constraint
//   robot_state_.joint_state.position << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0;
//   arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   // Arrival rate should be 50% when joint3 is at half the value of the initial + target value
//   EXPECT_NEAR(arrival_rate.get(), 50, 0.01);

//   robot_state_.joint_state.position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//   arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   // Arrival rate should be 100% when each joint is 0.0
//   EXPECT_NEAR(arrival_rate.get(), 100, 0.01);
// }

// // hard_link_constraints when there is no robot model
// TEST_F(ArrivalRateCalculatorTest, LinkConstraintsCannotCalculatedWithoutKinematicsModel) {
//   // setup
//   Eigen::Translation3d translation(1.0, 1.0, 1.0);
//   Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);
//   tsr_.origin_to_tsr = translation * quaternion;
//   constraints_.hard_link_constraints.push_back(
//       std::make_shared<TsrLinkConstraint>(tsr_, 0, 0));

//   // Retain and delete robot model parameters
//   std::string keep_description;
//   nh_.getParam("robot_description", keep_description);
//   nh_.deleteParam("robot_description");
//   // Confirm absence
//   ASSERT_FALSE(nh_.hasParam("robot_description"));

//   // Instance creation
//   arrival_rate_calculator_ = ArrivalRateCalculator::Ptr(new ArrivalRateCalculator());

//   // Cannot calculate without a robot model
//   arrival_rate_calculator_->SetConstraints(constraints_);
//   auto arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_FALSE(static_cast<bool>(arrival_rate));

//   // teardown
//   nh_.setParam("robot_description", keep_description);
// }

// // hard_joint_constraints when there is no robot model
// TEST_F(ArrivalRateCalculatorTest, JointConstraintsCanCalculatedWithoutKinematicsModel) {
//   // setup
//   constraints_.hard_joint_constraints.push_back(
//       std::make_shared<RangeJointConstraint>(min_, max_, 0, 0));

//   // Retain and delete robot model parameters
//   std::string keep_description;
//   nh_.getParam("robot_description", keep_description);
//   nh_.deleteParam("robot_description");
//   // Confirm absence
//   ASSERT_FALSE(nh_.hasParam("robot_description"));

//   // Instance creation
//   arrival_rate_calculator_ = ArrivalRateCalculator::Ptr(new ArrivalRateCalculator());

//   // Since joint_constraint is calculated by displacement, it can be calculated without a robot model
//   arrival_rate_calculator_->SetConstraints(constraints_);
//   auto arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   EXPECT_NEAR(arrival_rate.get(), 0.0, 0.01);

//   robot_state_.joint_state.position << 0.6, 0.7, 0.8, 0.9, 1.0, 1.1;
//   arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   EXPECT_NEAR(arrival_rate.get(), 50.0, 0.01);

//   // teardown
//   nh_.setParam("robot_description", keep_description);
// }

// // When both hard_link_constraint and hard_joint_constraint are present
// TEST_F(ArrivalRateCalculatorTest, JointAndLinkConstraints) {
//   // setup
//   // Setting of hard_joint_constraint
//   tmc_manipulation_types::RobotState min;
//   min.joint_state.name.push_back("joint6");
//   min.joint_state.position.resize(1);
//   min.joint_state.position << 0.0;
//   tmc_manipulation_types::RobotState max = min;
//   constraints_.hard_joint_constraints.push_back(
//       std::make_shared<RangeJointConstraint>(min, max, 0, 0));

//   // Position of link7 when each joint is 0.0
//   Eigen::Translation3d translation(0.0, -0.25, 0.95);
//   Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);
//   tsr_.origin_to_tsr = translation * quaternion;
//   constraints_.hard_link_constraints.push_back(
//       std::make_shared<TsrLinkConstraint>(tsr_, 0, 0));

//   // ArrivalRate when receiving a constraint
//   arrival_rate_calculator_->SetConstraints(constraints_);
//   robot_state_.joint_state.position << 0.0, 0.0, 0.2, 0.0, 0.0, 0.2;
//   auto arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   // The initial arrival rate when a constraint is set should be 0%
//   EXPECT_NEAR(arrival_rate.get(), 0.0, 0.01);

//   // ArrivalRate when not receiving a constraint
//   robot_state_.joint_state.position << 0.0, 0.0, 0.1, 0.0, 0.0, 0.1;
//   arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   // If it moves halfway, the arrival rate should be 50%
//   EXPECT_NEAR(arrival_rate.get(), 50.0, 0.01);

//   robot_state_.joint_state.position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//   arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
//   ASSERT_TRUE(static_cast<bool>(arrival_rate));
//   // If it reaches the goal, it should be 100%
//   EXPECT_NEAR(arrival_rate.get(), 100.0, 0.01);
// }

// When the constraint is empty
TEST_F(ArrivalRateCalculatorTest, EmptyConstraints) {
  // Set an empty constraint
  arrival_rate_calculator_->SetConstraints(constraints_);

  // If the constraint is empty, the arrival rate should be 100%
  auto arrival_rate = arrival_rate_calculator_->CalculateArrivalRate(robot_state_);
  ASSERT_TRUE(static_cast<bool>(arrival_rate));
  EXPECT_NEAR(arrival_rate.get(), 100.0, 0.01);
}

}  // namespace tmc_robot_local_planner_utils

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
