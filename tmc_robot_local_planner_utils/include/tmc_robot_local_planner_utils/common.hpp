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
/// @brief Common functions
#ifndef TMC_ROBOT_LOCAL_PLANNER_UTILS_COMMON_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_UTILS_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_local_planner_utils {

// /// @brief Get the map origin
// /// @param map Map
// /// @return Map origin
// Eigen::Affine2d GetMapOrigin(const tmc_manipulation_types::OccupancyGrid& map);

/// @brief Get the last trajectory point
/// @param trajectory Joint trajectory
/// @param state_out Last trajectory point
/// @return Function success or failure
bool GetLastPoint(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                  tmc_manipulation_types::RobotState& state_out);

/// @brief Erase trajectory points exceeding the specified time
/// @param time Threshold time for erasure
/// @param trajectory Returns a list of processed joint trajectories from the input list of joint trajectories
void DeleteTrajectoryPointsAtTime(double time,
                                  tmc_manipulation_types::TimedRobotTrajectory& trajectory);

/// @brief Erase trajectory points in order of oldest timestamps for the specified number of points
/// @param num Number of points to erase
/// @param trajectory Returns a list of processed joint trajectories from the input list of joint trajectories
/// @return Function success or failure
bool DeleteTrajectoryPointsAtPoint(uint32_t num, trajectory_msgs::msg::JointTrajectory& trajectory);

/// @brief Determine if joint trajectory points and base trajectory points have the same time
/// @param trajectory Joint trajectory
/// @return Result of the same time determination
bool CheckTimeFromStarts(const tmc_manipulation_types::TimedRobotTrajectory& trajectory);

/// @brief Return the trajectory point with the smallest time greater than the current time + period from the input trajectory
/// @param trajectory Joint trajectory
/// @param target_time Current time + period time
/// @param connectable_time Connectable trajectory time
/// @param state_out Trajectory point close to current time + period
/// @return Function success or failure
bool SearchConnectablePoint(
    const tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory>& trajectory,
    const rclcpp::Time& target_time,
    rclcpp::Time& connectable_time,
    tf2::Stamped<tmc_manipulation_types::RobotState>& state_out);

/// @brief Shift the trajectory time
/// @param trajectory Joint trajectory
/// @param shift_time Time to shift
/// @param trajectory_out Trajectory point close to current time + period
void ShiftTrajectoryTimes(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    double shift_time,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out);

}  // namespace tmc_robot_local_planner_utils
#endif  // TMC_ROBOT_LOCAL_PLANNER_UTILS_COMMON_HPP_
