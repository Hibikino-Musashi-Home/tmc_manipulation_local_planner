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
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tmc_eigen_utils/eigen_utils.hpp>
#include <tmc_planning_msgs/msg/constraints.hpp>
#include <tmc_planning_msgs/msg/robot_local_goal.hpp>
#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

#include "markers.hpp"

namespace {
constexpr double kPositionBoundDistanceThreshold = 0.2;
constexpr double kPositionBoundMaxValueAbs = 1.0;

constexpr double kRotationBoundDistanceThreshold = M_PI / 4.0;

Eigen::Affine3d RegionValuesToPose(double x, double y, double z, double roll, double pitch, double yaw) {
  return Eigen::Translation3d(x, y, z) * tmc_eigen_utils::RPYToQuaternion(Eigen::Vector3d(roll, pitch, yaw));
}

Eigen::Affine3d RotationFirstRegionValuesToPose(double x, double y, double z, double roll, double pitch, double yaw) {
  const Eigen::Affine3d linear = Eigen::Translation3d::Identity()
                               * tmc_eigen_utils::RPYToQuaternion(Eigen::Vector3d(roll, pitch, yaw));
  const Eigen::Affine3d trans = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond::Identity();
  return linear * trans;
}

}  // namespace

namespace tmc_robot_local_planner_visualization {

struct PoseStamped {
  std::string frame_id;
  rclcpp::Time stamp;
  Eigen::Affine3d pose;
};
using TsrMarkersAndPoses = std::pair<std::vector<visualization_msgs::msg::Marker>, std::vector<PoseStamped>>;

TsrMarkersAndPoses GenerateTsrMarkers(
    const std::vector<tmc_planning_msgs::msg::TsrLinkConstraint>& constraints,
    int32_t start_id,
    const rclcpp::Time& stamp,
    const MarkerParameters& params) {
  // When converted to the type of tmc_robot_local_planner, min/max_bounds, etc. are hidden as private, so process it as a msg
  std::vector<visualization_msgs::msg::Marker> markers;
  std::vector<PoseStamped> poses;
  // Since tmc_planning_msgs 3.0.0, a constraint has multiple TSRs. Visualize all of them.
  std::vector<std::pair<std_msgs::msg::Header, tmc_planning_msgs::msg::TaskSpaceRegion>> header_and_tsrs;
  for (const auto& constraint : constraints) {
    for (const auto& tsr : constraint.tsrs) {
      header_and_tsrs.emplace_back(constraint.header, tsr);
    }
  }
  for (const auto& [header, tsr] : header_and_tsrs) {
    if (header.frame_id.empty()) {
      continue;
    }
    Eigen::Affine3d origin_to_tsr;
    tf2::fromMsg(tsr.origin_to_tsr, origin_to_tsr);

    Eigen::Affine3d tsr_to_end;
    tf2::fromMsg(tsr.tsr_to_end, tsr_to_end);

    std::vector<std::vector<double>> candidates_xyz;
    for (auto i = 0; i < 3; ++i) {
      const auto distance = std::abs(tsr.max_bounds[i] - tsr.min_bounds[i]);
      if (distance < std::numeric_limits<double>::min()) {
        candidates_xyz.emplace_back(std::vector<double>({tsr.min_bounds[i]}));
      } else if (distance < kPositionBoundDistanceThreshold) {
        candidates_xyz.emplace_back(std::vector<double>({tsr.min_bounds[i], tsr.max_bounds[i]}));
      } else {
        const auto min_value = std::max(-kPositionBoundMaxValueAbs, tsr.min_bounds[i]);
        const auto max_value = std::min(kPositionBoundMaxValueAbs, tsr.max_bounds[i]);
        candidates_xyz.emplace_back(std::vector<double>({min_value, (min_value + max_value) / 2.0, max_value}));
      }
    }

    std::vector<std::vector<double>> candidates_rpy;
    for (auto i = 3; i < 6; ++i) {
      const auto distance = std::abs(tsr.max_bounds[i] - tsr.min_bounds[i]);
      if (distance < std::numeric_limits<double>::min()) {
        candidates_rpy.emplace_back(std::vector<double>({tsr.min_bounds[i]}));
      } else if (distance < kRotationBoundDistanceThreshold) {
        candidates_rpy.emplace_back(std::vector<double>({tsr.min_bounds[i], tsr.max_bounds[i]}));
      } else {
        const auto min_value = std::max(-M_PI, tsr.min_bounds[i]);
        const auto max_value = std::min(M_PI, tsr.max_bounds[i]);
        // It's fine as long as it's somewhat divided, so I haven't properly considered rounding
        const auto step = (max_value - min_value) / (std::round(distance / kRotationBoundDistanceThreshold) + 1.0);
        std::vector<double> candidates;
        for (auto value = min_value; value < max_value; value += step) {
          candidates.push_back(value);
        }
        if (std::abs(candidates.back() - max_value) > (step / 2.0)) {
          candidates.push_back(max_value);
        }
        candidates_rpy.push_back(candidates);
      }
    }

    for (auto x : candidates_xyz[0]) {
      for (auto y : candidates_xyz[1]) {
        for (auto z : candidates_xyz[2]) {
          for (auto roll : candidates_rpy[0]) {
            for (auto pitch : candidates_rpy[1]) {
              for (auto yaw : candidates_rpy[2]) {
                Eigen::Affine3d origin_to_end;
                if (tsr.rotation_first) {
                  origin_to_end =
                      origin_to_tsr * RotationFirstRegionValuesToPose(x, y, z, roll, pitch, yaw) * tsr_to_end;
                } else {
                  origin_to_end = origin_to_tsr * RegionValuesToPose(x, y, z, roll, pitch, yaw) * tsr_to_end;
                }
                markers.emplace_back(GeneratePoseMarkers(
                    origin_to_end, stamp, header.frame_id, start_id + markers.size(), params));
                poses.emplace_back(PoseStamped{header.frame_id, stamp, origin_to_end});
              }
            }
          }
        }
      }
    }
  }
  return {markers, poses};
}

visualization_msgs::msg::Marker GenerateOriginMarker(
    const PoseStamped& pose_stamped,
    int32_t id,
    const MarkerParameters& params) {
  visualization_msgs::msg::Marker cube;
  cube.header.stamp = pose_stamped.stamp;
  cube.header.frame_id = pose_stamped.frame_id;
  cube.ns = params.ns;
  cube.id = id;
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.action = visualization_msgs::msg::Marker::ADD;
  cube.pose = tf2::toMsg(pose_stamped.pose);
  cube.scale.x = params.line_width * 2.0;
  cube.scale.y = params.line_width * 2.0;
  cube.scale.z = params.line_width * 2.0;
  cube.lifetime.sec = params.life_time;
  cube.color.r = 1.0;
  cube.color.g = 1.0;
  cube.color.b = 1.0;
  cube.color.a = params.alpha;
  return cube;
}

bool ValidateAxis(const geometry_msgs::msg::Vector3& axis) {
  return std::abs(axis.x) + std::abs(axis.y) + std::abs(axis.z) > std::numeric_limits<double>::min();
}

visualization_msgs::msg::Marker GenerateGoalRelativeMarker(
    const PoseStamped& pose_stamped,
    const tmc_planning_msgs::msg::LinearConstraint& constraint,
    int32_t id,
    const MarkerParameters& params) {
  visualization_msgs::msg::Marker arrow;
  arrow.header.stamp = pose_stamped.stamp;
  arrow.header.frame_id = pose_stamped.frame_id;
  arrow.ns = params.ns;
  arrow.id = id;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;

  const auto origin_to_end = pose_stamped.pose;
  const auto end_to_start_axis = Eigen::Vector3d(constraint.axis.x, constraint.axis.y, constraint.axis.z);
  const auto end_to_start_axis_normalized = end_to_start_axis.normalized();
  const auto origin_to_start = origin_to_end * Eigen::Translation3d(
      end_to_start_axis_normalized * constraint.distance);
  arrow.points.resize(2);
  arrow.points[0].x = origin_to_start.translation().x();
  arrow.points[0].y = origin_to_start.translation().y();
  arrow.points[0].z = origin_to_start.translation().z();
  arrow.points[1].x = pose_stamped.pose.translation().x();
  arrow.points[1].y = pose_stamped.pose.translation().y();
  arrow.points[1].z = pose_stamped.pose.translation().z();

  arrow.scale.x = params.line_width;
  arrow.scale.y = params.line_width * 2.0;
  arrow.lifetime.sec = params.life_time;
  arrow.color.r = 1.0;
  arrow.color.g = 1.0;
  arrow.color.b = 0.0;
  arrow.color.a = params.alpha;
  return arrow;
}

visualization_msgs::msg::Marker GenerateInitialMotionMarker(
    const tmc_planning_msgs::msg::LinearConstraintWithPose& constraint,
    const rclcpp::Time& stamp,
    int32_t id,
    const MarkerParameters& params) {
  visualization_msgs::msg::Marker arrow;
  arrow.header.stamp = stamp;
  arrow.header.frame_id = constraint.reference.header.frame_id;
  arrow.ns = params.ns;
  arrow.id = id;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;

  Eigen::Affine3d origin_to_start;
  tf2::fromMsg(constraint.reference.pose, origin_to_start);
  const auto start_to_end_axis = Eigen::Vector3d(constraint.axis.x, constraint.axis.y, constraint.axis.z);
  const auto start_to_end_axis_normalized = start_to_end_axis.normalized();
  const auto origin_to_end = origin_to_start * Eigen::Translation3d(
      start_to_end_axis_normalized * constraint.distance);
  arrow.points.resize(2);
  arrow.points[0] = constraint.reference.pose.position;
  arrow.points[1].x = origin_to_end.translation().x();
  arrow.points[1].y = origin_to_end.translation().y();
  arrow.points[1].z = origin_to_end.translation().z();

  arrow.scale.x = params.line_width;
  arrow.scale.y = params.line_width * 2.0;
  arrow.lifetime.sec = params.life_time;
  arrow.color.r = 0.0;
  arrow.color.g = 1.0;
  arrow.color.b = 1.0;
  arrow.color.a = params.alpha;
  return arrow;
}


class LinkConstraintsVisualization : public rclcpp::Node {
 public:
  LinkConstraintsVisualization() : LinkConstraintsVisualization(rclcpp::NodeOptions()) {}
  explicit LinkConstraintsVisualization(const rclcpp::NodeOptions& options);

 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  rclcpp::Subscription<tmc_planning_msgs::msg::Constraints>::SharedPtr constraints_sub_;
  void ConstraintsCallback(const tmc_planning_msgs::msg::Constraints::SharedPtr msg);

  rclcpp::Subscription<tmc_planning_msgs::msg::RobotLocalGoal>::SharedPtr robot_local_goal_sub_;
  void RobotLocalGoalCallback(const tmc_planning_msgs::msg::RobotLocalGoal::SharedPtr msg);

  void Visualize(const tmc_planning_msgs::msg::Constraints& constraints_msg);

  tmc_utils::DynamicParameter<double>::Ptr line_length_hard_;
  tmc_utils::DynamicParameter<double>::Ptr line_width_hard_;
  tmc_utils::DynamicParameter<double>::Ptr alpha_hard_;
  tmc_utils::DynamicParameter<double>::Ptr line_length_soft_;
  tmc_utils::DynamicParameter<double>::Ptr line_width_soft_;
  tmc_utils::DynamicParameter<double>::Ptr alpha_soft_;
  tmc_utils::DynamicParameter<int32_t>::Ptr life_time_;
};

LinkConstraintsVisualization::LinkConstraintsVisualization(const rclcpp::NodeOptions& options)
    : Node("link_constraints_visualization", options) {
  line_length_hard_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.hard.line_length", 0.2);
  line_width_hard_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.hard.line_width", 0.02);
  alpha_hard_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.hard.alpha", 1.0);
  line_length_soft_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.soft.line_length", 0.2);
  line_width_soft_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.soft.line_width", 0.04);
  alpha_soft_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.soft.alpha", 0.5);
  life_time_ = std::make_shared<tmc_utils::DynamicParameter<int32_t>>(this, "marker.life_time", 0);

  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/link_constraints_marker_array", tmc_utils::BestEffortQoS(2));

  constraints_sub_ = this->create_subscription<tmc_planning_msgs::msg::Constraints>(
      "~/constraints", tmc_utils::BestEffortQoS(),
      std::bind(&LinkConstraintsVisualization::ConstraintsCallback, this, std::placeholders::_1));

  robot_local_goal_sub_ = this->create_subscription<tmc_planning_msgs::msg::RobotLocalGoal>(
      "~/robot_local_goal", tmc_utils::BestEffortQoS(),
      std::bind(&LinkConstraintsVisualization::RobotLocalGoalCallback, this, std::placeholders::_1));
}

void LinkConstraintsVisualization::ConstraintsCallback(const tmc_planning_msgs::msg::Constraints::SharedPtr msg) {
  Visualize(*msg);
}

void LinkConstraintsVisualization::RobotLocalGoalCallback(const tmc_planning_msgs::msg::RobotLocalGoal::SharedPtr msg) {
  Visualize(msg->constraints);
}

void LinkConstraintsVisualization::Visualize(const tmc_planning_msgs::msg::Constraints& constraints_msg) {
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.resize(1);
    markers.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_pub_->publish(markers);
  }

  const auto stamp = this->now();
  visualization_msgs::msg::MarkerArray markers;

  MarkerParameters hard_goal_params;
  hard_goal_params.ns = "hard_link_constraints";
  hard_goal_params.line_length = line_length_hard_->value();
  hard_goal_params.line_width = line_width_hard_->value();
  hard_goal_params.alpha = alpha_hard_->value();
  hard_goal_params.life_time = life_time_->value();

  const auto [hard_goal_markers, hard_goal_poses] = GenerateTsrMarkers(
      constraints_msg.hard_link_constraints, markers.markers.size(), stamp, hard_goal_params);
  markers.markers.insert(markers.markers.end(), hard_goal_markers.begin(), hard_goal_markers.end());

  for (const auto& pose_stamped : hard_goal_poses) {
    markers.markers.push_back(GenerateOriginMarker(pose_stamped, markers.markers.size(), hard_goal_params));
  }
  for (const auto& linear_constraint : constraints_msg.goal_relative_linear_constraints) {
    if (!linear_constraint.end_frame_id.empty() && ValidateAxis(linear_constraint.axis)) {
      for (const auto& pose_stamped : hard_goal_poses) {
        markers.markers.push_back(GenerateGoalRelativeMarker(
            pose_stamped, linear_constraint, markers.markers.size(), hard_goal_params));
      }
    }
  }
  for (const auto& linear_constraint : constraints_msg.initial_motion_linear_constraints) {
    if (!linear_constraint.end_frame_id.empty() && ValidateAxis(linear_constraint.axis)) {
      markers.markers.push_back(GenerateInitialMotionMarker(
          linear_constraint, stamp, markers.markers.size(), hard_goal_params));
    }
  }

  MarkerParameters soft_goal_params;
  soft_goal_params.ns = "soft_link_constraints";
  soft_goal_params.line_length = line_length_soft_->value();
  soft_goal_params.line_width = line_width_soft_->value();
  soft_goal_params.alpha = alpha_soft_->value();
  soft_goal_params.life_time = life_time_->value();

  const auto [soft_goal_markers, soft_goal_poses] = GenerateTsrMarkers(
      constraints_msg.soft_link_constraints, markers.markers.size(), stamp, soft_goal_params);
  markers.markers.insert(markers.markers.end(), soft_goal_markers.begin(), soft_goal_markers.end());

  for (const auto& pose_stamped : soft_goal_poses) {
    markers.markers.push_back(GenerateOriginMarker(pose_stamped, markers.markers.size(), soft_goal_params));
  }

  MarkerParameters hard_path_params;
  hard_path_params.ns = "hard_path_link_constraints";
  hard_path_params.line_length = line_length_hard_->value();
  hard_path_params.line_width = line_width_hard_->value();
  hard_path_params.alpha = alpha_hard_->value();
  hard_path_params.life_time = life_time_->value();

  const auto [hard_path_markers, hard_path_poses] = GenerateTsrMarkers(
      constraints_msg.hard_path_link_constraints, markers.markers.size(), stamp, hard_path_params);
  markers.markers.insert(markers.markers.end(), hard_path_markers.begin(), hard_path_markers.end());

  marker_array_pub_->publish(markers);
}

}  // namespace tmc_robot_local_planner_visualization

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tmc_robot_local_planner_visualization::LinkConstraintsVisualization)
