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
/// @brief Test of CollisionDetectingValidator
#include <filesystem>

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_utils/msg_io.hpp>

#include "../src/tmc_collision_detecting_validator/collision_detecting_validator.hpp"

namespace {
const char* const kLogDir = "/tmp/validator_test";

void SetMapData(const uint32_t& map_size,
                uint32_t free_ratio,
                nav_msgs::msg::OccupancyGrid& map_out) {
  auto free_num = std::round(map_size * free_ratio / 100);
  for (uint32_t i = 0; i < free_num; ++i) {
    map_out.data.push_back(0);
  }
  for (uint32_t i = free_num; i < map_size; ++i) {
    map_out.data.push_back(100);
  }
}

nav_msgs::msg::OccupancyGrid GenerateMap() {
  // Map with the lower half occupied
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.05;
  map.info.width = 200;
  map.info.height = 200;
  map.info.origin.position.x = 1.0;
  map.info.origin.position.y = -1.0;
  map.info.origin.orientation.z = 1.0;
  map.info.origin.orientation.w = 0.0;
  SetMapData(map.info.width * map.info.height, 50, map);
  return map;
}

moveit_msgs::msg::RobotTrajectory GenerateRobotTrajectory(double x, double y, uint32_t num_of_points) {
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  robot_trajectory.joint_trajectory.points.resize(num_of_points);
  for (uint32_t i = 0; i < num_of_points; ++i) {
    robot_trajectory.joint_trajectory.points[i].positions.resize(6, 1.57);
  }
  geometry_msgs::msg::Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.rotation.w = 1.0;
  robot_trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  robot_trajectory.multi_dof_joint_trajectory.points.resize(num_of_points);
  for (uint32_t i = 0; i < num_of_points; ++i) {
    robot_trajectory.multi_dof_joint_trajectory.points[i].transforms.push_back(transform);
  }
  return robot_trajectory;
}

moveit_msgs::msg::RobotTrajectory GenerateOnePointRobotTrajectory(double x, double y) {
  return GenerateRobotTrajectory(x, y, 1);
}

moveit_msgs::msg::RobotTrajectory GenerateTestDefaultTrajectory() {
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  robot_trajectory.joint_trajectory.points.resize(2);
  for (uint32_t i = 0; i < robot_trajectory.joint_trajectory.joint_names.size(); ++i) {
    robot_trajectory.joint_trajectory.points[0].positions.push_back(1.57);
    robot_trajectory.joint_trajectory.points[1].positions.push_back(0.0);
  }
  robot_trajectory.joint_trajectory.points[1].positions[2] = 0.3;
  geometry_msgs::msg::Transform transform;
  robot_trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  robot_trajectory.multi_dof_joint_trajectory.points.resize(2);
  transform.translation.x = 0.0;
  transform.translation.y = 0.5;
  robot_trajectory.multi_dof_joint_trajectory.points[0].transforms.push_back(transform);
  transform.translation.x = 1.0;
  transform.translation.y = 0.5;
  robot_trajectory.multi_dof_joint_trajectory.points[1].transforms.push_back(transform);

  return robot_trajectory;
}

tmc_manipulation_types::RobotState GenerateDefaultRobotState() {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(
      GenerateTestDefaultTrajectory(), trajectory);

  tmc_manipulation_types::RobotState robot_state;
  robot_state.joint_state.name = trajectory.joint_trajectory.joint_names;
  robot_state.joint_state.position = trajectory.joint_trajectory.points[0].positions;
  robot_state.multi_dof_joint_state.names = trajectory.multi_dof_joint_trajectory.joint_names;
  robot_state.multi_dof_joint_state.poses = trajectory.multi_dof_joint_trajectory.points[0].transforms;
  return robot_state;
}

void CheckTrajectory(const moveit_msgs::msg::RobotTrajectory& actual,
                     const moveit_msgs::msg::RobotTrajectory& expected) {
  ASSERT_EQ(actual.joint_trajectory.points.size(), expected.joint_trajectory.points.size());
  for (auto point_i = 0; point_i < expected.joint_trajectory.points.size(); ++point_i) {
    SCOPED_TRACE(::testing::Message() << "Point " << point_i);
    ASSERT_EQ(actual.joint_trajectory.points[point_i].positions.size(),
              expected.joint_trajectory.points[point_i].positions.size());
    for (auto position_i = 0; position_i < expected.joint_trajectory.points[point_i].positions.size(); ++position_i) {
      SCOPED_TRACE(::testing::Message() << "Position " << position_i);
      EXPECT_DOUBLE_EQ(actual.joint_trajectory.points[point_i].positions[position_i],
                       expected.joint_trajectory.points[point_i].positions[position_i]);
    }
  }

  ASSERT_EQ(actual.multi_dof_joint_trajectory.points.size(), expected.multi_dof_joint_trajectory.points.size());
  for (auto point_i = 0; point_i < expected.multi_dof_joint_trajectory.points.size(); ++point_i) {
    SCOPED_TRACE(::testing::Message() << "Point " << point_i);
    ASSERT_EQ(actual.multi_dof_joint_trajectory.points[point_i].transforms.size(),
              expected.multi_dof_joint_trajectory.points[point_i].transforms.size());
    for (auto position_i = 0; position_i < expected.multi_dof_joint_trajectory.points[point_i].transforms.size();
         ++position_i) {
      SCOPED_TRACE(::testing::Message() << "Transform " << position_i);
      auto actual_point = actual.multi_dof_joint_trajectory.points[point_i];
      auto expected_point = expected.multi_dof_joint_trajectory.points[point_i];
      ASSERT_EQ(actual_point.transforms.size(), 1);
      EXPECT_DOUBLE_EQ(actual_point.transforms[0].translation.x, expected_point.transforms[0].translation.x);
      EXPECT_DOUBLE_EQ(actual_point.transforms[0].translation.y, expected_point.transforms[0].translation.y);
      EXPECT_DOUBLE_EQ(std::atan2(actual_point.transforms[0].rotation.z, actual_point.transforms[0].rotation.w),
                       std::atan2(expected_point.transforms[0].rotation.z, expected_point.transforms[0].rotation.w));
    }
  }
}

moveit_msgs::msg::CollisionObject GenerateOneObstacleCube() {
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "odom";
  object.id = "object1";
  // Position that collides with the trajectory created by GenerateTestDefaultTrajectory
  object.pose.position.x = 1.0;
  object.pose.position.y = 0.5;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {0.01, 0.01, 0.01};
  object.primitive_poses.resize(1);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

// Temporarily overwrite environment variables
class EnvironmentVariable {
 public:
  EnvironmentVariable(const std::string& name, const std::string& value) : name_(name), value_(value) {
    old_value_ = std::getenv(name_.c_str());
    setenv(name_.c_str(), value_.c_str(), 1);
  }

  ~EnvironmentVariable() {
    if (old_value_) {
      setenv(name_.c_str(), old_value_, 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

 private:
  std::string name_;
  std::string value_;
  const char* old_value_;
};

// Create a directory for log files
class ScopedLogDir {
 public:
  ScopedLogDir() : ros_log_dir_(EnvironmentVariable("ROS_LOG_DIR", kLogDir)) {
    (void)std::filesystem::create_directories(kLogDir);
  }

  ~ScopedLogDir() {
    std::filesystem::remove_all(kLogDir);
  }

  std::vector<std::string> GetLogFiles(const std::string& postfix) {
    std::vector<std::string> result;
    for (const auto& entry : std::filesystem::directory_iterator(kLogDir)) {
      if (entry.is_regular_file() && entry.path().extension() == postfix) {
        result.push_back(entry.path().filename().string());
      }
    }
    return result;
  }

  std::optional<std::string> GetLatestLogFileFullPath(const std::string& postfix) {
    std::optional<std::filesystem::directory_entry> latest;
    auto latest_time = std::filesystem::file_time_type::min();
    for (const auto& entry : std::filesystem::directory_iterator(kLogDir)) {
      if (entry.is_regular_file() && entry.path().extension() == postfix) {
        auto last_write_time = entry.last_write_time();
        if (!latest || last_write_time > latest_time) {
          latest = entry;
          latest_time = last_write_time;
        }
      }
    }
    if (latest) {
      return latest->path().string();
    } else {
      return std::nullopt;
    }
  }

 private:
  EnvironmentVariable ros_log_dir_;
};

// Capture output of RCLCPP_*
class RclcppOutputCapture {
 public:
  RclcppOutputCapture() {
    original_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&RclcppOutputCapture::Handler);
  }

  ~RclcppOutputCapture() {
    rcutils_logging_set_output_handler(original_handler_);
    captured_outputs_.clear();
  }

  uint32_t GetRobotCollisionLogCount() const { return GetLogCount("Collision"); }
  uint32_t GetMapCollisionLogCount() const { return GetLogCount("Point"); }

 private:
  rcutils_logging_output_handler_t original_handler_;
  static std::vector<std::string> captured_outputs_;

  static void Handler(const rcutils_log_location_t* location,
                      int severity,
                      const char* name,
                      rcutils_time_point_value_t timestamp,
                      const char* format,
                      va_list* args) {
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, *args);
    captured_outputs_.emplace_back(std::string(buffer));
  }

  uint32_t GetLogCount(const std::string& prefix) const {
    uint32_t count = 0;
    for (const auto& output : captured_outputs_) {
      if (output.find(prefix) == 0) {
        ++count;
      }
    }
    return count;
  }
};
std::vector<std::string> RclcppOutputCapture::captured_outputs_;

// tmc_manipulation_msgs::CollisionObject GenerateObstacleCubes(int32_t id, double cube_size = 0.1) {
//   tmc_manipulation_msgs::CollisionObject collision_object;
//   collision_object.id.object_id = id;
//   collision_object.id.name = "object1";
//   collision_object.operation.operation = tmc_manipulation_msgs::CollisionObjectOperation::ADD;

//   // Range is appropriate
//   for (double x = 0.0; x < 3.0; x += cube_size) {
//     for (double y = 0.0; y < 3.0; y += cube_size) {
//       for (double z = 0.0; z < 1.0; z += cube_size) {
//         geometry_msgs::Pose pose;
//         pose.position.x = x;
//         pose.position.y = y;
//         pose.position.z = z;
//         pose.orientation.w = 1.0;
//         collision_object.poses.push_back(pose);
//       }
//     }
//   }

//   tmc_geometric_shapes_msgs::Shape shape;
//   shape.type = tmc_geometric_shapes_msgs::Shape::BOX;
//   shape.dimensions.resize(3);
//   shape.dimensions[0] = cube_size;
//   shape.dimensions[1] = cube_size;
//   shape.dimensions[2] = cube_size;
//   collision_object.shapes.resize(collision_object.poses.size(), shape);

//   collision_object.header.frame_id = collision_object.id.name + "_" + boost::lexical_cast<std::string>(id);
//   collision_object.header.stamp = ros::Time::now();
//   return collision_object;
// }

}  // namespace

namespace tmc_collision_detecting_validator {

// Differences in behavior due to parameter differences are tested in the node test
TEST(CollisionDetectingValidatorPluginTest, Validate) {
  pluginlib::ClassLoader<tmc_robot_local_planner::IValidator> loader(
      "tmc_robot_local_planner", "tmc_robot_local_planner::IValidator");
  auto validator = loader.createSharedInstance("tmc_collision_detecting_validator/CollisionDetectingValidatorPlugin");

  auto node = rclcpp::Node::make_shared("plugins");
  node->declare_parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("robot_collision_pair", tmc_manipulation_tests::stanford_manipulator::GetCollisionConfig());
  validator->Initialize(node);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_in;
  trajectories_in.resize(1);
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(
      GenerateTestDefaultTrajectory(), trajectories_in[0]);

  tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
  std::function<bool()> func = []() -> bool{ return false; };
  EXPECT_TRUE(validator->Validate(trajectories_in, GenerateDefaultRobotState(), func, trajectory_out));
}

// Test of multi-threaded implementation
TEST(CollisionDetectingValidatorPluginTest, ValidateMultiThread) {
  RclcppOutputCapture capture;

  pluginlib::ClassLoader<tmc_robot_local_planner::IValidator> loader(
      "tmc_robot_local_planner", "tmc_robot_local_planner::IValidator");
  auto validator_serial = loader.createSharedInstance(
      "tmc_collision_detecting_validator/CollisionDetectingValidatorPlugin");

  auto node = rclcpp::Node::make_shared("plugins");
  node->declare_parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("robot_collision_pair", tmc_manipulation_tests::stanford_manipulator::GetCollisionConfig());
  // Since it is based on multi-threading, I used fcl, but with an environment of just one cube, it didn't crash with ODE, I wonder why?
  node->declare_parameter("collision_engine", "fcl");
  // To make the effects of multi-threading clear, turn off log file output that runs before and after processing
  node->declare_parameter("save_request", false);
  // Since I want to check that it works even with log printing, I will include this
  node->declare_parameter("print_debug_info", true);
  validator_serial->Initialize(node);

  auto collision_object_pub = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
      "collision_environment_server/transformed_environment", 1);
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube()};
  collision_object_pub->publish(environment);
  rclcpp::spin_some(node);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_in;
  trajectories_in.resize(500);
  for (uint32_t i = 0; i < trajectories_in.size(); ++i) {
    tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(
        GenerateTestDefaultTrajectory(), trajectories_in[i]);
  }

  const auto initial_state = GenerateDefaultRobotState();

  tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
  std::function<bool()> func = []() -> bool{ return false; };
  const auto start_serial = std::chrono::system_clock::now();
  const auto result = validator_serial->Validate(trajectories_in, initial_state, func, trajectory_out);
  const auto end_serial = std::chrono::system_clock::now();
  const auto elapsed_serial = std::chrono::duration_cast<std::chrono::nanoseconds>(end_serial - start_serial).count();
  EXPECT_FALSE(result);
  EXPECT_EQ(capture.GetRobotCollisionLogCount(), trajectories_in.size());

  auto validator_parallel = loader.createSharedInstance(
      "tmc_collision_detecting_validator/CollisionDetectingValidatorPlugin");
  node->set_parameter(rclcpp::Parameter("validation_thread_num", 8));
  validator_parallel->Initialize(node);

  collision_object_pub->publish(environment);
  rclcpp::spin_some(node);

  const auto start_parallel = std::chrono::system_clock::now();
  const auto result_parallel = validator_parallel->Validate(trajectories_in, initial_state, func, trajectory_out);
  const auto end_parallel = std::chrono::system_clock::now();
  const auto elapsed_parallel =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end_parallel - start_parallel).count();
  EXPECT_FALSE(result_parallel);
  EXPECT_EQ(capture.GetRobotCollisionLogCount(), 2 * trajectories_in.size());
  // With this setting, it took about 1/4 of the time on Ryzen 9 3950X, so I will set it to 1/2 for test stabilization
  EXPECT_GT(elapsed_serial / 2.0, elapsed_parallel);
}

// When there are multiple feasible trajectories in the given trajectory group, the first trajectory is selected in the multi-threaded implementation
TEST(CollisionDetectingValidatorMultiThreadTest, SelectFirstFeasibleTrajectory) {
  pluginlib::ClassLoader<tmc_robot_local_planner::IValidator> loader(
      "tmc_robot_local_planner", "tmc_robot_local_planner::IValidator");
  auto validator_serial = loader.createSharedInstance(
      "tmc_collision_detecting_validator/CollisionDetectingValidatorPlugin");

  auto node = rclcpp::Node::make_shared("plugins");
  node->declare_parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("robot_collision_pair", tmc_manipulation_tests::stanford_manipulator::GetCollisionConfig());
  node->declare_parameter("collision_engine", "fcl");
  node->declare_parameter("save_request", false);
  node->declare_parameter("print_debug_info", false);
  node->declare_parameter("validation_timeout", 1.0);
  node->declare_parameter("validation_thread_num", 2);
  validator_serial->Initialize(node);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_in;
  trajectories_in.resize(2);
  for (uint32_t i = 0; i < trajectories_in.size(); ++i) {
    tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(
        GenerateTestDefaultTrajectory(), trajectories_in[i]);
  }

  // Make the 0th trajectory extremely long
  for (uint32_t i = 0; i < 1000; ++i) {
    for (uint32_t j = 0; j < trajectories_in[1].joint_trajectory.points.size(); ++j) {
      trajectories_in[0].joint_trajectory.points.push_back(trajectories_in[1].joint_trajectory.points[j]);
      trajectories_in[0].multi_dof_joint_trajectory.points.push_back(
          trajectories_in[1].multi_dof_joint_trajectory.points[j]);
    }
  }

  // Since it also depends on the mood of multi-threading, repeat an appropriate number of times
  for (uint32_t i = 0; i < 10; ++i) {
    tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
    ASSERT_TRUE(validator_serial->Validate(
        trajectories_in, GenerateDefaultRobotState(), [](void) { return false; }, trajectory_out));
    ASSERT_EQ(trajectory_out.joint_trajectory.points.size(), trajectories_in[0].joint_trajectory.points.size());
  }
}


class CollisionDetectingValidatorTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

  template <typename FutureT>
  void WaitForFutureComplete(const typename std::shared_future<FutureT>& future) {
    while (future.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout) {
      SpinSome();
    }
  }

  tmc_planning_msgs::action::ValidateRobotTrajectories::Result::SharedPtr SendGoalSync(
    const std::vector<moveit_msgs::msg::RobotTrajectory>& robot_trajectory,
    int8_t expected) {
      return SendGoalSync(robot_trajectory, GenerateDefaultRobotState(), expected);
  }

  tmc_planning_msgs::action::ValidateRobotTrajectories::Result::SharedPtr SendGoalSync(
      const std::vector<moveit_msgs::msg::RobotTrajectory>& robot_trajectory,
      const tmc_manipulation_types::RobotState& initial_state,
      int8_t expected) {
    tmc_planning_msgs::action::ValidateRobotTrajectories::Goal goal;
    goal.robot_trajectories = robot_trajectory;
    tmc_manipulation_types_bridge::RobotStateToRobotStateMsg(initial_state, goal.initial_state);

    auto future_goal_handle = validate_client_->async_send_goal(goal);
    WaitForFutureComplete(future_goal_handle);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());

    auto future_result = validate_client_->async_get_result(goal_handle);
    WaitForFutureComplete(future_result);

    auto wrapped_result = future_result.get();
    EXPECT_EQ(static_cast<int8_t>(wrapped_result.code), expected);
    EXPECT_TRUE(wrapped_result.result);
    return wrapped_result.result;
  }

  void PublishAttachedObject() {
    moveit_msgs::msg::RobotState robot_state;
    robot_state.attached_collision_objects.resize(1);
    robot_state.attached_collision_objects[0].link_name = "link7";
    robot_state.attached_collision_objects[0].object.header.frame_id = "link7";
    robot_state.attached_collision_objects[0].object.id = "object2";
    robot_state.attached_collision_objects[0].object.pose.position.x = 0.0;
    robot_state.attached_collision_objects[0].object.pose.position.y = 0.0;
    robot_state.attached_collision_objects[0].object.pose.position.z = 0.15;
    robot_state.attached_collision_objects[0].object.primitives.resize(1);
    robot_state.attached_collision_objects[0].object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    robot_state.attached_collision_objects[0].object.primitives[0].dimensions = {0.05, 0.05, 0.05};
    robot_state.attached_collision_objects[0].object.primitive_poses.resize(1);
    attached_object_pub_->publish(robot_state);
    SpinSome();
  }

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<CollisionDetectingValidator> server_node_;

  rclcpp_action::Client<tmc_planning_msgs::action::ValidateRobotTrajectories>::SharedPtr validate_client_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr collision_object_pub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotState>::SharedPtr attached_object_pub_;
  rclcpp::Publisher<tmc_planning_msgs::msg::ObjectPairsList>::SharedPtr disabled_object_pairs_pub_;

  moveit_msgs::msg::RobotTrajectory robot_trajectory_;
  std::vector<moveit_msgs::msg::RobotTrajectory> robot_trajectories_;

  ScopedLogDir ros_log_dir_;
  RclcppOutputCapture rclcpp_output_capture_;
};

void CollisionDetectingValidatorTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf()),
      rclcpp::Parameter("robot_collision_pair", tmc_manipulation_tests::stanford_manipulator::GetCollisionConfig()),
      rclcpp::Parameter("save_request", true),
      rclcpp::Parameter("obstacle_map_topic_names", std::vector<std::string>({"static_map", "dynamic_map"}))};
  server_node_ = std::make_shared<CollisionDetectingValidator>(options);
  server_node_->Init();

  client_node_ = rclcpp::Node::make_shared("client");
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(client_node_);

  geometry_msgs::msg::TransformStamped map_to_odom_tf;
  map_to_odom_tf.header.frame_id = "map";
  map_to_odom_tf.child_frame_id = "odom";
  map_to_odom_tf.transform.translation.x = 1.0;
  tf_static_broadcaster_->sendTransform(map_to_odom_tf);

  // QoS is aligned with tmc_grid_map_server and tmc_map_merger
  static_map_pub_ = client_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "static_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  dynamic_map_pub_ = client_node_->create_publisher<nav_msgs::msg::OccupancyGrid>("dynamic_map", 10);

  collision_object_pub_ = client_node_->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
      "collision_environment_server/transformed_environment", 1);
  attached_object_pub_ = client_node_->create_publisher<moveit_msgs::msg::RobotState>(
      "attached_object_publisher/attached_object", 1);
  disabled_object_pairs_pub_ = client_node_->create_publisher<tmc_planning_msgs::msg::ObjectPairsList>(
      "validator/collisions_disabled_object_pairs", 1);

  validate_client_ = rclcpp_action::create_client<tmc_planning_msgs::action::ValidateRobotTrajectories>(
      client_node_, "validator/validate");
  while (!validate_client_->action_server_is_ready()) {
    SpinSome();
  }
  robot_trajectory_ = GenerateTestDefaultTrajectory();

  robot_trajectories_.clear();
  robot_trajectories_.push_back(robot_trajectory_);
}

// Action returns successfully without interference
TEST_F(CollisionDetectingValidatorTest, ValidatorSucceeded) {
  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  CheckTrajectory(result->robot_trajectory, robot_trajectory_);

  EXPECT_EQ(ros_log_dir_.GetLogFiles(".goal").size(), 1);
  EXPECT_EQ(ros_log_dir_.GetLogFiles(".env").size(), 1);
  EXPECT_EQ(ros_log_dir_.GetLogFiles(".attached").size(), 1);
  EXPECT_EQ(ros_log_dir_.GetLogFiles(".disabled").size(), 1);
  EXPECT_EQ(ros_log_dir_.GetLogFiles(".indices").size(), 1);
}

// Log files are not output
TEST_F(CollisionDetectingValidatorTest, NoLogFiles) {
  server_node_->set_parameter(rclcpp::Parameter("save_request", false));
  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  CheckTrajectory(result->robot_trajectory, robot_trajectory_);

  EXPECT_TRUE(ros_log_dir_.GetLogFiles(".goal").empty());
  EXPECT_TRUE(ros_log_dir_.GetLogFiles(".env").empty());
  EXPECT_TRUE(ros_log_dir_.GetLogFiles(".attached").empty());
  EXPECT_TRUE(ros_log_dir_.GetLogFiles(".disabled").empty());
  EXPECT_TRUE(ros_log_dir_.GetLogFiles(".indices").empty());
}

// When there are interfering and non-interfering trajectories, the non-interfering trajectory is selected
TEST_F(CollisionDetectingValidatorTest, ValidateFeasibleTrajectory) {
  // Insert interfering trajectory at the beginning
  robot_trajectories_.insert(robot_trajectories_.begin(), GenerateOnePointRobotTrajectory(0.0, 0.0));
  robot_trajectories_[0].joint_trajectory.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  robot_trajectories_.push_back(robot_trajectories_[0]);

  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  CheckTrajectory(result->robot_trajectory, robot_trajectory_);

  const auto indices_log_file = ros_log_dir_.GetLatestLogFileFullPath(".indices");
  ASSERT_TRUE(indices_log_file);
  std_msgs::msg::UInt32MultiArray validation_indices;
  ASSERT_TRUE(tmc_utils::LoadMsg(*indices_log_file, validation_indices));
  // 0th and 1st trajectories are verified
  // There is also a possibility that the 2nd trajectory is verified depending on the mood of the thread
  EXPECT_GE(validation_indices.data.size(), 2);
  EXPECT_EQ(validation_indices.data[0], 0);
  EXPECT_EQ(validation_indices.data[1], 1);
}

// Works even with only cart trajectory
TEST_F(CollisionDetectingValidatorTest, ValidateOnlyBaseTrajectory) {
  robot_trajectories_[0].joint_trajectory.joint_names.clear();
  robot_trajectories_[0].joint_trajectory.points.clear();

  auto initial_state = GenerateDefaultRobotState();
  (void)SendGoalSync(robot_trajectories_, initial_state, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  // Should become interfering
  initial_state.joint_state.position = Eigen::VectorXd::Zero(initial_state.joint_state.position.size());
  (void)SendGoalSync(robot_trajectories_, initial_state, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// Interference check with static map
TEST_F(CollisionDetectingValidatorTest, StaticMapCollision) {
  const auto map = GenerateMap();
  static_map_pub_->publish(map);
  SpinSome();

  // Set to a non-collision location
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x = -1.0;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y = -2.0;

  // Since it is outside the map, it is judged as non-collision
  {
    SCOPED_TRACE("Outside of map");
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.1;
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -6.0;
    SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
    EXPECT_EQ(rclcpp_output_capture_.GetMapCollisionLogCount(), 0);
  }

  // Inside the map but in an area without obstacles, so no collision
  {
    SCOPED_TRACE("Not in collision");
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = -0.1;
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -5.9;
    SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
    EXPECT_EQ(rclcpp_output_capture_.GetMapCollisionLogCount(), 0);
  }

  // In an area with obstacles, so collision occurs
  {
    SCOPED_TRACE("In collision");
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = -0.1;
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -6.1;
    SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
    EXPECT_EQ(rclcpp_output_capture_.GetMapCollisionLogCount(), 1);
  }
}

// Interference check with dynamic map
TEST_F(CollisionDetectingValidatorTest, DynamicMapCollision) {
  // Just want to check if it's okay even if QoS is different, so check only when collision occurs
  const auto map = GenerateMap();
  dynamic_map_pub_->publish(map);
  SpinSome();

  // Set to a non-collision location
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x = -1.0;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y = -2.0;

  // In an area with obstacles, so collision occurs
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = -0.1;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -6.1;
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// Trajectory interferes with known objects
TEST_F(CollisionDetectingValidatorTest, RobotCollision) {
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube()};
  collision_object_pub_->publish(environment);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
  EXPECT_EQ(rclcpp_output_capture_.GetRobotCollisionLogCount(), 1);

  // Exclude collision judgment by object specification
  tmc_planning_msgs::msg::ObjectPairsList object_pairs_list;
  object_pairs_list.pairs.resize(2);
  object_pairs_list.pairs[0].names = {"link1/collision/0", "object1"};
  object_pairs_list.pairs[1].names = {"link1/collision/1", "object1"};
  disabled_object_pairs_pub_->publish(object_pairs_list);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  EXPECT_EQ(rclcpp_output_capture_.GetRobotCollisionLogCount(), 1);

  // Restore collision judgment settings
  object_pairs_list.pairs.clear();
  disabled_object_pairs_pub_->publish(object_pairs_list);
  SpinSome();

  server_node_->set_parameter(rclcpp::Parameter("print_debug_info", false));
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
  EXPECT_EQ(rclcpp_output_capture_.GetRobotCollisionLogCount(), 1);

  // Exclude collision judgment by object index specification
  object_pairs_list.pairs.resize(2);
  object_pairs_list.pairs[0].names = {"link1/collision/0", "object1#0"};
  object_pairs_list.pairs[1].names = {"link1/collision/1", "object1#0"};
  disabled_object_pairs_pub_->publish(object_pairs_list);
  SpinSome();

  server_node_->set_parameter(rclcpp::Parameter("print_debug_info", true));
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  EXPECT_EQ(rclcpp_output_capture_.GetRobotCollisionLogCount(), 1);
}

// Grasped objects not in the environment are reflected in collision judgment
TEST_F(CollisionDetectingValidatorTest, AttachedObjectNotInEnvironment) {
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube()};
  // Position/size that barely doesn't collide with the arm but collides with the grasped object
  environment.collision_objects[0].pose.position.y = 0.25;
  environment.collision_objects[0].pose.position.z = 1.40;
  environment.collision_objects[0].primitives[0].dimensions = {0.05, 0.05, 0.05};
  collision_object_pub_->publish(environment);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  // Add grasped object
  PublishAttachedObject();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// Grasped objects in the environment are reflected in collision judgment with relative position/orientation to the grasp frame
TEST_F(CollisionDetectingValidatorTest, AttachedObjectInEnvironment) {
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube(), GenerateOneObstacleCube()};
  // Position/size that barely doesn't collide with the arm
  environment.collision_objects[0].pose.position.y = 0.25;
  environment.collision_objects[0].pose.position.z = 1.40;
  environment.collision_objects[0].primitives[0].dimensions = {0.05, 0.05, 0.05};
  // Move the grasped object to an outrageous location
  environment.collision_objects[1].id = "object2";
  environment.collision_objects[1].pose.position.z = 10.0;
  environment.collision_objects[1].primitives[0].dimensions = {0.05, 0.05, 0.05};
  collision_object_pub_->publish(environment);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  // Relative position/orientation to the grasp frame is prioritized
  PublishAttachedObject();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// Function to escape from initial state collision
TEST_F(CollisionDetectingValidatorTest, EscapeInitialCollision) {
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube(), GenerateOneObstacleCube()};
  environment.collision_objects[0].pose.position.x = 0.0;
  environment.collision_objects[0].pose.position.y = 0.0;
  environment.collision_objects[0].primitives[0].dimensions = {0.1, 0.1, 0.1};
  environment.collision_objects[1].id = "object2";
  environment.collision_objects[1].pose.position.x = 1.0;
  environment.collision_objects[1].pose.position.y = 0.0;
  environment.collision_objects[1].primitives[0].dimensions = {0.1, 0.1, 0.1};
  collision_object_pub_->publish(environment);
  SpinSome();

  // 0th and 1st points collide, collision becomes shallower
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.1;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = 0.0;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x = 0.2;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y = 0.0;
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);

  // Add a non-collision 2nd point
  robot_trajectories_[0].joint_trajectory.points.push_back(robot_trajectories_[0].joint_trajectory.points.back());
  robot_trajectories_[0].multi_dof_joint_trajectory.points.push_back(
      robot_trajectories_[0].multi_dof_joint_trajectory.points.back());
  robot_trajectories_[0].multi_dof_joint_trajectory.points[2].transforms[0].translation.x = 0.3;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[2].transforms[0].translation.y = 0.0;
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  // Collide with another object at the 1st point
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x = 0.8;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y = 0.0;
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);

  // Collide with the same object at 0th and 1st points, escape from collision at 2nd point, re-collide at 3rd point
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.1;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = 0.0;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x = 0.2;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y = 0.0;
  robot_trajectories_[0].joint_trajectory.points.push_back(robot_trajectories_[0].joint_trajectory.points.back());
  robot_trajectories_[0].multi_dof_joint_trajectory.points.push_back(
      robot_trajectories_[0].multi_dof_joint_trajectory.points.back());
  robot_trajectories_[0].multi_dof_joint_trajectory.points[3].transforms[0].translation.x = 0.1;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[3].transforms[0].translation.y = 0.0;
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}


// // Fail due to timeout
// // 200 postures that take 5 msec per posture on Ryzen 9 3950x
// TEST_F(CollisionDetectingValidatorTest, TimeoutValidate) {
//   // Generate known object objects
//   tmc_manipulation_msgs::CollisionObject obstacle = GenerateObstacleCubes(1, 0.1);
//   collision_object_pub_.publish(obstacle);

//   tmc_manipulation_msgs::GetCollisionEnvironment get_environment_srv;
//   get_environment_srv.request.known_object_only = true;
//   get_environment_srv.request.origin_frame_id = "odom";
//   auto exists_object = [&] {
//     ros::service::call("get_collision_environment", get_environment_srv);
//     return get_environment_srv.response.environment.known_objects.size() != 0;
//   };
//   ASSERT_TRUE(tmc_rostest_utils::WaitUntil(exists_object, 5.0));

//   // Insert a large interfering trajectory to cause timeout
//   tmc_planning_msgs::ValidateRobotTrajectoriesGoal goal;
//   goal.robot_trajectories.push_back(GenerateRobotTrajectory(1.0, 0.5, 200));

//   validator_->sendGoal(goal);
//   while (validator_->getState().state_ != actionlib::SimpleClientGoalState::ACTIVE) {}
//   auto start = ros::Time::now();
//   while (validator_->getState().state_ != actionlib::SimpleClientGoalState::ABORTED) {}
//   double elapsed = (ros::Time::now() - start).toSec();
//   // When done locally, it was about 0.18 sec, but when run on Jenkins, it was about 0.5 sec, so I will put in double the value
//   EXPECT_GT(elapsed, 0.15);
//   EXPECT_LT(elapsed, 1.0);

//   // tear down
//   obstacle.operation.operation = tmc_manipulation_msgs::CollisionObjectOperation::REMOVE;
//   collision_object_pub_.publish(obstacle);
//   EXPECT_TRUE(tmc_rostest_utils::WaitUntil([&]() { return !exists_object(); }, 5.0));
// }

// Fail when trajectory is empty
TEST_F(CollisionDetectingValidatorTest, EmptyTrajectory) {
  tmc_planning_msgs::action::ValidateRobotTrajectories::Goal goal;

  auto future_goal_handle = validate_client_->async_send_goal(goal);
  WaitForFutureComplete(future_goal_handle);

  auto goal_handle = future_goal_handle.get();
  EXPECT_FALSE(goal_handle.get());
}

}  // namespace tmc_collision_detecting_validator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
