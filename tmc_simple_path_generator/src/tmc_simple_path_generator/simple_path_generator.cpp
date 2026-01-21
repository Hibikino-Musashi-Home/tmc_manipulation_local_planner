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
/// @brief Simple Path Generator Class

#include <tmc_simple_path_generator/simple_path_generator.hpp>

#include <rclcpp_action/create_server.hpp>

#include <tmc_manipulation_types/utils.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>

namespace {
// Coordinate transformation timeout
const double kTFTimeout = 0.05;
// Origin frame
const char* const kOriginFrame = "odom";
// Retry count under specific conditions
const uint32_t kMaxRetryCount = 10;

bool CheckRobotState(const moveit_msgs::msg::RobotState robot_state) {
  if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size() ||
      robot_state.joint_state.name.size() != robot_state.joint_state.velocity.size()) {
    return false;
  }
  if (robot_state.multi_dof_joint_state.joint_names.size() != robot_state.multi_dof_joint_state.transforms.size() ||
      robot_state.multi_dof_joint_state.joint_names.size() != robot_state.multi_dof_joint_state.twist.size()) {
    return false;
  }
  return true;
}
}  // namespace


namespace tmc_simple_path_generator {

struct Conditions {
  bool sample_from_random_state;
  bool use_random_middle_state;
  bool sample_from_random_on_initial_linear_constraint;
  uint32_t remaining_retries;

  // Default constructor is the most common condition
  Conditions() : sample_from_random_state(true),
                 use_random_middle_state(true),
                 sample_from_random_on_initial_linear_constraint(true),
                 remaining_retries(kMaxRetryCount) {}

  Conditions(bool _sample_from_random_state,
             bool _use_random_middle_state,
             bool _sample_from_random_on_initial_linear_constraint)
      : sample_from_random_state(_sample_from_random_state),
        use_random_middle_state(_use_random_middle_state),
        sample_from_random_on_initial_linear_constraint(_sample_from_random_on_initial_linear_constraint),
        remaining_retries(kMaxRetryCount) {}
};

class ConditionsQueue {
 public:
  ConditionsQueue(int64_t max_simple_trajectory_num, bool use_initial_motion_linear_constraint) {
    // Always include target state sampling from initial posture
    conditions_queue_.push(Conditions(false, false, false));
    conditions_queue_.push(Conditions(false, true, false));
    if (use_initial_motion_linear_constraint) {
      conditions_queue_.push(Conditions(false, false, true));
      conditions_queue_.push(Conditions(false, true, true));
    }

    // Solve without using random intermediate states up to max_simple_trajectory_num
    if (use_initial_motion_linear_constraint) {
      for (int64_t i = 1; i < max_simple_trajectory_num / 2; ++i) {
        conditions_queue_.push(Conditions(true, false, false));
        conditions_queue_.push(Conditions(true, false, true));
      }
    } else {
      for (int64_t i = 1; i < max_simple_trajectory_num; ++i) {
        conditions_queue_.push(Conditions(true, false, false));
      }
    }
  }

  Conditions Get() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (conditions_queue_.empty()) {
      return Conditions();
    }
    const auto conditions = conditions_queue_.front();
    conditions_queue_.pop();
    return conditions;
  }

 private:
  std::queue<Conditions> conditions_queue_;
  std::mutex mutex_;
};


class Trajectories {
 public:
  explicit Trajectories(int64_t max_trajectory_num) : max_trajectory_num_(max_trajectory_num) {}

  bool IsExceedingMaxSize() {
    std::lock_guard<std::mutex> lock(mutex_);
    return (max_trajectory_num_ > 0 && trajectories_.size() >= max_trajectory_num_);
  }
  void Add(const tmc_manipulation_types::TimedRobotTrajectory& trajectory) {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectories_.push_back(trajectory);
  }
  void Swap(std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectories_out.swap(trajectories_);
  }

 private:
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_;
  int64_t max_trajectory_num_;

  std::mutex mutex_;
};

class SimplePathGeneratorPlugin::SimplePathGeneratorImpl {
 public:
  SimplePathGeneratorImpl(const rclcpp::Node::SharedPtr& node, const SamplingParameters::Ptr& sampling_params);
  virtual ~SimplePathGeneratorImpl() = default;

  void Generate(const tmc_robot_local_planner::Constraints& constraints,
                const tmc_manipulation_types::RobotState& initial_state,
                double normalized_velocity,
                const SamplingParameters::Ptr& sampling_params,
                std::function<bool()> interrupt,
                ConditionsQueue& conditions_queue,
                Trajectories& trajectories_out);

 private:
  // SampleGoalGenerator loader
  pluginlib::ClassLoader<SampleGoalGeneratorBase> sample_goal_generator_loader_;
  // SampleGoalGenerator instance
  SampleGoalGeneratorBase::Ptr sample_goal_generator_;

  // SampleMiddleGenerator loader
  pluginlib::ClassLoader<ISampleMiddleGenerator> sample_middle_generator_loader_;
  // SampleMiddleGenerator instance
  ISampleMiddleGenerator::Ptr sample_middle_generator_;

  // TrajectoryInterpolator loader
  pluginlib::ClassLoader<ITrajectoryInterpolator> trajectory_interpolator_loader_;
  // TrajectoryInterpolator instance
  ITrajectoryInterpolator::Ptr trajectory_interpolator_;

  // Path constraints for the specified frame
  HardPathLinkConstraints::Ptr hard_path_link_constraints_;
  InitialMotionLinearConstraint::Ptr initial_motion_linear_constraint_;
  GoalRelativeLinearConstraint::Ptr goal_relative_linear_constraint_;
};

SimplePathGeneratorPlugin::SimplePathGeneratorImpl::SimplePathGeneratorImpl(
    const rclcpp::Node::SharedPtr& node, const SamplingParameters::Ptr& sampling_params)
    : sample_goal_generator_loader_("tmc_simple_path_generator",
                                    "tmc_simple_path_generator::SampleGoalGeneratorBase"),
      sample_middle_generator_loader_("tmc_simple_path_generator",
                                      "tmc_simple_path_generator::ISampleMiddleGenerator"),
      trajectory_interpolator_loader_("tmc_simple_path_generator",
                                      "tmc_simple_path_generator::ITrajectoryInterpolator") {
  const auto goal_sampler_type = tmc_utils::GetParameter<std::string>(
      node, "goal_sampler_type", "tmc_simple_path_generator/SampleGoalGenerator");
  sample_goal_generator_ = sample_goal_generator_loader_.createSharedInstance(goal_sampler_type);
  sample_goal_generator_->Initialize(node);

  // Use the specified plugin if specified
  // Switch class based on base_movement_type if not specified
  const auto middle_sampler_type = tmc_utils::GetParameter<std::string>(node, "middle_sampler_type", "");
  if (middle_sampler_type.empty()) {
    if (sampling_params->base_movement_type == tmc_manipulation_types::kPlanar) {
      sample_middle_generator_ = sample_middle_generator_loader_.createSharedInstance(
          "tmc_simple_path_generator/PlanerMiddleStateGenerator");
    } else {
      RCLCPP_FATAL(node->get_logger(), "Unknown base movement type");
      exit(EXIT_FAILURE);
    }
  } else {
    sample_middle_generator_ = sample_middle_generator_loader_.createSharedInstance(middle_sampler_type);
  }
  sample_middle_generator_->Initialize(node);

  const auto interpolation_type = tmc_utils::GetParameter<std::string>(node, "interpolation_type", "");
  if (interpolation_type.empty()) {
    if (sampling_params->base_movement_type == tmc_manipulation_types::kPlanar) {
      trajectory_interpolator_ = trajectory_interpolator_loader_.createSharedInstance(
          "tmc_simple_path_generator/PlanarTrajectoryInterpolator");
    } else {
      RCLCPP_FATAL(node->get_logger(), "Unknown base movement type");
      exit(EXIT_FAILURE);
    }
  } else {
    trajectory_interpolator_ = trajectory_interpolator_loader_.createSharedInstance(interpolation_type);
  }
  trajectory_interpolator_->Initialize(node, sampling_params);

  hard_path_link_constraints_ = std::make_shared<HardPathLinkConstraints>(node);
  initial_motion_linear_constraint_ = std::make_shared<InitialMotionLinearConstraint>(node);
  goal_relative_linear_constraint_ = std::make_shared<GoalRelativeLinearConstraint>(node);
}

void SimplePathGeneratorPlugin::SimplePathGeneratorImpl::Generate(
    const tmc_robot_local_planner::Constraints& constraints,
    const tmc_manipulation_types::RobotState& initial_state,
    double normalized_velocity,
    const SamplingParameters::Ptr& sampling_params,
    std::function<bool()> interrupt,
    ConditionsQueue& conditions_queue,
    Trajectories& trajectories_out) {
  Conditions conditions;
  conditions.remaining_retries = 0;

  while (!interrupt()) {
    // Termination judgment
    if (trajectories_out.IsExceedingMaxSize()) {
      break;
    }
    // Condition setting for using random posture
    if (conditions.remaining_retries > 0) {
      --conditions.remaining_retries;
    } else {
      conditions = conditions_queue.Get();
    }
    sample_goal_generator_->set_is_from_random_initial_state(conditions.sample_from_random_state);
    initial_motion_linear_constraint_->set_is_from_random_state(
        conditions.sample_from_random_on_initial_linear_constraint);

    // Sampling of goal posture
    auto goal_state = initial_state;
    if (!constraints.hard_joint_constraints.empty()) {
      if (!sample_goal_generator_->SampleFromJointConstraints(constraints.hard_joint_constraints, goal_state)) {
        continue;
      }
    }
    if (!constraints.hard_link_constraints.empty()) {
      if (!sample_goal_generator_->SampleFromLinkConstraints(
              initial_state, constraints.hard_link_constraints, sampling_params, goal_state)) {
        continue;
      }
    }
    // Extract initial posture consistent with goal posture joints
    auto initial_state_impl = initial_state;
    initial_state_impl.joint_state = tmc_manipulation_types::ExtractPartialJointState(
        initial_state.joint_state, goal_state.joint_state.name);
    if (initial_state_impl.joint_state.name.size() != goal_state.joint_state.name.size()) {
      continue;
    }

    // Linear path constraints at the start of movement
    const auto middle_states_near_initial = initial_motion_linear_constraint_->SampleLinearPath(
        constraints.initial_motion_linear_constraint, initial_state, sampling_params);
    if (!middle_states_near_initial) {
      continue;
    }
    // Sampling of intermediate posture
    std::optional<tmc_manipulation_types::RobotState> random_middle_state;
    if (conditions.use_random_middle_state) {
      if (middle_states_near_initial.value().empty()) {
        random_middle_state = sample_middle_generator_->ComputeRandomMiddleState(
            initial_state_impl, goal_state, sampling_params);
      } else {
        random_middle_state = sample_middle_generator_->ComputeRandomMiddleState(
            middle_states_near_initial.value().back(), goal_state, sampling_params);
      }
      if (!random_middle_state) {
        continue;
      }
    }
    // Linear path constraints relative to goal
    auto initial_state_for_goal_relative = initial_state;
    if (random_middle_state) {
      initial_state_for_goal_relative = random_middle_state.value();
    } else if (middle_states_near_initial) {
      if (!middle_states_near_initial.value().empty()) {
        initial_state_for_goal_relative = middle_states_near_initial.value().back();
      }
    }
    const auto middle_states_near_goal = goal_relative_linear_constraint_->SampleLinearPath(
        constraints.goal_relative_linear_constraint, initial_state_for_goal_relative, goal_state, sampling_params);
    if (!middle_states_near_goal) {
      continue;
    }
    // Generate trajectory by summarizing waypoints
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    if (random_middle_state) {
      if (!trajectory_interpolator_->Interpolate(
              initial_state_impl, middle_states_near_initial.value(), middle_states_near_goal.value(), goal_state,
              normalized_velocity, random_middle_state.value(), trajectory)) {
        continue;
      }
    } else {
      if (!trajectory_interpolator_->Interpolate(
              initial_state_impl, middle_states_near_initial.value(), middle_states_near_goal.value(), goal_state,
              normalized_velocity, trajectory)) {
        continue;
      }
    }
    // Link path constraints
    if (!constraints.hard_path_link_constraints.empty()) {
      if (!hard_path_link_constraints_->ConstrainTrajectory(constraints.hard_path_link_constraints, sampling_params,
                                                            middle_states_near_initial.value().size(),
                                                            middle_states_near_goal.value().size(),
                                                            trajectory)) {
        continue;
      }
    }
    trajectories_out.Add(trajectory);
    // Successfully generated trajectory, set retry count to zero (no more retries)
    conditions.remaining_retries = 0;
  }
  sample_goal_generator_->Terminate();
}


void SimplePathGeneratorPlugin::Initialize(const rclcpp::Node::SharedPtr& node) {
  clock_ = node->get_clock();

  generate_timeout_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "generator_timeout", 0.03);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback,
                generate_timeout_, std::placeholders::_1)));

  max_simple_trajectory_num_ = std::make_shared<tmc_utils::DynamicParameter<int64_t>>(
      node, "max_simple_trajectory_num", 5);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<int64_t>::SetParameterCallback,
                max_simple_trajectory_num_, std::placeholders::_1)));

  max_trajectory_num_ = std::make_shared<tmc_utils::DynamicParameter<int64_t>>(
      node, "max_trajectory_num", 0);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<int64_t>::SetParameterCallback,
                max_trajectory_num_, std::placeholders::_1)));

  sampling_params_ = std::make_shared<SamplingParameters>(node);

  const auto generation_thread_num = tmc_utils::GetParameter<int64_t>(node, "generation_thread_num", 1);
  if (generation_thread_num <= 0) {
    RCLCPP_FATAL(node->get_logger(), "generation_thread_num must be greater than 0");
    exit(EXIT_FAILURE);
  }
  for (int64_t i = 0; i < generation_thread_num; ++i) {
    impls_.emplace_back(std::make_shared<SimplePathGeneratorImpl>(node, sampling_params_));
  }
}

bool SimplePathGeneratorPlugin::Generate(const tmc_robot_local_planner::Constraints& constraints,
                                         const tmc_manipulation_types::RobotState& initial_state,
                                         double normalized_velocity,
                                         bool enable_base,
                                         std::function<bool()> interrupt,
                                         std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) {
  sampling_params_->UpdateBaseMovementType(enable_base);

  auto end_stamp = clock_->now() + std::chrono::duration<double>(generate_timeout_->value());
  std::function<bool()> interrupt_with_timeout = [&]() {
    return interrupt() || clock_->now() >= end_stamp;
  };

  // Generate
  Trajectories trajectories(max_trajectory_num_->value());
  ConditionsQueue conditions_queue(max_simple_trajectory_num_->value(),
                                   !constraints.initial_motion_linear_constraint.end_frame_id.empty());
  std::vector<std::thread> threads;
  for (auto i = 0u; i < impls_.size(); ++i) {
    threads.emplace_back(std::thread(&SimplePathGeneratorImpl::Generate, impls_[i], constraints, initial_state,
                                     normalized_velocity, sampling_params_, interrupt_with_timeout,
                                     std::ref(conditions_queue), std::ref(trajectories)));
  }
  for (auto& thread : threads) {
    thread.join();
  }
  trajectories.Swap(trajectories_out);

  return !trajectories_out.empty();
}

SimplePathGenerator::SimplePathGenerator() : SimplePathGenerator(rclcpp::NodeOptions()) {}

SimplePathGenerator::SimplePathGenerator(const rclcpp::NodeOptions& options)
    : Node("generator", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_), impl_() {}

void SimplePathGenerator::Initialize() {
  const auto node = shared_from_this();
  impl_.Initialize(node);

  origin_frame_ = tmc_utils::GetParameter<std::string>(node, "origin_frame", kOriginFrame);
  timeout_ = tmc_utils::GetParameter<double>(node, "timeout", kTFTimeout);

  server_ = rclcpp_action::create_server<tmc_planning_msgs::action::GenerateRobotTrajectories>(
      this, "~/generate",
      std::bind(&SimplePathGenerator::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SimplePathGenerator::CancelCallback, this, std::placeholders::_1),
      std::bind(&SimplePathGenerator::FeedbackSetupCallback, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse SimplePathGenerator::GoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const tmc_planning_msgs::action::GenerateRobotTrajectories::Goal> goal) {
  if (!CheckRobotState(goal->initial_state)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->constraints.hard_link_constraints.empty() && goal->constraints.hard_joint_constraints.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Constraint isn't set");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SimplePathGenerator::CancelCallback(const ServerGoalHandlePtr goal_handle) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SimplePathGenerator::FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
  std::thread{std::bind(&SimplePathGenerator::Execute, this, std::placeholders::_1), goal_handle}.detach();
}

void SimplePathGenerator::Execute(const ServerGoalHandlePtr goal_handle) {
  const auto goal = goal_handle->get_goal();

  tmc_robot_local_planner::Constraints constraints;
  tmc_planning_msgs::msg::Constraints goal_constraints = goal->constraints;
  if (tmc_robot_local_planner_utils::TransformConstraints(origin_frame_, timeout_, tf_buffer_, goal_constraints)) {
    tmc_robot_local_planner_utils::ConvertConstraints(goal_constraints, constraints);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform");
    auto action_result = std::make_shared<tmc_planning_msgs::action::GenerateRobotTrajectories::Result>();
    goal_handle->abort(action_result);
    return;
  }

  tmc_manipulation_types::RobotState initial_state;
  tmc_manipulation_types_bridge::RobotStateMsgToRobotState(goal->initial_state, initial_state);

  auto interrupt_func = [goal_handle]() { return goal_handle->is_canceling(); };
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories;
  const auto gen_result = impl_.Generate(constraints, initial_state, goal->normalized_velocity, goal->enable_base,
                                         interrupt_func, trajectories);
  auto action_result = std::make_shared<tmc_planning_msgs::action::GenerateRobotTrajectories::Result>();
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(action_result);
  } else if (gen_result) {
    action_result->robot_trajectories.reserve(trajectories.size());
    for (const auto trajectory : trajectories) {
      moveit_msgs::msg::RobotTrajectory trajectory_msg;
      tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(trajectory, trajectory_msg);
      action_result->robot_trajectories.emplace_back(trajectory_msg);
    }
    goal_handle->succeed(action_result);
  } else {
    goal_handle->abort(action_result);
  }
}

}  // namespace tmc_simple_path_generator

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::SimplePathGeneratorPlugin,
                       tmc_robot_local_planner::IGenerator)
