#include "rviz2_urdf_editor/robot_state_publisher_runtime.hpp"

#include <chrono>

#include <robot_state_publisher/robot_state_publisher.hpp>

#include "rviz2_urdf_editor/urdf_editor_state.hpp"

namespace rviz2_urdf_editor {

namespace {

std::string robotDescriptionForStartup(const std::string &override_text) {
  if (!override_text.empty()) {
    return override_text;
  }
  const auto snapshot = UrdfEditorState::instance().snapshot();
  if (!snapshot.expanded_urdf.empty()) {
    return snapshot.expanded_urdf;
  }
  return "<robot name=\"dashboard_empty\"><link name=\"base_link\"/></robot>";
}

} // namespace

RobotStatePublisherRuntime &RobotStatePublisherRuntime::instance() {
  static RobotStatePublisherRuntime runtime;
  return runtime;
}

RobotStatePublisherRuntime::~RobotStatePublisherRuntime() { stop(); }

RobotStatePublisherSnapshot RobotStatePublisherRuntime::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool RobotStatePublisherRuntime::apply(
    const RobotStatePublisherSettings &settings, const bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  stopLocked();
  state_.settings = settings;
  state_.enabled = enabled;
  state_.last_error.clear();
  if (!enabled) {
    state_.status = "Disabled.";
    return true;
  }
  return startLocked();
}

bool RobotStatePublisherRuntime::restartWithRobotDescription(
    const std::string &robot_description) {
  std::lock_guard<std::mutex> lock(mutex_);
  startup_robot_description_ = robot_description;
  if (!state_.enabled) {
    return true;
  }
  stopLocked();
  return startLocked();
}

void RobotStatePublisherRuntime::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  stopLocked();
  state_.enabled = false;
  state_.status = "Disabled.";
}

bool RobotStatePublisherRuntime::startLocked() {
  try {
    std::vector<std::string> arguments = {
        "--ros-args",
        "-r", "__node:=" + state_.settings.node_name,
        "-r", "joint_states:=" + state_.settings.joint_states_topic,
        "--log-level", "warn",
    };
    if (!state_.settings.node_namespace.empty()) {
      arguments.push_back("-r");
      arguments.push_back("__ns:=" + state_.settings.node_namespace);
    }
    state_.arguments = arguments;
    rclcpp::get_logger(state_.settings.node_name)
        .set_level(rclcpp::Logger::Level::Warn);
    rclcpp::get_logger("robot_state_publisher")
        .set_level(rclcpp::Logger::Level::Warn);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(false);
    options.automatically_declare_parameters_from_overrides(false);
    options.append_parameter_override("robot_description",
                                      robotDescriptionForStartup(startup_robot_description_));
    options.append_parameter_override("frame_prefix", state_.settings.frame_prefix);
    options.append_parameter_override("publish_frequency",
                                      state_.settings.publish_frequency);
    options.append_parameter_override("ignore_timestamp",
                                      state_.settings.ignore_timestamp);
    options.arguments(arguments);

    rsp_node_ =
        std::make_shared<robot_state_publisher::RobotStatePublisher>(options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(rsp_node_);
    stop_requested_.store(false);
    spin_thread_ = std::thread([this, executor = executor_]() {
      while (!stop_requested_.load()) {
        executor->spin_some(std::chrono::milliseconds(50));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });
    state_.running = true;
    state_.status = "Running as " + state_.settings.node_name + ".";
    return true;
  } catch (const std::exception &error) {
    state_.running = false;
    state_.last_error = error.what();
    state_.status = state_.last_error;
    return false;
  }
}

void RobotStatePublisherRuntime::stopLocked() {
  stop_requested_.store(true);
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && rsp_node_) {
    executor_->remove_node(rsp_node_);
  }
  rsp_node_.reset();
  executor_.reset();
  state_.running = false;
}

} // namespace rviz2_urdf_editor
