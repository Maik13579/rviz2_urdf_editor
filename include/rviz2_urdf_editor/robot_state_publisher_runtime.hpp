// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace robot_state_publisher {
class RobotStatePublisher;
}

namespace rviz2_urdf_editor {

struct RobotStatePublisherSettings {
  std::string node_name{"dashboard_robot_state_publisher"};
  std::string node_namespace{""};
  std::string robot_description_topic{"/robot_description"};
  std::string joint_states_topic{"/joint_states"};
  std::string frame_prefix{""};
  double publish_frequency{20.0};
  bool ignore_timestamp{false};
};

struct RobotStatePublisherSnapshot {
  bool enabled{false};
  bool running{false};
  RobotStatePublisherSettings settings;
  std::vector<std::string> arguments;
  std::string status{"Disabled."};
  std::string last_error;
};

class RobotStatePublisherRuntime {
public:
  static RobotStatePublisherRuntime &instance();

  RobotStatePublisherSnapshot snapshot() const;
  bool apply(const RobotStatePublisherSettings &settings, bool enabled);
  bool restartWithRobotDescription(const std::string &robot_description);
  void stop();

private:
  RobotStatePublisherRuntime() = default;
  ~RobotStatePublisherRuntime();

  bool startLocked();
  void stopLocked();

  mutable std::mutex mutex_;
  RobotStatePublisherSnapshot state_;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> rsp_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::string startup_robot_description_;
  std::atomic_bool stop_requested_{false};
};

} // namespace rviz2_urdf_editor
