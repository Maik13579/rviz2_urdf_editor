// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include "rviz2_urdf_editor/urdf_editor_state.hpp"

namespace rviz2_urdf_editor {

class UrdfTfTreeMarkerServer {
public:
  static constexpr const char *kMarkerNamespace = "/rviz2_urdf_editor/tf_tree";

  static UrdfTfTreeMarkerServer &instance();

  void start(const rclcpp::Node::SharedPtr &node);
  void stop();

  static std::vector<visualization_msgs::msg::InteractiveMarker> buildMarkers(
      const UrdfEditorSnapshot &snapshot,
      const std::map<std::string, geometry_msgs::msg::TransformStamped>
          &joint_transforms = {});

  void handleFeedbackForTesting(
      const visualization_msgs::msg::InteractiveMarkerFeedback &feedback);

private:
  UrdfTfTreeMarkerServer() = default;

  void rebuildFromState();
  void refreshJointMarkers();
  std::map<std::string, geometry_msgs::msg::TransformStamped>
  lookupJointTransforms(const UrdfEditorSnapshot &snapshot) const;
  void handleFeedback(
      visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  std::uint64_t state_callback_id_{0};
  std::uint64_t seen_revision_{0};
  std::string seen_selected_link_;
  std::string seen_selected_joint_;
  std::string seen_highlight_color_;
  std::string seen_tf_joint_color_;
  std::map<std::string, geometry_msgs::msg::TransformStamped>
      last_joint_transforms_;
};

} // namespace rviz2_urdf_editor
