// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "rviz2_urdf_editor/urdf_tf_tree_marker_server.hpp"

#include <algorithm>
#include <cmath>

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rviz2_urdf_editor {
namespace {

constexpr uint32_t kEditMenuEntryId = 1;

geometry_msgs::msg::Pose identityPose() {
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}

std_msgs::msg::ColorRGBA color(const float r, const float g, const float b,
                               const float a = 1.0F) {
  std_msgs::msg::ColorRGBA result;
  result.r = r;
  result.g = g;
  result.b = b;
  result.a = a;
  return result;
}

std_msgs::msg::ColorRGBA withAlpha(std_msgs::msg::ColorRGBA result,
                                   const double alpha_multiplier) {
  result.a *= static_cast<float>(std::clamp(alpha_multiplier, 0.0, 1.0));
  return result;
}

std_msgs::msg::ColorRGBA highlightColor(const std::string &hex) {
  if (hex.size() == 7 && hex[0] == '#') {
    try {
      const auto r = std::stoi(hex.substr(1, 2), nullptr, 16);
      const auto g = std::stoi(hex.substr(3, 2), nullptr, 16);
      const auto b = std::stoi(hex.substr(5, 2), nullptr, 16);
      return color(r / 255.0F, g / 255.0F, b / 255.0F, 1.0F);
    } catch (const std::exception &) {
    }
  }
  return color(1.0F, 0.82F, 0.05F, 1.0F);
}

geometry_msgs::msg::Point point(const double x, const double y, const double z) {
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

visualization_msgs::msg::MenuEntry editMenuEntry() {
  visualization_msgs::msg::MenuEntry entry;
  entry.id = kEditMenuEntryId;
  entry.title = "Edit";
  entry.command_type = visualization_msgs::msg::MenuEntry::FEEDBACK;
  return entry;
}

visualization_msgs::msg::InteractiveMarkerControl buttonMenuControl() {
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = "select";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  return control;
}

visualization_msgs::msg::Marker axisArrow(const geometry_msgs::msg::Point &end,
                                          const std_msgs::msg::ColorRGBA &arrow_color,
                                          const bool selected,
                                          const double marker_scale) {
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = (selected ? 0.014 : 0.01) * marker_scale;
  marker.scale.y = (selected ? 0.038 : 0.03) * marker_scale;
  marker.scale.z = (selected ? 0.045 : 0.035) * marker_scale;
  marker.points = {point(0.0, 0.0, 0.0), end};
  marker.color = arrow_color;
  return marker;
}

std::vector<visualization_msgs::msg::Marker>
axisArrows(const bool selected, const std::string &highlight,
           const double alpha_multiplier, const double marker_scale,
           const double axis_length) {
  const auto selected_color =
      withAlpha(highlightColor(highlight), alpha_multiplier);
  const auto x_color =
      selected ? selected_color
               : withAlpha(color(0.95F, 0.08F, 0.08F), alpha_multiplier);
  const auto y_color =
      selected ? selected_color
               : withAlpha(color(0.12F, 0.8F, 0.12F), alpha_multiplier);
  const auto z_color =
      selected ? selected_color
               : withAlpha(color(0.15F, 0.38F, 1.0F), alpha_multiplier);
  return {axisArrow(point(axis_length, 0.0, 0.0), x_color, selected, marker_scale),
          axisArrow(point(0.0, axis_length, 0.0), y_color, selected, marker_scale),
          axisArrow(point(0.0, 0.0, axis_length), z_color, selected, marker_scale)};
}

visualization_msgs::msg::Marker jointArrow(
    const UrdfJointSummary &joint,
    const std::map<std::string, geometry_msgs::msg::TransformStamped>
        &joint_transforms,
    const bool selected, const std::string &highlight,
    const std::string &joint_color,
    const double alpha_multiplier, const double marker_scale,
    const double joint_thickness) {
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  const auto thickness = marker_scale * joint_thickness;
  marker.scale.x = (selected ? 0.012 : 0.008) * thickness;
  marker.scale.y = (selected ? 0.035 : 0.024) * thickness;
  marker.scale.z = (selected ? 0.04 : 0.03) * marker_scale;
  marker.color = withAlpha(
      selected ? highlightColor(highlight) : highlightColor(joint_color),
      alpha_multiplier);

  geometry_msgs::msg::Point child = point(0.12 * marker_scale, 0.0, 0.0);
  const auto transform = joint_transforms.find(joint.name);
  if (transform != joint_transforms.end()) {
    child.x = transform->second.transform.translation.x;
    child.y = transform->second.transform.translation.y;
    child.z = transform->second.transform.translation.z;
    const auto length =
        std::sqrt(child.x * child.x + child.y * child.y + child.z * child.z);
    if (length < 0.001) {
      child = point(0.12 * marker_scale, 0.0, 0.0);
    }
  }
  marker.points = {point(0.0, 0.0, 0.0), child};
  return marker;
}

std::string linkMarkerName(const std::string &link_name) {
  return "link:" + link_name;
}

std::string jointMarkerName(const std::string &joint_name) {
  return "joint:" + joint_name;
}

std::string markerPayloadName(const std::string &marker_name,
                              const std::string &prefix) {
  if (marker_name.rfind(prefix, 0) != 0) {
    return {};
  }
  return marker_name.substr(prefix.size());
}

} // namespace

UrdfTfTreeMarkerServer &UrdfTfTreeMarkerServer::instance() {
  static UrdfTfTreeMarkerServer server;
  return server;
}

void UrdfTfTreeMarkerServer::start(const rclcpp::Node::SharedPtr &node) {
  if (!node) {
    stop();
    return;
  }
  if (node_ == node && server_) {
    return;
  }
  stop();
  node_ = node;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      kMarkerNamespace, node_);
  state_callback_id_ = UrdfEditorState::instance().addChangeCallback(
      [this]() { rebuildFromState(); });
  refresh_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(250),
      [this]() {
        const auto snapshot = UrdfEditorState::instance().snapshot();
        if (snapshot.revision != seen_revision_ ||
            snapshot.selected_link != seen_selected_link_ ||
            snapshot.selected_joint != seen_selected_joint_ ||
            snapshot.highlight_color != seen_highlight_color_ ||
            snapshot.tf_joint_color != seen_tf_joint_color_) {
          rebuildFromState();
        } else {
          refreshJointMarkers();
        }
      });
  rebuildFromState();
}

void UrdfTfTreeMarkerServer::stop() {
  if (state_callback_id_ != 0) {
    UrdfEditorState::instance().removeChangeCallback(state_callback_id_);
    state_callback_id_ = 0;
  }
  if (refresh_timer_) {
    refresh_timer_->cancel();
  }
  refresh_timer_.reset();
  server_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  node_.reset();
  seen_revision_ = 0;
  seen_selected_link_.clear();
  seen_selected_joint_.clear();
  seen_highlight_color_.clear();
  seen_tf_joint_color_.clear();
}

std::vector<visualization_msgs::msg::InteractiveMarker>
UrdfTfTreeMarkerServer::buildMarkers(
    const UrdfEditorSnapshot &snapshot,
    const std::map<std::string, geometry_msgs::msg::TransformStamped>
        &joint_transforms) {
  std::vector<visualization_msgs::msg::InteractiveMarker> markers;
  markers.reserve(snapshot.model.links.size() + snapshot.model.joints.size());

  for (const auto &link : snapshot.model.links) {
    if (snapshot.hidden_geometry_links.count(link) > 0) {
      continue;
    }
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = link;
    marker.name = linkMarkerName(link);
    marker.description = link;
    marker.scale = 0.25F * static_cast<float>(snapshot.tf_marker_scale);
    marker.pose = identityPose();
    marker.menu_entries.push_back(editMenuEntry());
    auto control = buttonMenuControl();
    control.markers =
        axisArrows(snapshot.selected_link == link, snapshot.highlight_color,
                   snapshot.tf_alpha_multiplier, snapshot.tf_marker_scale,
                   snapshot.tf_axis_length);
    marker.controls.push_back(control);
    markers.push_back(marker);
  }

  for (const auto &joint : snapshot.model.joints) {
    if (joint.name.empty() || joint.parent_link.empty() ||
        snapshot.hidden_geometry_links.count(joint.child_link) > 0) {
      continue;
    }
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = joint.parent_link;
    marker.name = jointMarkerName(joint.name);
    marker.description = joint.name;
    marker.scale = 0.25F * static_cast<float>(snapshot.tf_marker_scale);
    marker.pose = identityPose();
    marker.menu_entries.push_back(editMenuEntry());
    auto control = buttonMenuControl();
    control.markers.push_back(jointArrow(
        joint, joint_transforms, snapshot.selected_joint == joint.name,
        snapshot.highlight_color, snapshot.tf_joint_color,
        snapshot.tf_alpha_multiplier,
        snapshot.tf_marker_scale, snapshot.tf_joint_thickness));
    marker.controls.push_back(control);
    markers.push_back(marker);
  }

  return markers;
}

void UrdfTfTreeMarkerServer::handleFeedbackForTesting(
    const visualization_msgs::msg::InteractiveMarkerFeedback &feedback) {
  handleFeedback(std::make_shared<visualization_msgs::msg::InteractiveMarkerFeedback>(
      feedback));
}

void UrdfTfTreeMarkerServer::rebuildFromState() {
  if (!server_) {
    return;
  }
  const auto snapshot = UrdfEditorState::instance().snapshot();
  const auto transforms = lookupJointTransforms(snapshot);
  server_->clear();
  for (const auto &marker : buildMarkers(snapshot, transforms)) {
    server_->insert(marker,
                    [this](const auto feedback) { handleFeedback(feedback); });
  }
  server_->applyChanges();
  seen_revision_ = snapshot.revision;
  seen_selected_link_ = snapshot.selected_link;
  seen_selected_joint_ = snapshot.selected_joint;
  seen_highlight_color_ = snapshot.highlight_color;
  seen_tf_joint_color_ = snapshot.tf_joint_color;
}

void UrdfTfTreeMarkerServer::refreshJointMarkers() {
  if (!server_) {
    return;
  }
  const auto snapshot = UrdfEditorState::instance().snapshot();
  const auto transforms = lookupJointTransforms(snapshot);
  bool changed = false;
  for (const auto &joint : snapshot.model.joints) {
    if (snapshot.hidden_geometry_links.count(joint.child_link) > 0) {
      continue;
    }
    visualization_msgs::msg::InteractiveMarker marker;
    if (!server_->get(jointMarkerName(joint.name), marker)) {
      continue;
    }
    if (marker.controls.empty()) {
      continue;
    }
    marker.controls.front().markers.clear();
    marker.controls.front().markers.push_back(
        jointArrow(joint, transforms, snapshot.selected_joint == joint.name,
                   snapshot.highlight_color, snapshot.tf_joint_color,
                   snapshot.tf_alpha_multiplier,
                   snapshot.tf_marker_scale, snapshot.tf_joint_thickness));
    server_->insert(marker,
                    [this](const auto feedback) { handleFeedback(feedback); });
    changed = true;
  }
  if (changed) {
    server_->applyChanges();
  }
}

std::map<std::string, geometry_msgs::msg::TransformStamped>
UrdfTfTreeMarkerServer::lookupJointTransforms(
    const UrdfEditorSnapshot &snapshot) const {
  std::map<std::string, geometry_msgs::msg::TransformStamped> transforms;
  if (!tf_buffer_) {
    return transforms;
  }
  for (const auto &joint : snapshot.model.joints) {
    if (joint.name.empty() || joint.parent_link.empty() || joint.child_link.empty()) {
      continue;
    }
    try {
      transforms[joint.name] = tf_buffer_->lookupTransform(
          joint.parent_link, joint.child_link, tf2::TimePointZero,
          tf2::durationFromSec(0.01));
    } catch (const tf2::TransformException &) {
    }
  }
  return transforms;
}

void UrdfTfTreeMarkerServer::handleFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
  if (!feedback) {
    return;
  }

  if (const auto link = markerPayloadName(feedback->marker_name, "link:");
      !link.empty()) {
    auto &state = UrdfEditorState::instance();
    if (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK) {
      state.setSelectedLink(link);
      return;
    }
    if (feedback->event_type ==
            visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT &&
        feedback->menu_entry_id == kEditMenuEntryId) {
      state.setSelectedLink(link);
      state.openSourceElementEditor("link", link, "Edit Link XML: " + link);
    }
    return;
  }

  if (const auto joint = markerPayloadName(feedback->marker_name, "joint:");
      !joint.empty()) {
    auto &state = UrdfEditorState::instance();
    if (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK) {
      state.setSelectedJoint(joint);
      return;
    }
    if (feedback->event_type ==
            visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT &&
        feedback->menu_entry_id == kEditMenuEntryId) {
      state.setSelectedJoint(joint);
      state.openSourceElementEditor("joint", joint, "Edit Joint XML: " + joint);
    }
  }
}

} // namespace rviz2_urdf_editor
