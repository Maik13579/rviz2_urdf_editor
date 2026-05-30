// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "rviz2_urdf_editor/urdf_editor_state.hpp"
#include "rviz2_urdf_editor/urdf_tf_tree_marker_server.hpp"

namespace {

std::filesystem::path writeTempFile(const std::string &name,
                                    const std::string &text) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream output(path);
  output << text;
  return path;
}

const char kTwoJointUrdf[] = R"(
<robot name="marker_bot">
  <link name="base_link"/>
  <link name="arm_link"/>
  <link name="tool_link"/>
  <joint name="arm_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_link"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="arm_link"/>
    <child link="tool_link"/>
  </joint>
</robot>
)";

const char kOneJointUrdf[] = R"(
<robot name="marker_bot">
  <link name="base_link"/>
  <link name="arm_link"/>
  <joint name="arm_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_link"/>
  </joint>
</robot>
)";

std::vector<visualization_msgs::msg::InteractiveMarker> currentMarkers() {
  return rviz2_urdf_editor::UrdfTfTreeMarkerServer::buildMarkers(
      rviz2_urdf_editor::UrdfEditorState::instance().snapshot());
}

const visualization_msgs::msg::InteractiveMarker *findMarker(
    const std::vector<visualization_msgs::msg::InteractiveMarker> &markers,
    const std::string &name) {
  const auto marker =
      std::find_if(markers.begin(), markers.end(), [&](const auto &candidate) {
        return candidate.name == name;
      });
  return marker == markers.end() ? nullptr : &(*marker);
}

std_msgs::msg::ColorRGBA firstMarkerColor(
    const visualization_msgs::msg::InteractiveMarker &marker) {
  EXPECT_FALSE(marker.controls.empty());
  EXPECT_FALSE(marker.controls.front().markers.empty());
  const auto &visual = marker.controls.front().markers.front();
  if (!visual.colors.empty()) {
    return visual.colors.front();
  }
  return visual.color;
}

} // namespace

TEST(UrdfTfTreeMarkerServer, BuildsFrameAndJointMarkersFromCurrentModel) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_two_joint.urdf", kTwoJointUrdf).string(), {}))
      << state.snapshot().last_error;

  const auto markers = currentMarkers();
  EXPECT_NE(findMarker(markers, "link:base_link"), nullptr);
  EXPECT_NE(findMarker(markers, "link:arm_link"), nullptr);
  EXPECT_NE(findMarker(markers, "link:tool_link"), nullptr);
  EXPECT_NE(findMarker(markers, "joint:arm_joint"), nullptr);
  EXPECT_NE(findMarker(markers, "joint:tool_joint"), nullptr);
  EXPECT_EQ(markers.size(), 5u);
  const auto *base = findMarker(markers, "link:base_link");
  ASSERT_NE(base, nullptr);
  ASSERT_EQ(base->controls.size(), 1u);
  EXPECT_EQ(base->controls.front().interaction_mode,
            visualization_msgs::msg::InteractiveMarkerControl::BUTTON);
  ASSERT_GE(base->controls.front().markers.size(), 3u);
  EXPECT_EQ(base->controls.front().markers.front().type,
            visualization_msgs::msg::Marker::ARROW);
}

TEST(UrdfTfTreeMarkerServer, UsesLinkFramesAndJointParentFrames) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_frames.urdf", kTwoJointUrdf).string(), {}))
      << state.snapshot().last_error;

  const auto markers = currentMarkers();
  const auto *base = findMarker(markers, "link:base_link");
  const auto *arm_joint = findMarker(markers, "joint:arm_joint");
  const auto *tool_joint = findMarker(markers, "joint:tool_joint");
  ASSERT_NE(base, nullptr);
  ASSERT_NE(arm_joint, nullptr);
  ASSERT_NE(tool_joint, nullptr);
  EXPECT_EQ(base->header.frame_id, "base_link");
  EXPECT_EQ(arm_joint->header.frame_id, "base_link");
  EXPECT_EQ(tool_joint->header.frame_id, "arm_link");
}

TEST(UrdfTfTreeMarkerServer, RebuildDropsRemovedLinksAndJoints) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_stale_first.urdf", kTwoJointUrdf).string(), {}))
      << state.snapshot().last_error;
  ASSERT_NE(findMarker(currentMarkers(), "link:tool_link"), nullptr);
  ASSERT_NE(findMarker(currentMarkers(), "joint:tool_joint"), nullptr);

  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_stale_second.urdf", kOneJointUrdf).string(), {}))
      << state.snapshot().last_error;
  const auto markers = currentMarkers();
  EXPECT_EQ(findMarker(markers, "link:tool_link"), nullptr);
  EXPECT_EQ(findMarker(markers, "joint:tool_joint"), nullptr);
  EXPECT_NE(findMarker(markers, "link:arm_link"), nullptr);
  EXPECT_NE(findMarker(markers, "joint:arm_joint"), nullptr);
}

TEST(UrdfTfTreeMarkerServer, HighlightsSelectedLinkAndJoint) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_highlight.urdf", kOneJointUrdf).string(), {}))
      << state.snapshot().last_error;
  state.setHighlightColor("#123456");
  state.setSelectedLink("base_link");

  auto markers = currentMarkers();
  auto *base = findMarker(markers, "link:base_link");
  ASSERT_NE(base, nullptr);
  auto color = firstMarkerColor(*base);
  EXPECT_NEAR(color.r, 0x12 / 255.0, 0.001);
  EXPECT_NEAR(color.g, 0x34 / 255.0, 0.001);
  EXPECT_NEAR(color.b, 0x56 / 255.0, 0.001);

  state.setSelectedJoint("arm_joint");
  markers = currentMarkers();
  auto *joint = findMarker(markers, "joint:arm_joint");
  ASSERT_NE(joint, nullptr);
  color = firstMarkerColor(*joint);
  EXPECT_NEAR(color.r, 0x12 / 255.0, 0.001);
  EXPECT_NEAR(color.g, 0x34 / 255.0, 0.001);
  EXPECT_NEAR(color.b, 0x56 / 255.0, 0.001);
  EXPECT_TRUE(state.snapshot().selected_link.empty());
}

TEST(UrdfTfTreeMarkerServer, UsesConfiguredJointColor) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_joint_color.urdf", kOneJointUrdf).string(), {}))
      << state.snapshot().last_error;
  state.setSelectedLink("base_link");
  state.setTfJointColor("#abcdef");

  const auto markers = currentMarkers();
  const auto *joint = findMarker(markers, "joint:arm_joint");
  ASSERT_NE(joint, nullptr);
  const auto color = firstMarkerColor(*joint);
  EXPECT_NEAR(color.r, 0xab / 255.0, 0.001);
  EXPECT_NEAR(color.g, 0xcd / 255.0, 0.001);
  EXPECT_NEAR(color.b, 0xef / 255.0, 0.001);
}

TEST(UrdfTfTreeMarkerServer, AppliesAlphaScaleAndHiddenLinkSettings) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_visual_settings.urdf", kTwoJointUrdf).string(), {}))
      << state.snapshot().last_error;
  state.setTfAlphaMultiplier(0.4);
  state.setTfMarkerSettings(2.0, 0.3, 0.5);
  state.setLinkGeometryVisible("tool_link", false);

  const auto markers = currentMarkers();
  EXPECT_EQ(findMarker(markers, "link:tool_link"), nullptr);
  EXPECT_EQ(findMarker(markers, "joint:tool_joint"), nullptr);
  const auto *base = findMarker(markers, "link:base_link");
  const auto *joint = findMarker(markers, "joint:arm_joint");
  ASSERT_NE(base, nullptr);
  ASSERT_NE(joint, nullptr);
  ASSERT_FALSE(base->controls.front().markers.empty());
  EXPECT_NEAR(base->controls.front().markers.front().color.a, 0.4, 0.001);
  EXPECT_NEAR(base->controls.front().markers.front().points.back().x, 0.3,
              0.001);
  ASSERT_FALSE(joint->controls.front().markers.empty());
  EXPECT_NEAR(joint->controls.front().markers.front().scale.x, 0.008, 0.001);
}

TEST(UrdfTfTreeMarkerServer, AlphaZeroPublishesNoInteractiveMarkers) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_alpha_zero.urdf", kTwoJointUrdf).string(), {}))
      << state.snapshot().last_error;
  state.setTfAlphaMultiplier(0.0);

  EXPECT_TRUE(currentMarkers().empty());
}

TEST(UrdfTfTreeMarkerServer, MenuCallbacksOpenExpectedEditors) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_callbacks.urdf", kOneJointUrdf).string(), {}))
      << state.snapshot().last_error;

  visualization_msgs::msg::InteractiveMarkerFeedback feedback;
  feedback.event_type =
      visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT;
  feedback.menu_entry_id = 1;
  feedback.marker_name = "link:base_link";
  rviz2_urdf_editor::UrdfTfTreeMarkerServer::instance().handleFeedbackForTesting(
      feedback);
  EXPECT_EQ(state.snapshot().selected_link, "base_link");
  EXPECT_TRUE(state.snapshot().selected_joint.empty());
  EXPECT_EQ(state.xmlEditorDocument().kind, "link");
  EXPECT_EQ(state.xmlEditorDocument().name, "base_link");

  feedback.marker_name = "joint:arm_joint";
  rviz2_urdf_editor::UrdfTfTreeMarkerServer::instance().handleFeedbackForTesting(
      feedback);
  EXPECT_EQ(state.snapshot().selected_joint, "arm_joint");
  EXPECT_TRUE(state.snapshot().selected_link.empty());
  EXPECT_EQ(state.xmlEditorDocument().kind, "joint");
  EXPECT_EQ(state.xmlEditorDocument().name, "arm_joint");
}

TEST(UrdfTfTreeMarkerServer, ButtonClicksSelectWithoutOpeningEditors) {
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(
      writeTempFile("marker_button_clicks.urdf", kOneJointUrdf).string(), {}))
      << state.snapshot().last_error;
  const auto initial_document = state.xmlEditorDocument();

  visualization_msgs::msg::InteractiveMarkerFeedback feedback;
  feedback.event_type =
      visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
  feedback.marker_name = "link:base_link";
  rviz2_urdf_editor::UrdfTfTreeMarkerServer::instance().handleFeedbackForTesting(
      feedback);
  EXPECT_EQ(state.snapshot().selected_link, "base_link");
  EXPECT_TRUE(state.snapshot().selected_joint.empty());
  EXPECT_EQ(state.xmlEditorDocument().kind, initial_document.kind);
  EXPECT_EQ(state.xmlEditorDocument().name, initial_document.name);

  feedback.marker_name = "joint:arm_joint";
  rviz2_urdf_editor::UrdfTfTreeMarkerServer::instance().handleFeedbackForTesting(
      feedback);
  EXPECT_EQ(state.snapshot().selected_joint, "arm_joint");
  EXPECT_TRUE(state.snapshot().selected_link.empty());
  EXPECT_EQ(state.xmlEditorDocument().kind, initial_document.kind);
  EXPECT_EQ(state.xmlEditorDocument().name, initial_document.name);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rviz2_urdf_editor::UrdfTfTreeMarkerServer::instance().stop();
  rviz2_urdf_editor::UrdfEditorState::instance().setNode(nullptr);
  rclcpp::shutdown();
  return result;
}
