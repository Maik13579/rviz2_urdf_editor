// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QApplication>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QWidget>
#include <gtest/gtest.h>
#include <rviz_common/config.hpp>

#include "rviz2_urdf_editor/widgets.hpp"

namespace {

std::string readFile(const std::filesystem::path &path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::string editorRvizText() {
  const auto share =
      ament_index_cpp::get_package_share_directory("rviz2_urdf_editor");
  return readFile(std::filesystem::path(share) / "layouts" / "editor.rviz");
}

} // namespace

TEST(UrdfEditorPanel, ConstructsLoadAndXmlEditorWidgets) {
  rviz2_urdf_editor::UrdfEditorPanel panel;

  ASSERT_NE(panel.findChild<QWidget *>("urdf_file_widget"), nullptr);
  ASSERT_NE(panel.findChild<QWidget *>("urdf_xml_editor_widget"), nullptr);
  EXPECT_NE(panel.findChild<QLineEdit *>(), nullptr);
  EXPECT_NE(panel.findChild<QPlainTextEdit *>("xml_editor"), nullptr);
  const auto buttons = panel.findChildren<QPushButton *>();
  const auto button_index = [&](const QString &text) {
    for (int index = 0; index < buttons.size(); ++index) {
      if (buttons[index]->text() == text) {
        return index;
      }
    }
    return -1;
  };
  const int browse = button_index("Browse");
  const int load = button_index("Load");
  const int settings = button_index("Settings");
  const int save = button_index("Save");
  ASSERT_GE(browse, 0);
  ASSERT_GE(load, 0);
  ASSERT_GE(settings, 0);
  ASSERT_GE(save, 0);
  EXPECT_LT(browse, load);
  EXPECT_LT(load, settings);
  EXPECT_LT(settings, save);
}

TEST(UrdfEditorPanel, SavesTfMarkerSettingsToRvizConfig) {
  rviz2_urdf_editor::UrdfEditorPanel panel;
  rviz_common::Config config;

  panel.save(config);
  const auto file_config = config.mapGetChild("File Widget");
  ASSERT_TRUE(file_config.isValid());
  float marker_scale = 0.0F;
  float axis_length = 0.0F;
  float joint_thickness = 0.0F;
  QString highlight_color;
  QString joint_color;
  ASSERT_TRUE(file_config.mapGetFloat("tf_marker_scale", &marker_scale));
  ASSERT_TRUE(file_config.mapGetFloat("tf_axis_length", &axis_length));
  ASSERT_TRUE(file_config.mapGetFloat("tf_joint_thickness", &joint_thickness));
  ASSERT_TRUE(file_config.mapGetString("highlight_color", &highlight_color));
  ASSERT_TRUE(file_config.mapGetString("tf_joint_color", &joint_color));
  EXPECT_FLOAT_EQ(marker_scale, 1.0F);
  EXPECT_FLOAT_EQ(axis_length, 0.18F);
  EXPECT_FLOAT_EQ(joint_thickness, 1.0F);
  EXPECT_EQ(highlight_color, "#ffd10d");
  EXPECT_EQ(joint_color, "#f2f2f2");
}

TEST(UrdfNavPanel, ConstructsExpectedTabs) {
  rviz2_urdf_editor::UrdfNavPanel panel;

  auto *tabs = panel.findChild<QTabWidget *>("urdf_nav_tabs");
  ASSERT_NE(tabs, nullptr);
  ASSERT_EQ(tabs->count(), 5);
  EXPECT_EQ(tabs->tabText(0), "Frames");
  EXPECT_EQ(tabs->tabText(1), "Files");
  EXPECT_EQ(tabs->tabText(2), "Joints");
  EXPECT_EQ(tabs->tabText(3), "Xacro");
  EXPECT_EQ(tabs->tabText(4), "Meshes");
}

TEST(EditorRvizLayout, UsesInteractiveMarkersInsteadOfTfDisplay) {
  const auto layout = editorRvizText();

  EXPECT_EQ(layout.find("Class: rviz_default_plugins/TF\n      Enabled: true"),
            std::string::npos);
  EXPECT_NE(layout.find("Class: rviz_default_plugins/InteractiveMarkers"),
            std::string::npos);
  EXPECT_NE(layout.find("Interactive Markers Namespace: /rviz2_urdf_editor/tf_tree"),
            std::string::npos);
  EXPECT_NE(layout.find("Class: rviz_default_plugins/TF\n  Value: true"),
            std::string::npos);
}

int main(int argc, char **argv) {
  setenv("QT_QPA_PLATFORM", "offscreen", 0);
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
