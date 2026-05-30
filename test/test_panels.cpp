#include <cstdlib>

#include <QApplication>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QWidget>
#include <gtest/gtest.h>

#include "rviz2_urdf_editor/widgets.hpp"

TEST(UrdfEditorPanel, ConstructsLoadAndXmlEditorWidgets) {
  rviz2_urdf_editor::UrdfEditorPanel panel;

  ASSERT_NE(panel.findChild<QWidget *>("urdf_file_widget"), nullptr);
  ASSERT_NE(panel.findChild<QWidget *>("urdf_xml_editor_widget"), nullptr);
  EXPECT_NE(panel.findChild<QLineEdit *>(), nullptr);
  EXPECT_NE(panel.findChild<QPlainTextEdit *>("xml_editor"), nullptr);
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

int main(int argc, char **argv) {
  setenv("QT_QPA_PLATFORM", "offscreen", 0);
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
