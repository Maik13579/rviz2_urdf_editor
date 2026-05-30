#include <algorithm>
#include <cstdlib>
#include <string>

#include <QApplication>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>

namespace {

void expectExported(pluginlib::ClassLoader<rviz_common::Panel> &loader,
                    const std::string &class_name) {
  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), class_name), classes.end())
      << class_name;
  const auto instance = loader.createSharedInstance(class_name);
  ASSERT_NE(instance, nullptr);
}

} // namespace

TEST(PluginExports, RvizPanelsLoad) {
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common",
                                                    "rviz_common::Panel");

  expectExported(loader, "rviz2_urdf_editor/UrdfEditorPanel");
  expectExported(loader, "rviz2_urdf_editor/UrdfNavPanel");
}

int main(int argc, char **argv) {
  setenv("QT_QPA_PLATFORM", "offscreen", 0);
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
