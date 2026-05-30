// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <QApplication>
#include <QLabel>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPushButton>
#include <QTextBlock>
#include <QTextLayout>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "rviz2_urdf_editor/robot_state_publisher_runtime.hpp"
#include "rviz2_urdf_editor/urdf_editor_state.hpp"
#include "rviz2_urdf_editor/widgets.hpp"

namespace {

std::filesystem::path writeTempFile(const std::string &name,
                                    const std::string &text) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream output(path);
  output << text;
  return path;
}

std::string readFile(const std::filesystem::path &path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

const char kSimpleUrdf[] = R"(
<robot name="test_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="meshes/base.stl"/>
      </geometry>
    </visual>
  </link>
  <link name="arm_link"/>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <limit lower="-1.5" upper="1.5" effort="1" velocity="1"/>
  </joint>
</robot>
)";

const char kSimpleXacro[] = R"xacro(
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="$(arg robot_name)">
  <xacro:arg name="robot_name" default="default_bot"/>
  <link name="base_link"/>
</robot>
)xacro";

const char kMacroXacro[] = R"xacro(
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="macro_bot">
  <xacro:arg name="leg_prefix" default="left"/>
  <xacro:property name="leg_radius" value="0.1"/>
  <xacro:macro name="leg" params="prefix parent">
    <link name="${prefix}_link"/>
    <joint name="${prefix}_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="${prefix}_link"/>
    </joint>
  </xacro:macro>
  <link name="base_link"/>
  <xacro:leg prefix="left" parent="base_link"/>
</robot>
)xacro";

const char kSimpleDae[] = R"(
<COLLADA>
  <library_effects>
    <effect id="material-effect">
      <profile_COMMON>
        <technique sid="common">
          <phong>
            <diffuse><texture texture="paint"/></diffuse>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
</COLLADA>
)";

const char kColorDae[] = R"(
<COLLADA>
  <library_effects>
    <effect id="paint-effect">
      <profile_COMMON>
        <technique sid="common">
          <phong>
            <ambient><color>0.1 0.1 0.1 1.0</color></ambient>
            <diffuse><color>0.2 0.3 0.4 0.8</color></diffuse>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
</COLLADA>
)";

const char kReferencedMaterialUrdf[] = R"(
<robot name="material_bot">
  <material name="IPA/LightGrey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
  <link name="scanner_link">
    <visual>
      <geometry>
        <mesh filename="package://scanner.dae"/>
      </geometry>
      <material name="IPA/LightGrey"/>
    </visual>
  </link>
</robot>
)";

} // namespace

TEST(UrdfEditorState, LoadsUrdfAndSummarizesModel) {
  const auto path = writeTempFile("dashboard_simple.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  const auto snapshot = state.snapshot();
  EXPECT_TRUE(snapshot.loaded);
  EXPECT_EQ(snapshot.model.robot_name, "test_bot");
  ASSERT_EQ(snapshot.model.links.size(), 2u);
  ASSERT_EQ(snapshot.model.joints.size(), 1u);
  EXPECT_EQ(snapshot.model.joints.front().name, "arm_joint");
  EXPECT_DOUBLE_EQ(snapshot.model.joints.front().lower, -1.5);
  ASSERT_EQ(snapshot.model.meshes.size(), 1u);
  EXPECT_EQ(snapshot.model.meshes.front().owner, "base_link");
  EXPECT_FALSE(snapshot.model.meshes.front().resolved_path.empty());
  ASSERT_EQ(snapshot.dependencies.size(), 1u);
  EXPECT_EQ(snapshot.dependencies.front().kind, "source");
  EXPECT_FALSE(snapshot.dependencies.front().sticky);
}

TEST(UrdfEditorState, ExpandsXacroWithArguments) {
  const auto path = writeTempFile("dashboard_simple.xacro", kSimpleXacro);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {{"robot_name", "xacro_bot"}}))
      << state.snapshot().last_error;
  EXPECT_NE(state.snapshot().expanded_urdf.find("xacro_bot"),
            std::string::npos);
  EXPECT_EQ(state.snapshot().model.robot_name, "xacro_bot");
}

TEST(UrdfEditorState, ResolvesXacroFindPackageSharePaths) {
  const auto resolved =
      rviz2_urdf_editor::UrdfEditorState::resolveMeshPath(
          "$(find rviz2_urdf_editor)/package.xml", "");

  ASSERT_FALSE(resolved.empty());
  EXPECT_TRUE(std::filesystem::exists(resolved));
  EXPECT_NE(readFile(resolved).find("<name>rviz2_urdf_editor</name>"),
            std::string::npos);
}

TEST(UrdfEditorState, IndexesMacrosAndMarksInvocationsFromFindIncludes) {
  const auto path = writeTempFile(
      "dashboard_find_include_top.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"find_bot\">"
      "<xacro:include filename=\"$(find rviz2_urdf_editor)/test/fixtures/find_include_macro.xacro\"/>"
      "<link name=\"base_link\"/>"
      "<xacro:fixture_sensor prefix=\"front\" parent=\"base_link\"/>"
      "</robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  const auto macro = state.sourceElement("macro", "fixture_sensor");
  EXPECT_TRUE(macro.editable);
  EXPECT_NE(macro.source_path.find("find_include_macro.xacro"), std::string::npos);

  state.openSourceElementEditor("link", "front_sensor_link",
                                "Edit Link XML: front_sensor_link");
  const auto document = state.xmlEditorDocument();
  EXPECT_TRUE(document.editable);
  EXPECT_TRUE(document.whole_file);
  EXPECT_TRUE(document.has_selection);
  EXPECT_EQ(document.path, path.string());
  ASSERT_LE(document.selection_begin, document.selection_end);
  ASSERT_LE(document.selection_end, document.xml.size());
  EXPECT_NE(document.xml.substr(document.selection_begin,
                                document.selection_end -
                                    document.selection_begin)
                .find("<xacro:fixture_sensor"),
            std::string::npos);
}

TEST(UrdfEditorState, SourceIndexDetectsLinksJointsMacrosPropertiesAndArgs) {
  const auto path = writeTempFile("dashboard_source_index.xacro", kMacroXacro);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  const auto elements = state.sourceElements();
  const auto has_element = [&](const std::string &kind, const std::string &name) {
    return std::any_of(elements.begin(), elements.end(), [&](const auto &element) {
      return element.kind == kind && element.name == name && element.editable;
    });
  };
  EXPECT_TRUE(has_element("link", "base_link"));
  EXPECT_TRUE(has_element("macro", "leg"));
  EXPECT_TRUE(has_element("property", "leg_radius"));
  EXPECT_TRUE(has_element("arg", "leg_prefix"));
  EXPECT_FALSE(has_element("link", "left_link"));
  EXPECT_FALSE(state.sourceElement("link", "left_link").editable);
}

TEST(UrdfEditorState, XacroWidgetListsAndOpensMacrosPropertiesAndArgs) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_xacro_widget_test");
  const auto path = writeTempFile("dashboard_xacro_widget.xacro", kMacroXacro);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXacroMacroWidget widget;
  widget.initialize(node, "xacro_widget");
  widget.configure(widget.getDefaultConfig());
  QApplication::processEvents();

  const auto trees = widget.widget()->findChildren<QTreeWidget *>();
  ASSERT_EQ(trees.size(), 3);
  for (auto *tree : trees) {
    EXPECT_EQ(tree->columnCount(), 1);
  }

  auto find_item = [&](const std::string &kind, const std::string &name) {
    for (auto *tree : trees) {
      for (int index = 0; index < tree->topLevelItemCount(); ++index) {
        auto *file_item = tree->topLevelItem(index);
        for (int child_index = 0; child_index < file_item->childCount();
             ++child_index) {
          auto *item = file_item->child(child_index);
          if (item->data(0, Qt::UserRole).toString().toStdString() == kind &&
              item->data(0, Qt::UserRole + 1).toString().toStdString() == name) {
            return std::pair<QTreeWidget *, QTreeWidgetItem *>{tree, item};
          }
        }
      }
    }
    return std::pair<QTreeWidget *, QTreeWidgetItem *>{nullptr, nullptr};
  };

  EXPECT_NE(find_item("macro", "leg").second, nullptr);
  const auto property = find_item("property", "leg_radius");
  ASSERT_NE(property.first, nullptr);
  ASSERT_NE(property.second, nullptr);
  EXPECT_NE(find_item("arg", "leg_prefix").second, nullptr);

  ASSERT_TRUE(QMetaObject::invokeMethod(
      property.first, "itemDoubleClicked", Qt::DirectConnection,
      Q_ARG(QTreeWidgetItem *, property.second), Q_ARG(int, 0)));

  const auto document = state.xmlEditorDocument();
  EXPECT_TRUE(document.editable);
  EXPECT_TRUE(document.whole_file);
  EXPECT_TRUE(document.has_selection);
  EXPECT_EQ(document.kind, "property");
  EXPECT_EQ(document.name, "leg_radius");
  EXPECT_EQ(document.path, path.string());
  EXPECT_NE(document.xml.find("xacro:property"), std::string::npos);
  ASSERT_LE(document.selection_begin, document.selection_end);
  ASSERT_LE(document.selection_end, document.xml.size());
  EXPECT_NE(document.xml.substr(document.selection_begin,
                                document.selection_end -
                                    document.selection_begin)
                .find("xacro:property"),
            std::string::npos);
}

TEST(UrdfEditorState, GeneratedLinkEditorOpensGeneratingXacroSource) {
  const auto path = writeTempFile("dashboard_generated_link_editor.xacro",
                                  kMacroXacro);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  state.openSourceElementEditor("link", "left_link", "Edit Link XML: left_link");

  const auto document = state.xmlEditorDocument();
  EXPECT_TRUE(document.editable);
  EXPECT_TRUE(document.whole_file);
  EXPECT_TRUE(document.has_selection);
  EXPECT_EQ(document.path, path.string());
  EXPECT_EQ(document.title, path.filename().string());
  EXPECT_NE(document.xml.find("<xacro:leg prefix=\"left\""), std::string::npos);
  ASSERT_LE(document.selection_begin, document.selection_end);
  ASSERT_LE(document.selection_end, document.xml.size());
  EXPECT_NE(document.xml.substr(document.selection_begin,
                                document.selection_end -
                                    document.selection_begin)
                .find("<xacro:leg"),
            std::string::npos);
  EXPECT_NE(document.status.find("likely xacro invocation"), std::string::npos);
}

TEST(UrdfEditorState, GeneratedLinkEditorMarksTopLevelInvocationForIncludedMacro) {
  const auto include_path = writeTempFile(
      "dashboard_generated_velodyne_macro.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "<xacro:macro name=\"vlp16_lidar\" params=\"name parent *origin\">"
      "<joint name=\"${name}_frame_joint\" type=\"fixed\">"
      "<parent link=\"${name}_link\"/><child link=\"${name}\"/>"
      "</joint><link name=\"${name}\"/>"
      "</xacro:macro></robot>");
  const auto path = writeTempFile(
      "dashboard_generated_velodyne_top.xacro",
      std::string("<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" "
                  "name=\"bot\"><xacro:include filename=\"") +
          include_path.string() +
          "\"/><link name=\"velodyne_mount\"/>"
          "<xacro:vlp16_lidar name=\"velodyne\" parent=\"velodyne_mount\">"
          "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
          "</xacro:vlp16_lidar></robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  state.openSourceElementEditor("link", "velodyne",
                                "Edit Link XML: velodyne");
  auto document = state.xmlEditorDocument();
  EXPECT_TRUE(document.editable);
  EXPECT_TRUE(document.whole_file);
  EXPECT_TRUE(document.has_selection);
  EXPECT_EQ(document.path, path.string());
  EXPECT_NE(document.xml.substr(document.selection_begin,
                                document.selection_end -
                                    document.selection_begin)
                .find("<xacro:vlp16_lidar"),
            std::string::npos);

  state.openSourceElementEditor("joint", "velodyne_frame_joint",
                                "Edit Incoming Joint XML: velodyne_frame_joint");
  document = state.xmlEditorDocument();
  EXPECT_TRUE(document.editable);
  EXPECT_TRUE(document.whole_file);
  EXPECT_TRUE(document.has_selection);
  EXPECT_EQ(document.path, path.string());
  EXPECT_NE(document.xml.substr(document.selection_begin,
                                document.selection_end -
                                    document.selection_begin)
                .find("<xacro:vlp16_lidar"),
            std::string::npos);
}

TEST(UrdfEditorState, SourceIndexDetectsLiteralLinksInsideIncludedMacros) {
  const auto include_path = writeTempFile(
      "dashboard_body_macro_link.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "<xacro:macro name=\"body\">"
      "<link name=\"base_footprint\"><visual/></link>"
      "</xacro:macro></robot>");
  const auto path = writeTempFile(
      "dashboard_top_macro_link.xacro",
      std::string(
          "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"bot\">")
      + "<xacro:include filename=\"" + include_path.string() +
      "\"/><xacro:body/></robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  const auto element = state.sourceElement("link", "base_footprint");
  EXPECT_TRUE(element.editable);
  EXPECT_EQ(element.source_path, include_path.string());

  state.openSourceElementEditor("link", "base_footprint",
                                "Edit Link XML: base_footprint");
  const auto document = state.xmlEditorDocument();
  EXPECT_TRUE(document.editable);
  EXPECT_TRUE(document.whole_file);
  EXPECT_TRUE(document.has_selection);
  EXPECT_EQ(document.path, include_path.string());
  EXPECT_EQ(document.title, include_path.filename().string());
  EXPECT_NE(document.xml.find("base_footprint"), std::string::npos);
  ASSERT_LE(document.selection_begin, document.selection_end);
  ASSERT_LE(document.selection_end, document.xml.size());
  EXPECT_NE(document.xml.substr(document.selection_begin,
                                document.selection_end -
                                    document.selection_begin)
                .find("base_footprint"),
            std::string::npos);
}

TEST(UrdfEditorState, EditingLinkSnippetRebuildsAndMarksDirty) {
  const auto path = writeTempFile("dashboard_edit_link.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  const auto original_file_text = readFile(path);
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  ASSERT_TRUE(state.applySourceElementXml(
      "link", "arm_link",
      "<link name=\"arm_link\"><visual><origin xyz=\"1 2 3\"/></visual></link>"))
      << state.snapshot().last_error;
  const auto snapshot = state.snapshot();
  EXPECT_TRUE(snapshot.dirty);
  ASSERT_EQ(snapshot.dependencies.size(), 1u);
  EXPECT_TRUE(snapshot.dependencies.front().sticky);
  EXPECT_EQ(state.xmlEditorDocument().title, path.filename().string() + "*");
  EXPECT_NE(snapshot.source_text.find("xyz=\"1 2 3\""), std::string::npos);
  EXPECT_NE(snapshot.expanded_urdf.find("xyz=\"1 2 3\""), std::string::npos);
  EXPECT_EQ(readFile(path), original_file_text);
}

TEST(UrdfEditorState, EditingJointSnippetUpdatesParsedModel) {
  const auto path = writeTempFile("dashboard_edit_joint.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  ASSERT_TRUE(state.applySourceElementXml(
      "joint", "arm_joint",
      "<joint name=\"arm_joint\" type=\"fixed\"><parent link=\"arm_link\"/>"
      "<child link=\"base_link\"/></joint>"))
      << state.snapshot().last_error;
  const auto joints = state.snapshot().model.joints;
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints.front().type, "fixed");
  EXPECT_EQ(joints.front().parent_link, "arm_link");
  EXPECT_EQ(joints.front().child_link, "base_link");
}

TEST(UrdfEditorState, RejectsInvalidAndWrongSnippetXml) {
  const auto path = writeTempFile("dashboard_reject_edit.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  const auto before = state.snapshot().source_text;
  EXPECT_FALSE(state.applySourceElementXml("link", "arm_link",
                                           "<link name=\"arm_link\">"));
  EXPECT_EQ(state.snapshot().source_text, before);
  EXPECT_FALSE(state.applySourceElementXml("link", "arm_link",
                                           "<joint name=\"arm_link\"/>"));
  EXPECT_EQ(state.snapshot().source_text, before);
  EXPECT_FALSE(state.applySourceElementXml("link", "arm_link",
                                           "<link name=\"other_link\"/>"));
  EXPECT_EQ(state.snapshot().source_text, before);
}

TEST(UrdfEditorState, EditingMacroRebuildsExpandedUrdf) {
  const auto path = writeTempFile("dashboard_edit_macro.xacro", kMacroXacro);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  ASSERT_NE(state.snapshot().expanded_urdf.find("left_link"), std::string::npos);
  ASSERT_TRUE(state.applySourceElementXml(
      "macro", "leg",
      "<xacro:macro name=\"leg\" params=\"prefix parent\">"
      "<link name=\"${prefix}_foot\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_foot\"/>"
      "</joint></xacro:macro>"))
      << state.snapshot().last_error;
  const auto snapshot = state.snapshot();
  EXPECT_TRUE(snapshot.dirty);
  EXPECT_NE(snapshot.expanded_urdf.find("left_foot"), std::string::npos);
  EXPECT_EQ(snapshot.expanded_urdf.find("left_link"), std::string::npos);
}

TEST(UrdfEditorState, IndexesAndEditsMacrosFromIncludedXacroFiles) {
  const auto include_path = writeTempFile(
      "dashboard_included_macros.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "<xacro:macro name=\"wheel\" params=\"prefix parent\">"
      "<link name=\"${prefix}_wheel\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_wheel\"/>"
      "</joint></xacro:macro></robot>");
  const auto path = writeTempFile(
      "dashboard_include_top.xacro",
      std::string(
          "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"include_bot\">")
      + "<xacro:include filename=\"" +
          include_path.string() +
          "\"/><link name=\"base_link\"/>"
          "<xacro:wheel prefix=\"left\" parent=\"base_link\"/></robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  EXPECT_EQ(state.xmlEditorDocument().path, path.string());
  EXPECT_TRUE(state.xmlEditorDocument().whole_file);
  const auto element = state.sourceElement("macro", "wheel");
  EXPECT_TRUE(element.editable);
  EXPECT_EQ(element.source_path, include_path.string());
  ASSERT_NE(state.snapshot().expanded_urdf.find("left_wheel"), std::string::npos);

  state.openSourceElementEditor("macro", "wheel", "Edit Macro XML: wheel");
  const auto document = state.xmlEditorDocument();
  EXPECT_TRUE(document.editable);
  EXPECT_TRUE(document.whole_file);
  ASSERT_TRUE(document.has_selection);
  EXPECT_EQ(document.path, include_path.string());
  auto edited_xml = document.xml;
  edited_xml.replace(
      document.selection_begin, document.selection_end - document.selection_begin,
      "<xacro:macro name=\"wheel\" params=\"prefix parent\">"
      "<link name=\"${prefix}_caster\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_caster\"/>"
      "</joint></xacro:macro>");
  ASSERT_TRUE(state.applyXmlEditorDocument(edited_xml))
      << state.snapshot().last_error;
  EXPECT_EQ(readFile(include_path).find("${prefix}_caster"), std::string::npos);
  EXPECT_NE(state.snapshot().expanded_urdf.find("left_caster"), std::string::npos);
  EXPECT_TRUE(state.snapshot().dirty);
  const auto dirty_snapshot = state.snapshot();
  const auto sticky_dependency =
      std::find_if(dirty_snapshot.dependencies.begin(),
                   dirty_snapshot.dependencies.end(), [&](const auto &dependency) {
                     return dependency.path == include_path.string();
                   });
  ASSERT_NE(sticky_dependency, dirty_snapshot.dependencies.end());
  EXPECT_TRUE(sticky_dependency->sticky);
  state.openDependencyFileEditor(include_path.string());
  EXPECT_EQ(state.xmlEditorDocument().title, include_path.filename().string() + "*");

  ASSERT_TRUE(state.save()) << state.snapshot().last_error;
  EXPECT_NE(readFile(include_path).find("${prefix}_caster"), std::string::npos);
  const auto saved_snapshot = state.snapshot();
  const auto saved_dependency =
      std::find_if(saved_snapshot.dependencies.begin(),
                   saved_snapshot.dependencies.end(), [&](const auto &dependency) {
                     return dependency.path == include_path.string();
                   });
  ASSERT_NE(saved_dependency, saved_snapshot.dependencies.end());
  EXPECT_FALSE(saved_dependency->sticky);
  state.openDependencyFileEditor(include_path.string());
  EXPECT_EQ(state.xmlEditorDocument().title, include_path.filename().string());
}

TEST(UrdfEditorState, SavePersistsCurrentWholeFileEditorDraft) {
  const auto path = writeTempFile("dashboard_save_draft.xacro", kSimpleXacro);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {{"robot_name", "draft_bot"}}))
      << state.snapshot().last_error;
  const auto edited = std::string(kSimpleXacro) + "\n<!-- calibration note -->\n";
  state.setXmlEditorDraft(edited);

  auto snapshot = state.snapshot();
  const auto source =
      std::find_if(snapshot.dependencies.begin(), snapshot.dependencies.end(),
                   [&](const auto &dependency) {
                     return dependency.path == path.string();
                   });
  ASSERT_NE(source, snapshot.dependencies.end());
  EXPECT_TRUE(source->sticky);
  ASSERT_TRUE(state.save()) << state.snapshot().last_error;
  EXPECT_NE(readFile(path).find("calibration note"), std::string::npos);
  snapshot = state.snapshot();
  const auto saved_source =
      std::find_if(snapshot.dependencies.begin(), snapshot.dependencies.end(),
                   [&](const auto &dependency) {
                     return dependency.path == path.string();
                   });
  ASSERT_NE(saved_source, snapshot.dependencies.end());
  EXPECT_FALSE(saved_source->sticky);
}

TEST(UrdfEditorState, MacroEditorDraftMarksIncludedXacroSticky) {
  const auto include_path = writeTempFile(
      "dashboard_macro_draft_sticky_include.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "<xacro:macro name=\"wheel\" params=\"prefix parent\">"
      "<link name=\"${prefix}_wheel\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_wheel\"/>"
      "</joint></xacro:macro></robot>");
  const auto path = writeTempFile(
      "dashboard_macro_draft_sticky_top.xacro",
      std::string("<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" "
                  "name=\"include_bot\"><xacro:include filename=\"") +
          include_path.string() +
          "\"/><link name=\"base_link\"/>"
          "<xacro:wheel prefix=\"left\" parent=\"base_link\"/></robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  state.openSourceElementEditor("macro", "wheel", "Edit Macro XML: wheel");
  const auto document = state.xmlEditorDocument();
  ASSERT_EQ(document.path, include_path.string());
  ASSERT_TRUE(document.has_selection);
  auto edited_xml = document.xml;
  edited_xml.replace(
      document.selection_begin, document.selection_end - document.selection_begin,
      "<xacro:macro name=\"wheel\" params=\"prefix parent\">"
      "<!-- draft comment -->"
      "<link name=\"${prefix}_wheel\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_wheel\"/>"
      "</joint></xacro:macro>");
  state.setXmlEditorDraft(edited_xml);

  const auto snapshot = state.snapshot();
  const auto dependency =
      std::find_if(snapshot.dependencies.begin(), snapshot.dependencies.end(),
                   [&](const auto &candidate) {
                     return candidate.path == include_path.string();
                   });
  ASSERT_NE(dependency, snapshot.dependencies.end());
  EXPECT_TRUE(dependency->sticky);
  EXPECT_EQ(state.xmlEditorDocument().title, include_path.filename().string() + "*");
}

TEST(UrdfEditorState, EditorDraftEquivalentTrailingNewlineIsNotSticky) {
  const auto path = writeTempFile("dashboard_equivalent_draft.xacro", kSimpleXacro);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {{"robot_name", "draft_bot"}}))
      << state.snapshot().last_error;
  state.setXmlEditorDraft(std::string(kSimpleXacro) + "\n");

  const auto snapshot = state.snapshot();
  const auto source =
      std::find_if(snapshot.dependencies.begin(), snapshot.dependencies.end(),
                   [&](const auto &dependency) {
                     return dependency.path == path.string();
                   });
  ASSERT_NE(source, snapshot.dependencies.end());
  EXPECT_FALSE(source->sticky);
  EXPECT_FALSE(snapshot.dirty);
}

TEST(UrdfEditorState, EquivalentDependencyDraftIsNotSticky) {
  const auto include_path = writeTempFile(
      "dashboard_equivalent_dependency_draft_include.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "<xacro:macro name=\"wheel\" params=\"prefix parent\">"
      "<link name=\"${prefix}_wheel\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_wheel\"/>"
      "</joint></xacro:macro></robot>");
  const auto path = writeTempFile(
      "dashboard_equivalent_dependency_draft_top.xacro",
      std::string("<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" "
                  "name=\"include_bot\"><xacro:include filename=\"") +
          include_path.string() +
          "\"/><link name=\"base_link\"/>"
          "<xacro:wheel prefix=\"left\" parent=\"base_link\"/></robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  state.openDependencyFileEditor(include_path.string());
  state.setXmlEditorDraft(readFile(include_path) + "\n");

  const auto snapshot = state.snapshot();
  const auto dependency =
      std::find_if(snapshot.dependencies.begin(), snapshot.dependencies.end(),
                   [&](const auto &candidate) {
                     return candidate.path == include_path.string();
                   });
  ASSERT_NE(dependency, snapshot.dependencies.end());
  EXPECT_FALSE(dependency->sticky);
  EXPECT_FALSE(snapshot.dirty);
  EXPECT_EQ(state.xmlEditorDocument().title, include_path.filename().string());
}

TEST(UrdfEditorState, EquivalentDependencyOverlayIsNotSticky) {
  const auto include_path = writeTempFile(
      "dashboard_equivalent_dependency_overlay_include.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "<xacro:macro name=\"wheel\" params=\"prefix parent\">"
      "<link name=\"${prefix}_wheel\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_wheel\"/>"
      "</joint></xacro:macro></robot>");
  const auto path = writeTempFile(
      "dashboard_equivalent_dependency_overlay_top.xacro",
      std::string("<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" "
                  "name=\"include_bot\"><xacro:include filename=\"") +
          include_path.string() +
          "\"/><link name=\"base_link\"/>"
          "<xacro:wheel prefix=\"left\" parent=\"base_link\"/></robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  ASSERT_TRUE(state.applyDependencyFileXml(include_path.string(),
                                           readFile(include_path) + "\n"))
      << state.snapshot().last_error;

  const auto snapshot = state.snapshot();
  const auto dependency =
      std::find_if(snapshot.dependencies.begin(), snapshot.dependencies.end(),
                   [&](const auto &candidate) {
                     return candidate.path == include_path.string();
                   });
  ASSERT_NE(dependency, snapshot.dependencies.end());
  EXPECT_FALSE(dependency->sticky);
  EXPECT_FALSE(snapshot.dirty);
  EXPECT_EQ(state.xmlEditorDocument().title, path.filename().string());
}

TEST(UrdfEditorState, EditorWidgetRefreshDoesNotMarkOpenedFileSticky) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_widget_refresh_test");
  const auto include_path = writeTempFile(
      "dashboard_widget_refresh_include.xacro",
      "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "<xacro:macro name=\"wheel\" params=\"prefix parent\">"
      "<link name=\"${prefix}_wheel\"/>"
      "<joint name=\"${prefix}_joint\" type=\"fixed\">"
      "<parent link=\"${parent}\"/><child link=\"${prefix}_wheel\"/>"
      "</joint></xacro:macro></robot>");
  const auto path = writeTempFile(
      "dashboard_widget_refresh_top.xacro",
      std::string("<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" "
                  "name=\"include_bot\"><xacro:include filename=\"") +
          include_path.string() +
          "\"/><link name=\"base_link\"/>"
          "<xacro:wheel prefix=\"left\" parent=\"base_link\"/></robot>");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_widget");
  editor_widget.configure(editor_widget.getDefaultConfig());
  QApplication::processEvents();

  state.openDependencyFileEditor(include_path.string());
  QApplication::processEvents();

  const auto snapshot = state.snapshot();
  const auto dependency =
      std::find_if(snapshot.dependencies.begin(), snapshot.dependencies.end(),
                   [&](const auto &candidate) {
                     return candidate.path == include_path.string();
                   });
  ASSERT_NE(dependency, snapshot.dependencies.end());
  EXPECT_FALSE(dependency->sticky);
  EXPECT_FALSE(snapshot.dirty);
  EXPECT_EQ(state.xmlEditorDocument().title, include_path.filename().string());
}

TEST(UrdfEditorState, FileWidgetSavePersistsEditorWidgetDraft) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_widget_save_test");
  const auto path = writeTempFile("dashboard_widget_save.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_widget_save");
  editor_widget.configure(editor_widget.getDefaultConfig());
  rviz2_urdf_editor::UrdfXacroFileWidget file_widget;
  file_widget.initialize(node, "file_widget_save");
  auto config = file_widget.getDefaultConfig();
  config["path"] = path.string();
  file_widget.configure(config);
  QApplication::processEvents();

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  ASSERT_NE(editor, nullptr);
  const auto edited =
      std::string(kSimpleUrdf) + "\n<!-- persisted from editor widget -->\n";
  editor->setPlainText(QString::fromStdString(edited));
  QApplication::processEvents();
  EXPECT_TRUE(state.snapshot().dirty);

  QPushButton *save_button = nullptr;
  for (auto *button : file_widget.widget()->findChildren<QPushButton *>()) {
    if (button->text() == "Save") {
      save_button = button;
      break;
    }
  }
  ASSERT_NE(save_button, nullptr);
  save_button->click();
  QApplication::processEvents();

  EXPECT_NE(readFile(path).find("persisted from editor widget"),
            std::string::npos);
  EXPECT_FALSE(state.snapshot().dirty);
  ASSERT_FALSE(state.snapshot().dependencies.empty());
  EXPECT_FALSE(state.snapshot().dependencies.front().sticky);
}

TEST(UrdfEditorState, EditorWidgetTogglesLineWrap) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_line_wrap_test");
  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_line_wrap");
  editor_widget.configure(editor_widget.getDefaultConfig());

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  ASSERT_NE(editor, nullptr);
  QCheckBox *wrap_check = nullptr;
  for (auto *check_box : editor_widget.widget()->findChildren<QCheckBox *>()) {
    if (check_box->text() == "Wrap") {
      wrap_check = check_box;
      break;
    }
  }
  ASSERT_NE(wrap_check, nullptr);
  EXPECT_FALSE(wrap_check->isChecked());
  EXPECT_EQ(editor->lineWrapMode(), QPlainTextEdit::NoWrap);

  wrap_check->setChecked(true);
  EXPECT_EQ(editor->lineWrapMode(), QPlainTextEdit::WidgetWidth);
  wrap_check->setChecked(false);
  EXPECT_EQ(editor->lineWrapMode(), QPlainTextEdit::NoWrap);
}

TEST(UrdfEditorState, EditorWidgetDoesNotShowReloadButton) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_no_reload_test");
  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_no_reload");
  editor_widget.configure(editor_widget.getDefaultConfig());

  for (auto *button : editor_widget.widget()->findChildren<QPushButton *>()) {
    EXPECT_NE(button->text(), "Reload");
  }
}

TEST(UrdfEditorState, EditorWidgetFormatsXmlWithTwoSpaceIndentOnApply) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_format_test");
  const auto path = writeTempFile("dashboard_format.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_format");
  editor_widget.configure(editor_widget.getDefaultConfig());

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  ASSERT_NE(editor, nullptr);
  auto *apply_button = [&]() -> QPushButton * {
    for (auto *button : editor_widget.widget()->findChildren<QPushButton *>()) {
      if (button->text() == "Apply") {
        return button;
      }
    }
    return nullptr;
  }();
  ASSERT_NE(apply_button, nullptr);

  const auto compact_xml = std::string(
      "<robot name=\"format_bot\"><link name=\"base_link\"><visual><geometry>"
      "<box size=\"1 1 1\"/></geometry></visual></link></robot>");
  editor->setPlainText(QString::fromStdString(compact_xml));
  QApplication::processEvents();
  apply_button->click();
  QApplication::processEvents();

  const auto formatted = editor->toPlainText().toStdString();
  EXPECT_NE(formatted.find("\n  <link name=\"base_link\">"), std::string::npos);
  EXPECT_NE(formatted.find("\n    <visual>"), std::string::npos);
  EXPECT_NE(formatted.find("\n      <geometry>"), std::string::npos);
  EXPECT_NE(formatted.find("\n        <box size=\"1 1 1\" />"),
            std::string::npos);
  EXPECT_EQ(state.snapshot().model.robot_name, "format_bot");

  ASSERT_TRUE(editor->document()->isUndoAvailable());
  editor->undo();
  QApplication::processEvents();
  EXPECT_EQ(editor->toPlainText().toStdString(), compact_xml);
}

TEST(UrdfEditorState, EditorWidgetPreservesBlankLinesOnApply) {
  auto node =
      std::make_shared<rclcpp::Node>("urdf_editor_blank_lines_apply_test");
  const auto path = writeTempFile("dashboard_blank_lines_apply.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_blank_lines_apply");
  editor_widget.configure(editor_widget.getDefaultConfig());

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  ASSERT_NE(editor, nullptr);
  auto *apply_button = [&]() -> QPushButton * {
    for (auto *button : editor_widget.widget()->findChildren<QPushButton *>()) {
      if (button->text() == "Apply") {
        return button;
      }
    }
    return nullptr;
  }();
  ASSERT_NE(apply_button, nullptr);

  const auto edited = std::string(
      "<robot name=\"blank_lines_bot\">\n"
      "  <link name=\"base_link\" />\n"
      "\n"
      "  <link name=\"arm_link\" />\n"
      "\n"
      "  <joint name=\"arm_joint\" type=\"fixed\">\n"
      "    <parent link=\"base_link\" />\n"
      "\n"
      "\n"
      "    <child link=\"arm_link\" />\n"
      "  </joint>\n"
      "</robot>");
  editor->setPlainText(QString::fromStdString(edited));
  QApplication::processEvents();
  apply_button->click();
  QApplication::processEvents();

  const auto formatted = editor->toPlainText().toStdString();
  EXPECT_EQ(formatted, edited);
  EXPECT_EQ(state.snapshot().model.robot_name, "blank_lines_bot");
}

TEST(UrdfEditorState, EditorWidgetShowsCompactValidStatus) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_compact_status_test");
  const auto path = writeTempFile("dashboard_compact_status.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_compact_status");
  editor_widget.configure(editor_widget.getDefaultConfig());
  QApplication::processEvents();

  bool found_compact_status = false;
  for (auto *label : editor_widget.widget()->findChildren<QLabel *>()) {
    if (label->text() == "Editor is valid. Editing whole XML.") {
      found_compact_status = true;
      break;
    }
  }
  EXPECT_TRUE(found_compact_status);
}

TEST(UrdfEditorState, EditorWidgetSelectionUsesGutterOnly) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_gutter_selection_test");
  const auto path = writeTempFile("dashboard_gutter_selection.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  state.openSourceElementEditor("link", "base_link", "Edit Link XML: base_link");

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_gutter_selection");
  editor_widget.configure(editor_widget.getDefaultConfig());
  QApplication::processEvents();

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  ASSERT_NE(editor, nullptr);
  EXPECT_TRUE(editor->extraSelections().empty());
  EXPECT_FALSE(editor->textCursor().hasSelection());
  EXPECT_NE(editor_widget.widget()->findChild<QWidget *>("xml_editor_gutter"),
            nullptr);
}

TEST(UrdfEditorState, EditorWidgetHighlightsMultilineComments) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_comment_highlight_test");
  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_comment_highlight");
  editor_widget.configure(editor_widget.getDefaultConfig());

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  ASSERT_NE(editor, nullptr);
  editor->setPlainText(
      "<robot>\n"
      "  <!--distortion>\n"
      "    <k1>-0.043693598</k1>\n"
      "  </distortion-->\n"
      "</robot>\n");
  QApplication::processEvents();

  const auto block = editor->document()->findBlockByNumber(2);
  ASSERT_TRUE(block.isValid());
  EXPECT_EQ(block.userState(), 1);
  ASSERT_NE(block.layout(), nullptr);
  bool found_comment_format = false;
  for (const auto &range : block.layout()->formats()) {
    if (range.format.fontItalic()) {
      found_comment_format = true;
      break;
    }
  }
  EXPECT_TRUE(found_comment_format);
}

TEST(UrdfEditorState, EditorWidgetGutterFoldToggleHidesInteriorBlocks) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_gutter_fold_test");
  const auto path = writeTempFile("dashboard_gutter_fold.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_gutter_fold");
  editor_widget.configure(editor_widget.getDefaultConfig());
  editor_widget.widget()->resize(700, 500);
  editor_widget.widget()->show();
  QApplication::processEvents();

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  auto *gutter = editor_widget.widget()->findChild<QWidget *>("xml_editor_gutter");
  ASSERT_NE(editor, nullptr);
  ASSERT_NE(gutter, nullptr);
  ASSERT_TRUE(editor->document()->findBlockByNumber(2).isValid());
  ASSERT_TRUE(editor->document()->findBlockByNumber(3).isValid());
  EXPECT_TRUE(editor->document()->findBlockByNumber(3).isVisible());

  QTextCursor cursor(editor->document()->findBlockByNumber(2));
  const auto y = editor->cursorRect(cursor).center().y();
  QMouseEvent collapse(QEvent::MouseButtonPress, QPoint(gutter->width() - 4, y),
                       Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(gutter, &collapse);
  QApplication::processEvents();
  EXPECT_FALSE(editor->document()->findBlockByNumber(3).isVisible());

  QMouseEvent expand(QEvent::MouseButtonPress, QPoint(gutter->width() - 4, y),
                     Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(gutter, &expand);
  QApplication::processEvents();
  EXPECT_TRUE(editor->document()->findBlockByNumber(3).isVisible());
}

TEST(UrdfEditorState, EditorWidgetFoldsResetAfterDocumentReload) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_gutter_fold_reset_test");
  const auto path = writeTempFile("dashboard_gutter_fold_reset_a.urdf", kSimpleUrdf);
  const auto second_path = writeTempFile(
      "dashboard_gutter_fold_reset_b.urdf",
      "<robot name=\"second_bot\">\n"
      "  <link name=\"base_link\">\n"
      "    <visual/>\n"
      "  </link>\n"
      "</robot>\n");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  rviz2_urdf_editor::UrdfXmlEditorWidget editor_widget;
  editor_widget.initialize(node, "xml_editor_gutter_fold_reset");
  editor_widget.configure(editor_widget.getDefaultConfig());
  editor_widget.widget()->resize(700, 500);
  editor_widget.widget()->show();
  QApplication::processEvents();

  auto *editor = editor_widget.widget()->findChild<QPlainTextEdit *>();
  auto *gutter = editor_widget.widget()->findChild<QWidget *>("xml_editor_gutter");
  ASSERT_NE(editor, nullptr);
  ASSERT_NE(gutter, nullptr);
  QTextCursor cursor(editor->document()->findBlockByNumber(2));
  QMouseEvent collapse(QEvent::MouseButtonPress,
                       QPoint(gutter->width() - 4,
                              editor->cursorRect(cursor).center().y()),
                       Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(gutter, &collapse);
  QApplication::processEvents();
  EXPECT_FALSE(editor->document()->findBlockByNumber(3).isVisible());

  ASSERT_TRUE(state.loadFile(second_path.string(), {})) << state.snapshot().last_error;
  QApplication::processEvents();
  EXPECT_TRUE(editor->document()->findBlockByNumber(2).isVisible());
}

TEST(UrdfEditorState, MeshAlphaSliderDefersHeavyStateUpdatesWhileDragging) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_alpha_drag_test");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  state.setMeshAlphaMultiplier(1.0);

  rviz2_urdf_editor::UrdfXacroFileWidget file_widget;
  file_widget.initialize(node, "file_widget_alpha_drag");
  file_widget.configure(file_widget.getDefaultConfig());

  auto *slider = file_widget.widget()->findChild<QSlider *>();
  ASSERT_NE(slider, nullptr);
  slider->setSliderDown(true);
  slider->setValue(42);
  QApplication::processEvents();
  EXPECT_DOUBLE_EQ(state.snapshot().mesh_alpha_multiplier, 1.0);

  slider->setSliderDown(false);
  slider->setValue(43);
  QApplication::processEvents();
  EXPECT_DOUBLE_EQ(state.snapshot().mesh_alpha_multiplier, 0.43);
  state.setMeshAlphaMultiplier(1.0);
}

TEST(UrdfEditorState, PublishesRobotDescriptionTransientLocal) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_state_test");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  state.setNode(node);
  state.setPublishTopic("/test_robot_description");
  const auto path = writeTempFile("dashboard_publish.urdf", kSimpleUrdf);
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  state.setMeshAlphaMultiplier(0.5);

  std::string received;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/test_robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      [&received](const std_msgs::msg::String &msg) { received = msg.data; });
  ASSERT_TRUE(state.publishNow()) << state.snapshot().last_error;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NE(received.find("test_bot"), std::string::npos);
  EXPECT_NE(received.find("rgba=\"1 1 1 0.5\""), std::string::npos);
  received.clear();
  state.setLinkGeometryVisible("base_link", false);
  ASSERT_TRUE(state.publishNow()) << state.snapshot().last_error;
  const auto hide_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.empty() && std::chrono::steady_clock::now() < hide_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(received.find("meshes/base.stl"), std::string::npos);
  state.setLinkGeometryVisible("base_link", true);
  state.setMeshAlphaMultiplier(1.0);
}

TEST(UrdfEditorState, MeshAlphaLocalizesReferencedMaterials) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_material_alpha_test");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  state.setNode(node);
  state.setPublishTopic("/test_material_alpha_robot_description");
  const auto path =
      writeTempFile("dashboard_referenced_material.urdf", kReferencedMaterialUrdf);
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  state.setMeshAlphaMultiplier(0.25);

  std::string received;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/test_material_alpha_robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      [&received](const std_msgs::msg::String &msg) { received = msg.data; });
  ASSERT_TRUE(state.publishNow()) << state.snapshot().last_error;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NE(received.find("rgba=\"0.7 0.7 0.7 0.25\""), std::string::npos);
  EXPECT_EQ(received.find("<material name=\"IPA/LightGrey\"/>"),
            std::string::npos);
  state.setMeshAlphaMultiplier(1.0);
}

TEST(UrdfEditorState, MeshAlphaRewritesColladaMaterials) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_collada_alpha_test");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  state.setNode(node);
  state.setPublishTopic("/test_collada_alpha_robot_description");
  const auto dae_path = writeTempFile("dashboard_alpha_mesh.dae", kColorDae);
  const auto urdf_path = writeTempFile(
      "dashboard_alpha_mesh.urdf",
      "<robot name=\"dae_alpha_bot\"><link name=\"base_link\"><visual>"
      "<geometry><mesh filename=\"" +
          dae_path.string() +
          "\"/></geometry></visual></link></robot>");
  ASSERT_TRUE(state.loadFile(urdf_path.string(), {})) << state.snapshot().last_error;
  state.setMeshAlphaMultiplier(0.5);

  std::string received;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/test_collada_alpha_robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      [&received](const std_msgs::msg::String &msg) { received = msg.data; });
  ASSERT_TRUE(state.publishNow()) << state.snapshot().last_error;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(received.empty());
  const std::string prefix = "file:///tmp/rviz2_urdf_editor/alpha_";
  const auto begin = received.find(prefix);
  ASSERT_NE(begin, std::string::npos);
  const auto end = received.find(".dae", begin);
  ASSERT_NE(end, std::string::npos);
  const auto alpha_dae_path =
      received.substr(begin + std::string("file://").size(),
                      end + std::string(".dae").size() -
                          begin - std::string("file://").size());
  ASSERT_TRUE(std::filesystem::exists(alpha_dae_path));
  const auto alpha_dae = readFile(alpha_dae_path);
  EXPECT_NE(alpha_dae.find("0.2 0.3 0.4 0.4"), std::string::npos);
  EXPECT_NE(alpha_dae.find("<transparent opaque=\"A_ONE\">"), std::string::npos);
  EXPECT_NE(alpha_dae.find("<color>1 1 1 0.5</color>"), std::string::npos);
  EXPECT_NE(alpha_dae.find("<transparency>"), std::string::npos);
  EXPECT_NE(alpha_dae.find("<float>0.500000</float>"), std::string::npos);
  EXPECT_EQ(received.find("dashboard_mesh_alpha"), std::string::npos);
  state.setMeshAlphaMultiplier(1.0);
}

TEST(UrdfEditorState, NotifiesStateChanges) {
  const auto path = writeTempFile("dashboard_notify.urdf", kSimpleUrdf);
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  int notifications = 0;
  const auto callback_id =
      state.addChangeCallback([&notifications]() { ++notifications; });

  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  EXPECT_GT(notifications, 0);

  const auto previous_notifications = notifications;
  state.removeChangeCallback(callback_id);
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;
  EXPECT_EQ(notifications, previous_notifications);
}

TEST(UrdfEditorState, SelectedLinkPublishesHighlightedRobotDescription) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_highlight_test");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  state.setNode(node);
  state.setPublishTopic("/test_highlight_robot_description");
  const auto path = writeTempFile("dashboard_highlight.urdf", kSimpleUrdf);
  ASSERT_TRUE(state.loadFile(path.string(), {})) << state.snapshot().last_error;

  std::vector<std::string> received;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/test_highlight_robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      [&received](const std_msgs::msg::String &msg) {
        received.push_back(msg.data);
      });

  state.setSelectedLink("base_link");
  const auto first_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.empty() && std::chrono::steady_clock::now() < first_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(received.empty());
  EXPECT_NE(received.back().find("dashboard_selection_highlight"),
            std::string::npos);
  EXPECT_NE(received.back().find("base_link"), std::string::npos);

  state.setSelectedLink("base_link");
  const auto previous_count = received.size();
  const auto second_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.size() == previous_count &&
         std::chrono::steady_clock::now() < second_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(received.size(), previous_count);
  EXPECT_EQ(received.back().find("dashboard_selection_highlight"),
            std::string::npos);
}

TEST(UrdfEditorState, SelectedLinkRewritesDaeMeshesForHighlight) {
  auto node = std::make_shared<rclcpp::Node>("urdf_editor_dae_highlight_test");
  auto &state = rviz2_urdf_editor::UrdfEditorState::instance();
  state.setNode(node);
  state.setPublishTopic("/test_dae_highlight_robot_description");
  state.setHighlightColor("#ff0000");

  const auto dae_path = writeTempFile("dashboard_highlight_mesh.dae", kSimpleDae);
  const auto urdf_path = writeTempFile(
      "dashboard_highlight_dae.urdf",
      "<robot name=\"dae_bot\"><link name=\"base_link\"><visual><geometry><mesh filename=\"" +
          dae_path.string() + "\"/></geometry></visual></link></robot>");
  ASSERT_TRUE(state.loadFile(urdf_path.string(), {})) << state.snapshot().last_error;

  std::string received;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/test_dae_highlight_robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      [&received](const std_msgs::msg::String &msg) { received = msg.data; });

  state.setSelectedLink("base_link");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(received.empty());
  const std::string prefix = "file:///tmp/rviz2_urdf_editor/highlight_";
  const auto begin = received.find(prefix);
  ASSERT_NE(begin, std::string::npos);
  const auto end = received.find(".dae", begin);
  ASSERT_NE(end, std::string::npos);
  const auto highlighted_path =
      received.substr(begin + std::string("file://").size(),
                      end + std::string(".dae").size() -
                          begin - std::string("file://").size());
  ASSERT_TRUE(std::filesystem::exists(highlighted_path));
  const auto highlighted_dae = readFile(highlighted_path);
  EXPECT_NE(highlighted_dae.find("1 0 0 1.0"), std::string::npos);
  EXPECT_EQ(highlighted_dae.find("texture=\"paint\""), std::string::npos);
}

TEST(UrdfEditorState, WidgetsShareSingletonState) {
  const auto path = writeTempFile("dashboard_shared_state.urdf", kSimpleUrdf);
  ASSERT_TRUE(
      rviz2_urdf_editor::UrdfEditorState::instance().loadFile(path.string(), {}))
      << rviz2_urdf_editor::UrdfEditorState::instance()
             .snapshot()
             .last_error;
  rviz2_urdf_editor::UrdfModelTreeWidget first;
  rviz2_urdf_editor::UrdfMeshListWidget second;
  EXPECT_EQ(first.category(), "URDF");
  EXPECT_EQ(second.category(), "URDF");
  EXPECT_EQ(rviz2_urdf_editor::UrdfEditorState::instance()
                .snapshot()
                .model.robot_name,
            "test_bot");
}

TEST(UrdfEditorState, FileWidgetLoadStartsRobotStatePublisher) {
  auto &runtime =
      rviz2_urdf_editor::RobotStatePublisherRuntime::instance();
  runtime.stop();

  auto node = std::make_shared<rclcpp::Node>("urdf_file_widget_load_test");
  const auto path = writeTempFile("dashboard_file_widget_load.urdf", kSimpleUrdf);

  rviz2_urdf_editor::UrdfXacroFileWidget widget;
  widget.initialize(node, "file_widget");
  auto config = widget.getDefaultConfig();
  config["path"] = path.string();
  config["rsp_node_name"] = "dashboard_rsp_from_file_widget_test";
  config["joint_states_topic"] = "/file_widget_joint_states";
  widget.configure(config);

  QPushButton *load_button = nullptr;
  for (auto *button : widget.widget()->findChildren<QPushButton *>()) {
    if (button->text() == "Load") {
      load_button = button;
      break;
    }
  }
  ASSERT_NE(load_button, nullptr);
  load_button->click();

  const auto snapshot = runtime.snapshot();
  EXPECT_TRUE(snapshot.enabled);
  EXPECT_TRUE(snapshot.running);
  EXPECT_EQ(snapshot.settings.node_name,
            "dashboard_rsp_from_file_widget_test");
  EXPECT_EQ(snapshot.settings.joint_states_topic, "/file_widget_joint_states");
  EXPECT_NE(std::find(snapshot.arguments.begin(), snapshot.arguments.end(),
                      "--log-level"),
            snapshot.arguments.end());
  EXPECT_NE(std::find(snapshot.arguments.begin(), snapshot.arguments.end(),
                      "warn"),
            snapshot.arguments.end());

  runtime.stop();
}

TEST(UrdfEditorState, FileWidgetAutoPublishesWithoutManualPublishButton) {
  auto node = std::make_shared<rclcpp::Node>("urdf_file_widget_auto_publish_test");
  rviz2_urdf_editor::UrdfXacroFileWidget widget;
  widget.initialize(node, "file_widget_auto_publish");
  widget.configure(widget.getDefaultConfig());

  for (auto *button : widget.widget()->findChildren<QPushButton *>()) {
    EXPECT_NE(button->text(), "Publish Now");
  }
  EXPECT_TRUE(widget.getDefaultConfig()["auto_publish"].as<bool>());
  EXPECT_TRUE(rviz2_urdf_editor::UrdfEditorState::instance()
                  .snapshot()
                  .auto_publish);
}

TEST(RobotStatePublisherRuntime, StartsOnlyOneSharedRuntime) {
  auto &runtime =
      rviz2_urdf_editor::RobotStatePublisherRuntime::instance();
  rviz2_urdf_editor::RobotStatePublisherSettings settings;
  settings.node_name = "dashboard_rsp_test";
  settings.robot_description_topic = "/test_robot_description";
  settings.joint_states_topic = "/test_joint_states";

  const bool started = runtime.apply(settings, true);
  EXPECT_TRUE(started) << runtime.snapshot().last_error;
  EXPECT_TRUE(runtime.snapshot().enabled);
  EXPECT_TRUE(runtime.snapshot().running);

  const bool restarted = runtime.apply(settings, true);
  EXPECT_TRUE(restarted) << runtime.snapshot().last_error;
  EXPECT_TRUE(runtime.snapshot().running);
  runtime.stop();
  EXPECT_FALSE(runtime.snapshot().running);
}

int main(int argc, char **argv) {
  setenv("QT_QPA_PLATFORM", "offscreen", 0);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  const int result = RUN_ALL_TESTS();
  rviz2_urdf_editor::RobotStatePublisherRuntime::instance().stop();
  rviz2_urdf_editor::UrdfEditorState::instance().setNode(nullptr);
  rclcpp::shutdown();
  return result;
}
