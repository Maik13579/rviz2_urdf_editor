#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QColor>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPointer>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QSyntaxHighlighter>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include "rviz2_urdf_editor/robot_state_publisher_runtime.hpp"
#include "rviz2_urdf_editor/urdf_editor_state.hpp"

namespace rviz2_urdf_editor {

struct ConfigField {};

struct WidgetSizeHint {
  int width{1};
  int height{1};
};

class WidgetBase {
public:
  virtual ~WidgetBase();
  void initialize(const rclcpp::Node::SharedPtr &node,
                  const std::string &widget_id);
  void setDisplayContext(rviz_common::DisplayContext *display_context);
  QWidget *widget();
  std::string category() const;
  bool validateConfig(const YAML::Node &config,
                      std::string &error_message) const;

protected:
  static bool requireTopic(const YAML::Node &config, const std::string &key,
                           std::string &error_message);
  void watchUrdfState(UrdfEditorState::ChangeCallback callback);
  void clearUrdfStateWatchers();
  bool hasUrdfStateWatchers() const;

  rclcpp::Node::SharedPtr node_;
  std::string widget_id_;
  rviz_common::DisplayContext *display_context_{nullptr};
  QWidget *root_widget_{nullptr};

private:
  std::vector<std::uint64_t> urdf_state_callback_ids_;
};

class UrdfXacroFileWidget : public WidgetBase {
public:
  UrdfXacroFileWidget();
  void setDisplayContext(rviz_common::DisplayContext *display_context);
  void configure(const YAML::Node &config);
  std::string type() const;
  std::string displayName() const;
  std::string description() const;
  YAML::Node getDefaultConfig() const;
  std::vector<ConfigField> getConfigFields() const;
  WidgetSizeHint getDefaultSizeHint() const;
  bool validateConfig(const YAML::Node &config,
                      std::string &error_message) const;

private:
  void syncFromState();
  void updateRobotModelAlpha();
  void commitMeshAlpha();

  QLineEdit *path_edit_{nullptr};
  QSlider *mesh_alpha_slider_{nullptr};
  QLabel *mesh_alpha_label_{nullptr};
  std::string publish_topic_{"/robot_description"};
  RobotStatePublisherSettings rsp_settings_;
  bool auto_publish_{false};
  double mesh_alpha_multiplier_{1.0};
};

class UrdfDependencyTreeWidget : public WidgetBase {
public:
  UrdfDependencyTreeWidget();
  void configure(const YAML::Node &config);
  std::string type() const;
  std::string displayName() const;
  std::string description() const;
  YAML::Node getDefaultConfig() const;
  std::vector<ConfigField> getConfigFields() const;
  WidgetSizeHint getDefaultSizeHint() const;

private:
  void refresh();
  void openFileEditor(const std::string &path);
  void openContextMenu(const QPoint &position);
  QTreeWidget *tree_{nullptr};
};

class UrdfXmlEditorWidget : public WidgetBase {
public:
  UrdfXmlEditorWidget();
  void configure(const YAML::Node &config);
  std::string type() const;
  std::string displayName() const;
  std::string description() const;
  YAML::Node getDefaultConfig() const;
  std::vector<ConfigField> getConfigFields() const;
  WidgetSizeHint getDefaultSizeHint() const;

private:
  void refresh();
  void validateCurrentXml();
  void applyCurrentXml();
  void reloadCurrentXml();
  QLineEdit *title_edit_{nullptr};
  QLabel *status_label_{nullptr};
  QPlainTextEdit *editor_{nullptr};
  QSyntaxHighlighter *xml_highlighter_{nullptr};
  QCheckBox *line_wrap_check_{nullptr};
  QPushButton *undo_button_{nullptr};
  QPushButton *redo_button_{nullptr};
  QPushButton *apply_button_{nullptr};
  QPushButton *reload_button_{nullptr};
  std::uint64_t seen_document_revision_{0};
  std::string loaded_editor_text_;
  bool updating_from_state_{false};
};

class UrdfModelTreeWidget : public WidgetBase {
public:
  UrdfModelTreeWidget();
  void configure(const YAML::Node &config);
  std::string type() const;
  std::string displayName() const;
  std::string description() const;
  YAML::Node getDefaultConfig() const;
  std::vector<ConfigField> getConfigFields() const;
  WidgetSizeHint getDefaultSizeHint() const;

private:
  void refresh();
  void openLinkEditor(const std::string &link_name);
  void openIncomingJointEditor(const std::string &link_name);
  void openContextMenu(const QPoint &position);
  QCheckBox *select_all_geometry_check_{nullptr};
  QTreeWidget *tree_{nullptr};
  QColor highlight_color_{"#ffd10d"};
};

class UrdfXacroMacroWidget : public WidgetBase {
public:
  UrdfXacroMacroWidget();
  void configure(const YAML::Node &config);
  std::string type() const;
  std::string displayName() const;
  std::string description() const;
  YAML::Node getDefaultConfig() const;
  std::vector<ConfigField> getConfigFields() const;
  WidgetSizeHint getDefaultSizeHint() const;

private:
  void refresh();
  void openElementEditor(QTreeWidgetItem *item);
  void openContextMenu(QTreeWidget *tree, const QPoint &position);
  QTreeWidget *macro_tree_{nullptr};
  QTreeWidget *arg_tree_{nullptr};
  QTreeWidget *property_tree_{nullptr};
};

class UrdfMeshListWidget : public WidgetBase {
public:
  UrdfMeshListWidget();
  void configure(const YAML::Node &config);
  std::string type() const;
  std::string displayName() const;
  std::string description() const;
  YAML::Node getDefaultConfig() const;
  std::vector<ConfigField> getConfigFields() const;
  WidgetSizeHint getDefaultSizeHint() const;

private:
  void refresh();
  QTableWidget *table_{nullptr};
};

class JointStatePublisherWidget : public WidgetBase {
public:
  JointStatePublisherWidget();
  void initialize(const rclcpp::Node::SharedPtr &node,
                  const std::string &widget_id);
  void configure(const YAML::Node &config);
  std::string type() const;
  std::string displayName() const;
  std::string description() const;
  YAML::Node getDefaultConfig() const;
  std::vector<ConfigField> getConfigFields() const;
  WidgetSizeHint getDefaultSizeHint() const;
  bool validateConfig(const YAML::Node &config,
                      std::string &error_message) const;

private:
  struct JointControlRow {
    UrdfJointSummary joint;
    QSlider *slider{nullptr};
    QDoubleSpinBox *spin_box{nullptr};
    QLabel *value_label{nullptr};
  };

  void rebuildSliders();
  void syncModelFromState();
  void publish();
  double jointValue(const JointControlRow &row) const;

  QCheckBox *auto_publish_check_{nullptr};
  QLabel *status_label_{nullptr};
  QWidget *slider_container_{nullptr};
  QTimer *state_timer_{nullptr};
  std::vector<JointControlRow> joint_controls_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::string publisher_topic_;
  std::string topic_{"/joint_states"};
  std::uint64_t seen_state_revision_{0};
  bool auto_publish_{true};
};

class UrdfEditorPanel : public rviz_common::Panel {
public:
  explicit UrdfEditorPanel(QWidget *parent = nullptr);
  void onInitialize() override;

private:
  void initializeWidgets(const rclcpp::Node::SharedPtr &node,
                         rviz_common::DisplayContext *display_context);

  UrdfXacroFileWidget file_widget_;
  UrdfXmlEditorWidget xml_editor_widget_;
};

class UrdfNavPanel : public rviz_common::Panel {
public:
  explicit UrdfNavPanel(QWidget *parent = nullptr);
  void onInitialize() override;

private:
  void initializeWidgets(const rclcpp::Node::SharedPtr &node,
                         rviz_common::DisplayContext *display_context);

  QTabWidget *tabs_{nullptr};
  UrdfModelTreeWidget model_tree_widget_;
  UrdfDependencyTreeWidget dependency_tree_widget_;
  JointStatePublisherWidget joint_state_widget_;
  UrdfXacroMacroWidget xacro_macro_widget_;
  UrdfMeshListWidget mesh_list_widget_;
};

} // namespace rviz2_urdf_editor
