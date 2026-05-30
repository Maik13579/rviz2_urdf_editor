#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace rviz2_urdf_editor {

struct UrdfJointSummary {
  std::string name;
  std::string type;
  std::string parent_link;
  std::string child_link;
  double lower{0.0};
  double upper{0.0};
  bool has_limits{false};
};

struct UrdfMeshSummary {
  std::string owner;
  std::string role;
  std::string uri;
  std::string resolved_path;
  std::string scale;
  bool exists{false};
};

struct UrdfModelSummary {
  std::string robot_name;
  std::vector<std::string> links;
  std::vector<UrdfJointSummary> joints;
  std::vector<std::string> materials;
  std::vector<UrdfMeshSummary> meshes;
};

struct UrdfDependency {
  std::string path;
  std::string kind;
  bool exists{false};
  bool sticky{false};
};

struct UrdfEditorSnapshot {
  std::string source_path;
  std::string source_text;
  std::string expanded_urdf;
  std::map<std::string, std::string> xacro_args;
  std::vector<UrdfDependency> dependencies;
  UrdfModelSummary model;
  bool dirty{false};
  bool loaded{false};
  bool auto_publish{false};
  std::string publish_topic{"/robot_description"};
  std::string status{"No URDF loaded."};
  std::string last_error;
  std::string selected_link;
  UrdfMeshSummary selected_mesh;
  std::string highlight_color{"#ffd10d"};
  double mesh_alpha_multiplier{1.0};
  std::set<std::string> hidden_geometry_links;
  std::uint64_t revision{0};
};

struct UrdfSourceElement {
  std::string kind;
  std::string name;
  std::string source_path;
  std::size_t begin{0};
  std::size_t end{0};
  bool editable{false};
  std::string status;
};

struct UrdfXmlEditorDocument {
  std::string title{"No XML selected"};
  std::string kind;
  std::string name;
  std::string path;
  std::string xml;
  std::string status{"Open XML from the model, macro, or dependency widget."};
  bool editable{false};
  bool whole_file{false};
  bool has_selection{false};
  std::size_t selection_begin{0};
  std::size_t selection_end{0};
  std::uint64_t revision{0};
};

class UrdfEditorState {
public:
  using ChangeCallback = std::function<void()>;

  static UrdfEditorState &instance();

  void setNode(const rclcpp::Node::SharedPtr &node);
  UrdfEditorSnapshot snapshot() const;
  bool loadFile(const std::string &path,
                const std::map<std::string, std::string> &xacro_args);
  bool save();
  bool setSourceText(const std::string &text);
  std::vector<UrdfSourceElement> sourceElements() const;
  UrdfSourceElement sourceElement(const std::string &kind,
                                  const std::string &name) const;
  std::string sourceElementXml(const std::string &kind,
                               const std::string &name) const;
  std::string expandedElementXml(const std::string &kind,
                                 const std::string &name) const;
  bool applySourceElementXml(const std::string &kind, const std::string &name,
                             const std::string &xml);
  std::string dependencyFileXml(const std::string &path) const;
  bool applyDependencyFileXml(const std::string &path, const std::string &xml);
  UrdfXmlEditorDocument xmlEditorDocument() const;
  void openSourceElementEditor(const std::string &kind,
                               const std::string &name,
                               const std::string &title,
                               const std::string &source_path = {});
  void openDependencyFileEditor(const std::string &path);
  bool applyXmlEditorDocument(const std::string &xml);
  void setXmlEditorDraft(const std::string &xml);
  void reloadXmlEditorDocument();
  void setAutoPublish(bool enabled);
  void setPublishTopic(const std::string &topic);
  bool publishNow();
  void setHighlightColor(const std::string &color);
  void setMeshAlphaMultiplier(double multiplier);
  void setLinkGeometryVisible(const std::string &link_name, bool visible);
  void setAllLinkGeometryVisible(bool visible);
  void setSelectedLink(const std::string &link_name);
  bool toggleSelectedMesh(const UrdfMeshSummary &mesh);
  std::uint64_t addChangeCallback(ChangeCallback callback);
  void removeChangeCallback(std::uint64_t callback_id);

  static std::string resolveMeshPath(const std::string &uri,
                                     const std::string &source_path);

private:
  UrdfEditorState() = default;

  bool rebuildLocked();
  bool expandXacroLocked(std::string &expanded, std::string &error) const;
  void indexSourceElementsLocked();
  void parseExpandedUrdfLocked();
  void discoverDependenciesLocked();
  void ensurePublisherLocked();
  void publishEditorFrameTransformLocked();
  std::string effectiveRobotDescriptionLocked() const;
  std::string activeRobotDescriptionLocked() const;
  bool publishRobotDescriptionLocked(const std::string &robot_description);
  std::string selectedMeshRobotDescriptionLocked(const UrdfMeshSummary &mesh) const;
  std::string highlightedLinkRobotDescriptionLocked(
      const std::string &link_name) const;
  void notifyChanged();
  void setErrorLocked(const std::string &message);
  void setStatusLocked(const std::string &message);

  mutable std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> editor_tf_broadcaster_;
  std::string publisher_topic_;
  UrdfEditorSnapshot state_;
  std::vector<UrdfSourceElement> source_elements_;
  UrdfXmlEditorDocument xml_editor_;
  std::map<std::string, std::string> dependency_overlays_;
  std::unordered_map<std::uint64_t, ChangeCallback> callbacks_;
  std::uint64_t next_callback_id_{1};
};

} // namespace rviz2_urdf_editor
