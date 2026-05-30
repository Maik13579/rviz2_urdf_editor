// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "rviz2_urdf_editor/urdf_editor_state.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <iterator>
#include <set>
#include <sstream>

#include <QProcess>
#include <QStringList>
#include <QTemporaryDir>
#include <QTemporaryFile>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pugixml.hpp>

#include "rviz2_urdf_editor/robot_state_publisher_runtime.hpp"

namespace rviz2_urdf_editor {
namespace {

std::string readFile(const std::string &path) {
  std::ifstream input(path);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string comparableEditorText(const std::string &text) {
  std::string normalized;
  normalized.reserve(text.size());
  for (char ch : text) {
    if (ch != '\r') {
      normalized.push_back(ch);
    }
  }
  while (!normalized.empty() && normalized.back() == '\n') {
    normalized.pop_back();
  }
  return normalized;
}

std::string fileTitle(const std::string &path) {
  if (path.empty()) {
    return "No XML selected";
  }
  return std::filesystem::path(path).filename().string();
}

std::string fileTitle(const std::string &path, const bool sticky) {
  auto title = fileTitle(path);
  if (sticky && !title.empty()) {
    title += "*";
  }
  return title;
}

bool writeFile(const std::string &path, const std::string &text) {
  std::ofstream output(path);
  output << text;
  return output.good();
}

std::string attr(const pugi::xml_node &node, const char *name) {
  const auto value = node.attribute(name);
  return value ? value.as_string() : "";
}

std::string nodeNameForKind(const std::string &kind) {
  if (kind == "macro") {
    return "xacro:macro";
  }
  if (kind == "property") {
    return "xacro:property";
  }
  if (kind == "arg") {
    return "xacro:arg";
  }
  return kind;
}

std::string elementKindForNodeName(const std::string &node_name) {
  if (node_name == "xacro:macro") {
    return "macro";
  }
  if (node_name == "xacro:property") {
    return "property";
  }
  if (node_name == "xacro:arg") {
    return "arg";
  }
  if (node_name == "link" || node_name == "joint") {
    return node_name;
  }
  return {};
}

bool isNameChar(const char ch) {
  return std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' ||
         ch == '-' || ch == ':' || ch == '.';
}

std::size_t findTagEnd(const std::string &text, const std::size_t open) {
  char quote = '\0';
  for (std::size_t i = open + 1; i < text.size(); ++i) {
    const char ch = text[i];
    if (quote != '\0') {
      if (ch == quote) {
        quote = '\0';
      }
      continue;
    }
    if (ch == '"' || ch == '\'') {
      quote = ch;
    } else if (ch == '>') {
      return i + 1;
    }
  }
  return std::string::npos;
}

std::string tagNameAt(const std::string &text, std::size_t pos) {
  if (pos >= text.size() || text[pos] != '<') {
    return {};
  }
  ++pos;
  if (pos < text.size() && text[pos] == '/') {
    ++pos;
  }
  const auto begin = pos;
  while (pos < text.size() && isNameChar(text[pos])) {
    ++pos;
  }
  return text.substr(begin, pos - begin);
}

std::string attributeInOpenTag(const std::string &open_tag,
                               const std::string &attribute_name) {
  pugi::xml_document doc;
  auto self_contained = open_tag;
  const auto close = self_contained.find_last_of('>');
  if (close != std::string::npos &&
      (close == 0 || self_contained[close - 1] != '/')) {
    self_contained.insert(close, "/");
  }
  const auto wrapped = "<root>" + self_contained + "</root>";
  const auto result =
      doc.load_string(wrapped.c_str(), pugi::parse_default | pugi::parse_fragment);
  if (!result) {
    return {};
  }
  return attr(doc.child("root").first_child(), attribute_name.c_str());
}

bool isSelfClosingTag(const std::string &text, const std::size_t tag_end) {
  if (tag_end == 0) {
    return false;
  }
  std::size_t pos = tag_end - 1;
  while (pos > 0 && std::isspace(static_cast<unsigned char>(text[pos - 1]))) {
    --pos;
  }
  return pos > 0 && text[pos - 1] == '/';
}

std::size_t findElementEnd(const std::string &text, const std::size_t begin,
                           const std::string &name) {
  const auto first_tag_end = findTagEnd(text, begin);
  if (first_tag_end == std::string::npos || isSelfClosingTag(text, first_tag_end)) {
    return first_tag_end;
  }

  int depth = 1;
  std::size_t pos = first_tag_end;
  while (pos < text.size()) {
    const auto open = text.find('<', pos);
    if (open == std::string::npos) {
      return std::string::npos;
    }
    if (text.compare(open, 4, "<!--") == 0) {
      const auto close = text.find("-->", open + 4);
      if (close == std::string::npos) {
        return std::string::npos;
      }
      pos = close + 3;
      continue;
    }
    if (open + 1 < text.size() && (text[open + 1] == '?' || text[open + 1] == '!')) {
      const auto close = findTagEnd(text, open);
      if (close == std::string::npos) {
        return std::string::npos;
      }
      pos = close;
      continue;
    }

    const bool closing = open + 1 < text.size() && text[open + 1] == '/';
    const auto current_name = tagNameAt(text, open);
    const auto close = findTagEnd(text, open);
    if (close == std::string::npos) {
      return std::string::npos;
    }
    if (current_name == name) {
      if (closing) {
        --depth;
        if (depth == 0) {
          return close;
        }
      } else if (!isSelfClosingTag(text, close)) {
        ++depth;
      }
    }
    pos = close;
  }
  return std::string::npos;
}

std::vector<UrdfSourceElement> scanSourceElements(const std::string &source_path,
                                                  const std::string &text) {
  std::vector<UrdfSourceElement> elements;
  for (std::size_t pos = 0; pos < text.size();) {
    const auto open = text.find('<', pos);
    if (open == std::string::npos) {
      break;
    }
    if (text.compare(open, 4, "<!--") == 0) {
      const auto close = text.find("-->", open + 4);
      pos = close == std::string::npos ? text.size() : close + 3;
      continue;
    }
    if (open + 1 >= text.size() || text[open + 1] == '/' ||
        text[open + 1] == '?' || text[open + 1] == '!') {
      const auto close = findTagEnd(text, open);
      pos = close == std::string::npos ? text.size() : close;
      continue;
    }
    const auto name = tagNameAt(text, open);
    const auto kind = elementKindForNodeName(name);
    const auto open_end = findTagEnd(text, open);
    if (open_end == std::string::npos) {
      break;
    }
    if (kind.empty()) {
      pos = open_end;
      continue;
    }
    const auto element_name =
        attributeInOpenTag(text.substr(open, open_end - open), "name");
    if (element_name.empty()) {
      pos = open_end;
      continue;
    }
    const auto end = findElementEnd(text, open, name);
    if (end == std::string::npos) {
      pos = open_end;
      continue;
    }
    elements.push_back(
        {kind, element_name, source_path, open, end, true, "Editable source XML."});
    pos = kind == "macro" ? open_end : end;
  }
  return elements;
}

void collectIncludedSourcePaths(const pugi::xml_node &node,
                                const std::filesystem::path &base,
                                const std::string &source_path,
                                std::vector<std::string> &paths);

bool isXacroInvocation(const std::string &node_name) {
  return node_name.rfind("xacro:", 0) == 0 && node_name != "xacro:macro" &&
         node_name != "xacro:include";
}

bool generatedNameMatches(const std::string &generated_name,
                          const std::string &argument_value) {
  if (argument_value.empty() || argument_value.find("${") != std::string::npos ||
      argument_value.find("$(") != std::string::npos ||
      generated_name.size() < argument_value.size()) {
    return false;
  }
  if (generated_name == argument_value) {
    return true;
  }
  return generated_name.rfind(argument_value + "_", 0) == 0;
}

int generatedNameScore(const std::string &generated_name,
                       const std::string &argument_name,
                       const std::string &argument_value) {
  if (!generatedNameMatches(generated_name, argument_value)) {
    return -1;
  }
  int score = static_cast<int>(argument_value.size());
  if (argument_name.find("prefix") != std::string::npos ||
      argument_name.find("name") != std::string::npos) {
    score += 1000;
  }
  if (argument_name.find("parent") != std::string::npos) {
    score -= 500;
  }
  return score;
}

struct GeneratedXacroSourceResult {
  std::string source_path;
  std::string source_text;
  std::size_t invocation_begin{0};
  std::size_t invocation_end{0};
  bool has_invocation_range{false};
  std::string status;
};

void scoreXacroInvocationsInText(
    const std::string &source_path, const std::string &source_text,
    const std::string &generated_name,
    const std::map<std::string, std::string> &macro_paths,
    GeneratedXacroSourceResult &best_result, int &best_score) {
  for (std::size_t pos = 0; pos < source_text.size();) {
    const auto open = source_text.find('<', pos);
    if (open == std::string::npos) {
      break;
    }
    if (source_text.compare(open, 4, "<!--") == 0) {
      const auto close = source_text.find("-->", open + 4);
      pos = close == std::string::npos ? source_text.size() : close + 3;
      continue;
    }
    if (open + 1 >= source_text.size() || source_text[open + 1] == '/' ||
        source_text[open + 1] == '?' || source_text[open + 1] == '!') {
      const auto close = findTagEnd(source_text, open);
      pos = close == std::string::npos ? source_text.size() : close;
      continue;
    }
    const auto node_name = tagNameAt(source_text, open);
    const auto open_end = findTagEnd(source_text, open);
    if (open_end == std::string::npos) {
      break;
    }
    if (!isXacroInvocation(node_name)) {
      pos = open_end;
      continue;
    }

    auto open_tag = source_text.substr(open, open_end - open);
    if (!isSelfClosingTag(source_text, open_end)) {
      const auto close = open_tag.find_last_of('>');
      if (close != std::string::npos) {
        open_tag.insert(close, "/");
      }
    }
    pugi::xml_document doc;
    const auto wrapped = "<root xmlns:xacro=\"http://www.ros.org/wiki/xacro\">" +
                         open_tag + "</root>";
    if (doc.load_string(wrapped.c_str(), pugi::parse_default |
                                             pugi::parse_fragment)) {
      const auto invocation = doc.child("root").first_child();
      for (const auto &attribute : invocation.attributes()) {
        const auto score = generatedNameScore(
            generated_name, attribute.name(), attribute.as_string());
        if (score > best_score) {
          best_score = score;
          best_result.source_path = source_path;
          best_result.source_text = source_text;
          best_result.invocation_begin = open;
          best_result.invocation_end = open_end;
          best_result.has_invocation_range = true;
          best_result.status =
              "Editing xacro source. The marked range is the likely xacro "
              "invocation that generated the selected XML, not the generated "
              "XML definition.";
        }
      }
    }

    const auto macro_name = node_name.substr(std::string("xacro:").size());
    const auto macro_path = macro_paths.find(macro_name);
    if (!best_result.has_invocation_range && macro_path != macro_paths.end()) {
      best_result.source_path = macro_path->second;
      best_result.status =
          "Editing xacro source that generated XML; no exact source range was found.";
    }
    pos = open_end;
  }
}

GeneratedXacroSourceResult findGeneratedXacroSource(
    const std::string &source_path, const std::string &source_text,
    const std::map<std::string, std::string> &dependency_overlays,
    const std::string &generated_name) {
  std::vector<std::pair<std::string, std::string>> sources{
      {source_path, source_text}};

  pugi::xml_document doc;
  if (doc.load_string(source_text.c_str())) {
    std::vector<std::string> include_paths;
    collectIncludedSourcePaths(doc, std::filesystem::path(source_path).parent_path(),
                               source_path, include_paths);
    std::set<std::string> seen{source_path};
    for (std::size_t index = 0; index < include_paths.size(); ++index) {
      const auto path = include_paths[index];
      if (!seen.insert(path).second || !std::filesystem::exists(path)) {
        continue;
      }
      const auto overlay = dependency_overlays.find(path);
      const auto text =
          overlay == dependency_overlays.end() ? readFile(path) : overlay->second;
      sources.push_back({path, text});

      pugi::xml_document include_doc;
      if (include_doc.load_string(text.c_str())) {
        collectIncludedSourcePaths(include_doc, std::filesystem::path(path).parent_path(),
                                   path, include_paths);
      }
    }
  }

  std::map<std::string, std::string> macro_paths;
  for (const auto &[path, text] : sources) {
    pugi::xml_document source_doc;
    if (!source_doc.load_string(text.c_str())) {
      continue;
    }
    for (const auto &macro : source_doc.child("robot").children("xacro:macro")) {
      const auto name = attr(macro, "name");
      if (!name.empty()) {
        macro_paths[name] = path;
      }
    }
  }

  GeneratedXacroSourceResult best_result;
  int best_score = -1;
  for (const auto &[path, text] : sources) {
    scoreXacroInvocationsInText(path, text, generated_name, macro_paths,
                                best_result, best_score);
  }
  if (!best_result.source_path.empty() && best_result.source_text.empty()) {
    const auto overlay = dependency_overlays.find(best_result.source_path);
    best_result.source_text =
        overlay == dependency_overlays.end() ? readFile(best_result.source_path)
                                             : overlay->second;
  }
  return best_result;
}

void collectIncludedSourcePaths(const pugi::xml_node &node,
                                const std::filesystem::path &base,
                                const std::string &source_path,
                                std::vector<std::string> &paths) {
  for (const auto &child : node.children()) {
    const std::string name = child.name();
    if (name == "xacro:include" || name == "include") {
      const std::string filename = attr(child, "filename");
      if (!filename.empty() && filename.find('*') == std::string::npos) {
        const std::string resolved =
            UrdfEditorState::resolveMeshPath(filename, source_path);
        if (resolved.empty() && filename.find("$(") != std::string::npos) {
          collectIncludedSourcePaths(child, base, source_path, paths);
          continue;
        }
        const auto path = resolved.empty() ? (base / filename).lexically_normal().string()
                                           : resolved;
        paths.push_back(path);
      }
    }
    collectIncludedSourcePaths(child, base, source_path, paths);
  }
}

void rewriteIncludeFilenames(pugi::xml_node node,
                             const std::filesystem::path &base,
                             const std::string &source_path,
                             const std::map<std::string, std::string> &replacements) {
  for (auto child : node.children()) {
    const std::string name = child.name();
    if (name == "xacro:include" || name == "include") {
      auto filename = child.attribute("filename");
      if (filename) {
        const std::string value = filename.as_string();
        const std::string resolved = UrdfEditorState::resolveMeshPath(value, source_path);
        const auto path = resolved.empty()
                              ? (base / value).lexically_normal().string()
                              : resolved;
        const auto replacement = replacements.find(path);
        if (replacement != replacements.end()) {
          filename.set_value(replacement->second.c_str());
        }
      }
    }
    rewriteIncludeFilenames(child, base, source_path, replacements);
  }
}

std::string sourceWithRewrittenIncludes(
    const std::string &text, const std::string &source_path,
    const std::map<std::string, std::string> &replacements) {
  if (replacements.empty()) {
    return text;
  }
  pugi::xml_document doc;
  if (!doc.load_string(text.c_str())) {
    return text;
  }
  rewriteIncludeFilenames(doc, std::filesystem::path(source_path).parent_path(),
                          source_path, replacements);
  std::ostringstream output;
  doc.save(output);
  return output.str();
}

pugi::xml_node parseSnippetElement(const std::string &xml,
                                   pugi::xml_document &doc) {
  const std::string wrapped =
      "<root xmlns:xacro=\"http://www.ros.org/wiki/xacro\">" + xml + "</root>";
  const auto result =
      doc.load_string(wrapped.c_str(), pugi::parse_default | pugi::parse_fragment);
  if (!result) {
    return {};
  }
  auto root = doc.child("root");
  pugi::xml_node element;
  for (auto child : root.children()) {
    if (child.type() != pugi::node_element) {
      continue;
    }
    if (element) {
      return {};
    }
    element = child;
  }
  return element;
}

bool validateSnippet(const std::string &kind, const std::string &name,
                     const std::string &xml, std::string &error) {
  pugi::xml_document doc;
  const auto element = parseSnippetElement(xml, doc);
  if (!element) {
    error = "Edited XML must contain exactly one well-formed element.";
    return false;
  }
  if (std::string(element.name()) != nodeNameForKind(kind)) {
    error = "Edited XML must remain a " + nodeNameForKind(kind) + " element.";
    return false;
  }
  if (attr(element, "name") != name) {
    error = "Edited XML must keep name=\"" + name + "\".";
    return false;
  }
  return true;
}

std::string xmlEscape(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
    case '&':
      escaped += "&amp;";
      break;
    case '"':
      escaped += "&quot;";
      break;
    case '<':
      escaped += "&lt;";
      break;
    case '>':
      escaped += "&gt;";
      break;
    default:
      escaped += ch;
      break;
    }
  }
  return escaped;
}

std::string lowerExtension(const std::string &path) {
  auto extension = std::filesystem::path(path).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return extension;
}

std::string colorToRgba(const std::string &color) {
  if (color.size() == 7 && color[0] == '#') {
    try {
      const auto r = std::stoi(color.substr(1, 2), nullptr, 16) / 255.0;
      const auto g = std::stoi(color.substr(3, 2), nullptr, 16) / 255.0;
      const auto b = std::stoi(color.substr(5, 2), nullptr, 16) / 255.0;
      std::ostringstream rgba;
      rgba << r << ' ' << g << ' ' << b << " 1.0";
      return rgba.str();
    } catch (const std::exception &) {
    }
  }
  return "1.0 0.82 0.05 1.0";
}

std::vector<double> parseDoubles(const std::string &value) {
  std::vector<double> values;
  std::istringstream stream(value);
  double entry = 0.0;
  while (stream >> entry) {
    values.push_back(entry);
  }
  return values;
}

std::string formatRgba(const std::vector<double> &values,
                       const double alpha_multiplier) {
  const double r = values.size() > 0 ? values[0] : 1.0;
  const double g = values.size() > 1 ? values[1] : 1.0;
  const double b = values.size() > 2 ? values[2] : 1.0;
  const double a = values.size() > 3 ? values[3] : 1.0;
  std::ostringstream output;
  output << r << ' ' << g << ' ' << b << ' '
         << std::clamp(a * alpha_multiplier, 0.0, 1.0);
  return output.str();
}

bool isColladaMaterialColor(const pugi::xml_node &color) {
  const auto parent_name = std::string(color.parent().name());
  return parent_name == "ambient" || parent_name == "diffuse" ||
         parent_name == "emission" || parent_name == "reflective" ||
         parent_name == "specular";
}

void setColladaColorAlpha(pugi::xml_node color, const double alpha_multiplier) {
  const auto values = parseDoubles(color.text().as_string());
  color.text().set(formatRgba(values, alpha_multiplier).c_str());
}

void setColladaMaterialTransparency(pugi::xml_node material,
                                    const double alpha_multiplier) {
  auto transparent = material.child("transparent");
  if (!transparent) {
    transparent = material.append_child("transparent");
  }
  auto opaque = transparent.attribute("opaque");
  if (!opaque) {
    opaque = transparent.append_attribute("opaque");
  }
  opaque.set_value("A_ONE");

  auto transparent_color = transparent.child("color");
  if (!transparent_color) {
    transparent_color = transparent.append_child("color");
  }
  std::ostringstream transparent_rgba;
  transparent_rgba << "1 1 1 " << std::clamp(alpha_multiplier, 0.0, 1.0);
  transparent_color.text().set(transparent_rgba.str().c_str());

  auto transparency = material.child("transparency");
  if (!transparency) {
    transparency = material.append_child("transparency");
  }
  auto transparency_float = transparency.child("float");
  if (!transparency_float) {
    transparency_float = transparency.append_child("float");
  }
  transparency_float.text().set(std::to_string(
      std::clamp(alpha_multiplier, 0.0, 1.0)).c_str());
}

void multiplyColladaMaterialAlpha(pugi::xml_node node,
                                  const double alpha_multiplier) {
  const auto node_name = std::string(node.name());
  if (node_name == "phong" || node_name == "blinn" ||
      node_name == "lambert" || node_name == "constant") {
    setColladaMaterialTransparency(node, alpha_multiplier);
  }
  if (std::string(node.name()) == "color" && isColladaMaterialColor(node)) {
    setColladaColorAlpha(node, alpha_multiplier);
  }
  for (auto child : node.children()) {
    multiplyColladaMaterialAlpha(child, alpha_multiplier);
  }
}

std::string alphaAdjustedColladaUri(const std::string &uri,
                                    const std::string &source_path,
                                    const double alpha_multiplier) {
  const auto resolved = UrdfEditorState::resolveMeshPath(uri, source_path);
  if (resolved.empty() || lowerExtension(resolved) != ".dae" ||
      !std::filesystem::exists(resolved)) {
    return {};
  }

  pugi::xml_document doc;
  if (!doc.load_file(resolved.c_str())) {
    return {};
  }
  multiplyColladaMaterialAlpha(doc, alpha_multiplier);

  const auto output_dir =
      std::filesystem::temp_directory_path() / "rviz2_urdf_editor";
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    return {};
  }

  const auto key = resolved + ":" + std::to_string(alpha_multiplier);
  const auto output_path =
      output_dir /
      ("alpha_" + std::to_string(std::hash<std::string>{}(key)) + ".dae");
  if (!doc.save_file(output_path.string().c_str())) {
    return {};
  }
  return "file://" + output_path.string();
}

std::string colorRgba(const pugi::xml_node &material,
                      const std::map<std::string, std::string> &global_materials) {
  const auto color = material.child("color");
  if (color) {
    return attr(color, "rgba");
  }
  const auto material_name = attr(material, "name");
  const auto global = global_materials.find(material_name);
  return global == global_materials.end() ? std::string{} : global->second;
}

std::string applyMeshAlphaMultiplier(const std::string &robot_description,
                                     const std::string &source_path,
                                     const double alpha_multiplier) {
  if (alpha_multiplier >= 0.999) {
    return robot_description;
  }

  pugi::xml_document doc;
  if (!doc.load_string(robot_description.c_str())) {
    return robot_description;
  }

  std::map<std::string, std::string> global_materials;
  const auto robot = doc.child("robot");
  for (const auto &material : robot.children("material")) {
    const auto name = attr(material, "name");
    const auto color = material.child("color");
    if (!name.empty() && color) {
      global_materials[name] = attr(color, "rgba");
    }
  }

  for (auto link : robot.children("link")) {
    for (auto visual : link.children("visual")) {
      auto mesh = visual.child("geometry").child("mesh");
      if (!mesh) {
        continue;
      }
      const auto alpha_mesh_uri =
          alphaAdjustedColladaUri(attr(mesh, "filename"), source_path,
                                  alpha_multiplier);
      if (!alpha_mesh_uri.empty()) {
        auto filename = mesh.attribute("filename");
        if (!filename) {
          filename = mesh.append_attribute("filename");
        }
        filename.set_value(alpha_mesh_uri.c_str());
      }
      auto material = visual.child("material");
      if (!material) {
        if (!alpha_mesh_uri.empty()) {
          continue;
        }
        material = visual.append_child("material");
        material.append_attribute("name") = "dashboard_mesh_alpha";
      }
      const auto rgba = colorRgba(material, global_materials);
      auto color = material.child("color");
      if (!color) {
        color = material.append_child("color");
      }
      const auto values = parseDoubles(rgba);
      const auto adjusted = formatRgba(values, alpha_multiplier);
      auto rgba_attr = color.attribute("rgba");
      if (!rgba_attr) {
        rgba_attr = color.append_attribute("rgba");
      }
      rgba_attr.set_value(adjusted.c_str());
    }
  }

  std::ostringstream output;
  doc.save(output);
  return output.str();
}

std::string removeHiddenLinkGeometry(const std::string &robot_description,
                                     const std::set<std::string> &hidden_links) {
  if (hidden_links.empty()) {
    return robot_description;
  }

  pugi::xml_document doc;
  if (!doc.load_string(robot_description.c_str())) {
    return robot_description;
  }

  const auto robot = doc.child("robot");
  for (auto link : robot.children("link")) {
    if (hidden_links.count(attr(link, "name")) == 0) {
      continue;
    }
    while (auto visual = link.child("visual")) {
      link.remove_child(visual);
    }
    while (auto collision = link.child("collision")) {
      link.remove_child(collision);
    }
  }

  std::ostringstream output;
  doc.save(output);
  return output.str();
}

void replaceColorChildren(pugi::xml_node node, const char *rgba) {
  while (node.first_child()) {
    node.remove_child(node.first_child());
  }
  auto color = node.append_child("color");
  color.text().set(rgba);
}

void colorizeColladaNode(pugi::xml_node node, const char *rgba) {
  const std::string name = node.name();
  if (name == "diffuse" || name == "ambient" || name == "emission" ||
      name == "specular") {
    replaceColorChildren(node, rgba);
  }
  for (auto child : node.children()) {
    colorizeColladaNode(child, rgba);
  }
}

std::string highlightedColladaUri(const std::string &uri,
                                  const std::string &source_path,
                                  const std::string &rgba) {
  const auto resolved = UrdfEditorState::resolveMeshPath(uri, source_path);
  if (resolved.empty() || lowerExtension(resolved) != ".dae" ||
      !std::filesystem::exists(resolved)) {
    return uri;
  }

  pugi::xml_document doc;
  if (!doc.load_file(resolved.c_str())) {
    return uri;
  }
  colorizeColladaNode(doc, rgba.c_str());

  const auto output_dir =
      std::filesystem::temp_directory_path() / "rviz2_urdf_editor";
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    return uri;
  }

  const auto output_path =
      output_dir /
      ("highlight_" + std::to_string(std::hash<std::string>{}(resolved)) + ".dae");
  if (!doc.save_file(output_path.string().c_str())) {
    return uri;
  }
  return "file://" + output_path.string();
}

bool dependencyIsSticky(const UrdfEditorSnapshot &state,
                        const std::map<std::string, std::string> &dependency_overlays,
                        const std::string &path) {
  if (path.empty()) {
    return false;
  }
  const auto differs_from_disk = [](const std::string &file_path,
                                    const std::string &text) {
    return !std::filesystem::exists(file_path) ||
           comparableEditorText(text) != comparableEditorText(readFile(file_path));
  };
  if (path == state.source_path) {
    return differs_from_disk(path, state.source_text);
  }
  const auto overlay = dependency_overlays.find(path);
  return overlay != dependency_overlays.end() &&
         differs_from_disk(path, overlay->second);
}

void collectIncludeDependencies(const pugi::xml_node &node,
                                const std::filesystem::path &base,
                                const std::string &source_path,
                                std::vector<UrdfDependency> &dependencies) {
  for (const auto &child : node.children()) {
    const std::string name = child.name();
    if (name == "xacro:include" || name == "include") {
      const std::string filename = attr(child, "filename");
      if (!filename.empty() && filename.find('*') == std::string::npos) {
        const std::string resolved =
            UrdfEditorState::resolveMeshPath(filename, source_path);
        if (resolved.empty() && filename.find("$(") != std::string::npos) {
          collectIncludeDependencies(child, base, source_path, dependencies);
          continue;
        }
        const auto path = resolved.empty()
                              ? (base / filename).lexically_normal().string()
                              : resolved;
        dependencies.push_back(
            {path, "xacro include", std::filesystem::exists(path)});
      }
    }
    collectIncludeDependencies(child, base, source_path, dependencies);
  }
}

double parseDouble(const std::string &value, const double fallback) {
  if (value.empty()) {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

void collectMeshes(const pugi::xml_node &root, const std::string &source_path,
                   UrdfModelSummary &model) {
  for (const auto &link : root.children("link")) {
    const std::string owner = attr(link, "name");
    for (const std::string role : {"visual", "collision"}) {
      for (const auto &entry : link.children(role.c_str())) {
        const auto mesh = entry.child("geometry").child("mesh");
        const std::string uri = attr(mesh, "filename");
        if (uri.empty()) {
          continue;
        }
        UrdfMeshSummary summary;
        summary.owner = owner;
        summary.role = role;
        summary.uri = uri;
        summary.scale = attr(mesh, "scale");
        summary.resolved_path = UrdfEditorState::resolveMeshPath(uri, source_path);
        summary.exists = !summary.resolved_path.empty() &&
                         std::filesystem::exists(summary.resolved_path);
        model.meshes.push_back(summary);
      }
    }
  }
}

std::string resolvePackageUri(const std::string &uri) {
  const std::string prefix = "package://";
  if (uri.rfind(prefix, 0) != 0) {
    return {};
  }

  const auto rest = uri.substr(prefix.size());
  const auto slash = rest.find('/');
  if (slash == std::string::npos) {
    return {};
  }

  try {
    const auto share = ament_index_cpp::get_package_share_directory(rest.substr(0, slash));
    return (std::filesystem::path(share) / rest.substr(slash + 1)).string();
  } catch (const std::exception &) {
    return {};
  }
}

std::string resolvePackageShareSubstitution(const std::string &uri) {
  const std::vector<std::string> prefixes{"$(find ", "$(find-pkg-share "};
  for (const auto &prefix : prefixes) {
    if (uri.rfind(prefix, 0) != 0) {
      continue;
    }
    const auto close = uri.find(')', prefix.size());
    if (close == std::string::npos) {
      return {};
    }
    const auto package_name = uri.substr(prefix.size(), close - prefix.size());
    if (package_name.empty()) {
      return {};
    }
    try {
      const auto share = ament_index_cpp::get_package_share_directory(package_name);
      auto suffix = uri.substr(close + 1);
      while (!suffix.empty() && suffix.front() == '/') {
        suffix.erase(suffix.begin());
      }
      return (std::filesystem::path(share) / suffix).lexically_normal().string();
    } catch (const std::exception &) {
      return {};
    }
  }
  return {};
}

std::string rootLinkName(const UrdfModelSummary &model) {
  std::set<std::string> child_links;
  for (const auto &joint : model.joints) {
    child_links.insert(joint.child_link);
  }
  for (const auto &link : model.links) {
    if (child_links.count(link) == 0) {
      return link;
    }
  }
  if (!model.links.empty()) {
    return model.links.front();
  }
  return "root";
}

} // namespace

UrdfEditorState &UrdfEditorState::instance() {
  static UrdfEditorState state;
  return state;
}

void UrdfEditorState::setNode(const rclcpp::Node::SharedPtr &node) {
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = node;
  if (!node_) {
    publisher_.reset();
    editor_tf_broadcaster_.reset();
    return;
  }
  publishEditorFrameTransformLocked();
}

UrdfEditorSnapshot UrdfEditorState::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool UrdfEditorState::loadFile(
    const std::string &path, const std::map<std::string, std::string> &xacro_args) {
  std::string robot_description;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path.empty() || !std::filesystem::exists(path)) {
      setErrorLocked("URDF/Xacro file does not exist: " + path);
      return false;
    }

    state_.source_path = path;
    state_.source_text = readFile(path);
    dependency_overlays_.clear();
    state_.xacro_args = xacro_args;
    state_.dirty = false;
    state_.selected_link.clear();
    state_.selected_mesh = {};
    state_.hidden_geometry_links.clear();
    if (!rebuildLocked()) {
      return false;
    }
    xml_editor_.title = fileTitle(state_.source_path, false);
    xml_editor_.kind.clear();
    xml_editor_.name.clear();
    xml_editor_.path = state_.source_path;
    xml_editor_.xml = state_.source_text;
    xml_editor_.status = "Editing loaded source file: " + state_.source_path;
    xml_editor_.editable = true;
    xml_editor_.whole_file = true;
    xml_editor_.has_selection = false;
    xml_editor_.selection_begin = 0;
    xml_editor_.selection_end = 0;
    ++xml_editor_.revision;
    state_.loaded = true;
    setStatusLocked("Loaded " + std::filesystem::path(path).filename().string());
    if (state_.auto_publish) {
      ensurePublisherLocked();
      if (publisher_) {
        std_msgs::msg::String message;
        message.data = activeRobotDescriptionLocked();
        publisher_->publish(message);
      }
    }
    robot_description = activeRobotDescriptionLocked();
  }
  RobotStatePublisherRuntime::instance().restartWithRobotDescription(robot_description);
  notifyChanged();
  return true;
}

bool UrdfEditorState::save() {
  const auto document = xmlEditorDocument();
  if (document.editable && !document.xml.empty()) {
    if (!applyXmlEditorDocument(document.xml)) {
      return false;
    }
  }

  std::string robot_description;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.source_path.empty()) {
      setErrorLocked("No source path is available for Save.");
      return false;
    }
    if (!writeFile(state_.source_path, state_.source_text)) {
      setErrorLocked("Failed to write " + state_.source_path);
      return false;
    }
    for (const auto &[path, text] : dependency_overlays_) {
      if (!writeFile(path, text)) {
        setErrorLocked("Failed to write " + path);
        return false;
      }
    }
    dependency_overlays_.clear();
    state_.dirty = false;
    discoverDependenciesLocked();
    setStatusLocked("Saved " + state_.source_path);
    if (state_.auto_publish) {
      ensurePublisherLocked();
      if (publisher_) {
        std_msgs::msg::String message;
        message.data = activeRobotDescriptionLocked();
        publisher_->publish(message);
      }
    }
    robot_description = activeRobotDescriptionLocked();
  }
  RobotStatePublisherRuntime::instance().restartWithRobotDescription(robot_description);
  notifyChanged();
  return true;
}

bool UrdfEditorState::setSourceText(const std::string &text) {
  std::string robot_description;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.source_text = text;
    state_.dirty = true;
    if (!rebuildLocked()) {
      return false;
    }
    robot_description = activeRobotDescriptionLocked();
  }
  RobotStatePublisherRuntime::instance().restartWithRobotDescription(robot_description);
  notifyChanged();
  return true;
}

std::vector<UrdfSourceElement> UrdfEditorState::sourceElements() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return source_elements_;
}

UrdfSourceElement UrdfEditorState::sourceElement(const std::string &kind,
                                                 const std::string &name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &element : source_elements_) {
    if (element.kind == kind && element.name == name) {
      return element;
    }
  }
  return {kind, name, {}, 0, 0, false,
          "No matching source XML element was found. This item may be generated by xacro."};
}

std::string UrdfEditorState::sourceElementXml(const std::string &kind,
                                              const std::string &name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &element : source_elements_) {
    if (element.kind != kind || element.name != name || element.end < element.begin) {
      continue;
    }
    const auto text = element.source_path == state_.source_path
                          ? state_.source_text
                          : (dependency_overlays_.count(element.source_path) > 0
                                 ? dependency_overlays_.at(element.source_path)
                                 : readFile(element.source_path));
    if (element.end <= text.size()) {
      return text.substr(element.begin, element.end - element.begin);
    }
  }
  return {};
}

std::string UrdfEditorState::expandedElementXml(const std::string &kind,
                                                const std::string &name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  pugi::xml_document doc;
  if (!doc.load_string(state_.expanded_urdf.c_str())) {
    return {};
  }
  const auto robot = doc.child("robot");
  for (auto element : robot.children(nodeNameForKind(kind).c_str())) {
    if (attr(element, "name") == name) {
      std::ostringstream output;
      element.print(output);
      return output.str();
    }
  }
  return {};
}

bool UrdfEditorState::applySourceElementXml(const std::string &kind,
                                            const std::string &name,
                                            const std::string &xml) {
  std::string expanded_urdf;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string error;
    if (!validateSnippet(kind, name, xml, error)) {
      setErrorLocked(error);
      return false;
    }
    const auto preferred_path =
        xml_editor_.kind == kind && xml_editor_.name == name ? xml_editor_.path
                                                             : std::string{};
    auto element = source_elements_.end();
    if (!preferred_path.empty()) {
      element = std::find_if(source_elements_.begin(), source_elements_.end(),
                             [&](const UrdfSourceElement &candidate) {
                               return candidate.kind == kind &&
                                      candidate.name == name &&
                                      candidate.source_path == preferred_path &&
                                      candidate.editable;
                             });
    }
    if (element == source_elements_.end()) {
      element = std::find_if(source_elements_.begin(), source_elements_.end(),
                             [&](const UrdfSourceElement &candidate) {
                               return candidate.kind == kind &&
                                      candidate.name == name && candidate.editable;
                             });
    }
    if (element == source_elements_.end()) {
      setErrorLocked("No editable source XML element was found for " + kind +
                     " '" + name + "'.");
      return false;
    }
    auto previous_source = state_.source_text;
    auto previous_dirty = state_.dirty;
    const auto previous_overlay = dependency_overlays_.find(element->source_path);
    const bool had_previous_overlay =
        previous_overlay != dependency_overlays_.end();
    const auto previous_overlay_text =
        had_previous_overlay ? previous_overlay->second : std::string{};
    auto file_text = element->source_path == state_.source_path
                         ? state_.source_text
                         : (dependency_overlays_.count(element->source_path) > 0
                                ? dependency_overlays_[element->source_path]
                                : readFile(element->source_path));
    if (element->end < element->begin || element->end > file_text.size()) {
      setErrorLocked("Stored source XML range is no longer valid.");
      return false;
    }

    file_text.replace(element->begin, element->end - element->begin, xml);
    if (element->source_path == state_.source_path) {
      state_.source_text = std::move(file_text);
      state_.dirty = true;
    } else {
      dependency_overlays_[element->source_path] = std::move(file_text);
      state_.dirty = true;
    }
    if (!rebuildLocked()) {
      state_.source_text = previous_source;
      state_.dirty = previous_dirty;
      if (element->source_path != state_.source_path) {
        if (had_previous_overlay) {
          dependency_overlays_[element->source_path] = previous_overlay_text;
        } else {
          dependency_overlays_.erase(element->source_path);
        }
      }
      rebuildLocked();
      return false;
    }
    setStatusLocked("Applied " + kind + " XML in memory: " + name);
    if (xml_editor_.path == element->source_path) {
      xml_editor_.title =
          fileTitle(xml_editor_.path,
                    dependencyIsSticky(state_, dependency_overlays_, xml_editor_.path));
      ++xml_editor_.revision;
    }
    if (state_.auto_publish) {
      publishRobotDescriptionLocked(activeRobotDescriptionLocked());
    }
    expanded_urdf = activeRobotDescriptionLocked();
  }
  RobotStatePublisherRuntime::instance().restartWithRobotDescription(expanded_urdf);
  notifyChanged();
  return true;
}

std::string UrdfEditorState::dependencyFileXml(const std::string &path) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (path == state_.source_path) {
    return state_.source_text;
  }
  const auto overlay = dependency_overlays_.find(path);
  if (overlay != dependency_overlays_.end()) {
    return overlay->second;
  }
  if (!path.empty() && std::filesystem::exists(path)) {
    return readFile(path);
  }
  return {};
}

bool UrdfEditorState::applyDependencyFileXml(const std::string &path,
                                             const std::string &xml) {
  std::string expanded_urdf;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path.empty()) {
      setErrorLocked("Dependency path is empty.");
      return false;
    }
    const auto previous_source = state_.source_text;
    const auto previous_dirty = state_.dirty;
    const auto previous_overlay = dependency_overlays_.find(path);
    const bool had_previous_overlay = previous_overlay != dependency_overlays_.end();
    const auto previous_overlay_text =
        had_previous_overlay ? previous_overlay->second : std::string{};
    if (path == state_.source_path) {
      state_.source_text = xml;
      state_.dirty = true;
    } else {
      dependency_overlays_[path] = xml;
      state_.dirty = true;
    }
    if (!rebuildLocked()) {
      if (path == state_.source_path) {
        state_.source_text = previous_source;
      } else if (had_previous_overlay) {
        dependency_overlays_[path] = previous_overlay_text;
      } else {
        dependency_overlays_.erase(path);
      }
      state_.dirty = previous_dirty;
      rebuildLocked();
      return false;
    }
    setStatusLocked("Applied XML changes in memory: " + path);
    if (xml_editor_.path == path) {
      xml_editor_.xml = xml;
      xml_editor_.title =
          fileTitle(path, dependencyIsSticky(state_, dependency_overlays_, path));
      if (xml_editor_.has_selection && !xml_editor_.kind.empty() &&
          !xml_editor_.name.empty()) {
        const auto element =
            std::find_if(source_elements_.begin(), source_elements_.end(),
                         [&](const UrdfSourceElement &candidate) {
                           return candidate.kind == xml_editor_.kind &&
                                  candidate.name == xml_editor_.name &&
                                  candidate.source_path == path &&
                                  candidate.editable;
                         });
        if (element != source_elements_.end() && element->end >= element->begin &&
            element->end <= xml_editor_.xml.size()) {
          xml_editor_.selection_begin = element->begin;
          xml_editor_.selection_end = element->end;
        } else {
          xml_editor_.has_selection = false;
          xml_editor_.selection_begin = 0;
          xml_editor_.selection_end = 0;
        }
      }
      ++xml_editor_.revision;
    }
    if (state_.auto_publish) {
      publishRobotDescriptionLocked(activeRobotDescriptionLocked());
    }
    expanded_urdf = activeRobotDescriptionLocked();
  }
  RobotStatePublisherRuntime::instance().restartWithRobotDescription(expanded_urdf);
  notifyChanged();
  return true;
}

UrdfXmlEditorDocument UrdfEditorState::xmlEditorDocument() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return xml_editor_;
}

void UrdfEditorState::openSourceElementEditor(const std::string &kind,
                                              const std::string &name,
                                              const std::string &title,
                                              const std::string &source_path) {
  (void)title;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto element = source_elements_.end();
    if (!source_path.empty()) {
      element = std::find_if(source_elements_.begin(), source_elements_.end(),
                             [&](const UrdfSourceElement &candidate) {
                               return candidate.kind == kind &&
                                      candidate.name == name &&
                                      candidate.source_path == source_path;
                             });
    }
    if (element == source_elements_.end()) {
      element = std::find_if(source_elements_.begin(), source_elements_.end(),
                             [&](const UrdfSourceElement &candidate) {
                               return candidate.kind == kind &&
                                      candidate.name == name;
                             });
    }
    const bool editable = element != source_elements_.end() && element->editable;
    std::string xml;
    GeneratedXacroSourceResult generated_source;
    if (editable) {
      const auto source_text = element->source_path == state_.source_path
                                   ? state_.source_text
                                   : (dependency_overlays_.count(element->source_path) > 0
                                          ? dependency_overlays_[element->source_path]
                                          : readFile(element->source_path));
      xml = source_text;
    } else {
      pugi::xml_document doc;
      if (doc.load_string(state_.expanded_urdf.c_str())) {
        const auto robot = doc.child("robot");
        for (auto expanded : robot.children(nodeNameForKind(kind).c_str())) {
          if (attr(expanded, "name") == name) {
            std::ostringstream output;
            expanded.print(output);
            xml = output.str();
            break;
          }
        }
      }
      generated_source = findGeneratedXacroSource(
          state_.source_path, state_.source_text, dependency_overlays_, name);
    }
    if (!editable && !generated_source.source_path.empty()) {
      xml_editor_.title =
          fileTitle(generated_source.source_path,
                    dependencyIsSticky(state_, dependency_overlays_,
                                       generated_source.source_path));
      xml_editor_.kind = kind;
      xml_editor_.name = name;
      xml_editor_.path = generated_source.source_path;
      xml_editor_.xml = generated_source.source_text;
      if (xml_editor_.xml.empty()) {
        xml_editor_.xml =
            generated_source.source_path == state_.source_path
                ? state_.source_text
                : (dependency_overlays_.count(generated_source.source_path) > 0
                       ? dependency_overlays_[generated_source.source_path]
                       : readFile(generated_source.source_path));
      }
      xml_editor_.status =
          generated_source.status.empty()
              ? "Editing xacro source that generated " + kind +
                    " XML: " + generated_source.source_path +
                    "; no exact source range was found."
              : generated_source.status + " File: " + generated_source.source_path;
      xml_editor_.editable = true;
      xml_editor_.whole_file = true;
      xml_editor_.has_selection =
          generated_source.has_invocation_range &&
          generated_source.invocation_end >= generated_source.invocation_begin &&
          generated_source.invocation_end <= xml_editor_.xml.size();
      xml_editor_.selection_begin =
          xml_editor_.has_selection ? generated_source.invocation_begin : 0;
      xml_editor_.selection_end =
          xml_editor_.has_selection ? generated_source.invocation_end : 0;
      ++xml_editor_.revision;
    } else if (!editable && xml.empty() && !state_.source_text.empty()) {
      xml_editor_.title =
          fileTitle(state_.source_path,
                    dependencyIsSticky(state_, dependency_overlays_, state_.source_path));
      xml_editor_.kind = kind;
      xml_editor_.name = name;
      xml_editor_.path = state_.source_path;
      xml_editor_.xml = state_.source_text;
      xml_editor_.status = "Editing loaded xacro source; no generated XML range was found.";
      xml_editor_.editable = true;
      xml_editor_.whole_file = true;
      xml_editor_.has_selection = false;
      xml_editor_.selection_begin = 0;
      xml_editor_.selection_end = 0;
      ++xml_editor_.revision;
    } else {
      const auto editor_path = editable ? element->source_path : state_.source_path;
      xml_editor_.title =
          fileTitle(editor_path,
                    dependencyIsSticky(state_, dependency_overlays_, editor_path));
      xml_editor_.kind = kind;
      xml_editor_.name = name;
      xml_editor_.path = editor_path;
      xml_editor_.xml = xml;
      xml_editor_.status =
          editable ? "Editing whole XML file from " + xml_editor_.path
                   : "Read-only expanded XML; no source element was found.";
      xml_editor_.editable = editable;
      xml_editor_.whole_file = editable;
      xml_editor_.has_selection =
          editable && element->end >= element->begin && element->end <= xml.size();
      xml_editor_.selection_begin = xml_editor_.has_selection ? element->begin : 0;
      xml_editor_.selection_end = xml_editor_.has_selection ? element->end : 0;
      ++xml_editor_.revision;
    }
  }
  notifyChanged();
}

void UrdfEditorState::openDependencyFileEditor(const std::string &path) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    xml_editor_.title =
        fileTitle(path, dependencyIsSticky(state_, dependency_overlays_, path));
    xml_editor_.kind.clear();
    xml_editor_.name.clear();
    xml_editor_.path = path;
    xml_editor_.xml = path == state_.source_path ? state_.source_text : readFile(path);
    if (path != state_.source_path && dependency_overlays_.count(path) > 0) {
      xml_editor_.xml = dependency_overlays_[path];
    }
    xml_editor_.status = "Editing whole XML file: " + path;
    xml_editor_.editable = !path.empty() && std::filesystem::exists(path);
    xml_editor_.whole_file = true;
    xml_editor_.has_selection = false;
    xml_editor_.selection_begin = 0;
    xml_editor_.selection_end = 0;
    ++xml_editor_.revision;
  }
  notifyChanged();
}

bool UrdfEditorState::applyXmlEditorDocument(const std::string &xml) {
  const auto document = xmlEditorDocument();
  if (document.whole_file) {
    return applyDependencyFileXml(document.path, xml);
  }
  return applySourceElementXml(document.kind, document.name, xml);
}

void UrdfEditorState::setXmlEditorDraft(const std::string &xml) {
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (xml_editor_.xml == xml) {
      return;
    }
    xml_editor_.xml = xml;

    const auto update_sticky = [&](const std::string &path, const bool sticky) {
      for (auto &dependency : state_.dependencies) {
        if (dependency.path == path) {
          if (dependency.sticky == sticky) {
            break;
          }
          dependency.sticky = sticky;
          changed = true;
          break;
        }
      }
      state_.dirty = std::any_of(
          state_.dependencies.begin(), state_.dependencies.end(),
          [](const UrdfDependency &dependency) { return dependency.sticky; });
    };

    if (xml_editor_.editable && xml_editor_.whole_file && !xml_editor_.path.empty()) {
      const bool sticky =
          !std::filesystem::exists(xml_editor_.path) ||
          comparableEditorText(xml) !=
              comparableEditorText(readFile(xml_editor_.path));
      update_sticky(xml_editor_.path, sticky);
      xml_editor_.title = fileTitle(xml_editor_.path, sticky);
    } else if (xml_editor_.editable && !xml_editor_.kind.empty() &&
               !xml_editor_.name.empty()) {
      const auto element =
          std::find_if(source_elements_.begin(), source_elements_.end(),
                       [&](const UrdfSourceElement &candidate) {
                         return candidate.kind == xml_editor_.kind &&
                                candidate.name == xml_editor_.name &&
                                candidate.editable;
                       });
      if (element != source_elements_.end()) {
        auto file_text =
            element->source_path == state_.source_path
                ? state_.source_text
                : (dependency_overlays_.count(element->source_path) > 0
                       ? dependency_overlays_[element->source_path]
                       : readFile(element->source_path));
        if (element->end >= element->begin && element->end <= file_text.size()) {
          file_text.replace(element->begin, element->end - element->begin, xml);
          const bool sticky = !std::filesystem::exists(element->source_path) ||
                              comparableEditorText(file_text) !=
                                  comparableEditorText(readFile(element->source_path));
          update_sticky(element->source_path, sticky);
          xml_editor_.title = fileTitle(element->source_path, sticky);
        }
      }
    }
  }
  if (changed) {
    notifyChanged();
  }
}

void UrdfEditorState::reloadXmlEditorDocument() {
  const auto document = xmlEditorDocument();
  if (!document.kind.empty() && !document.name.empty()) {
    openSourceElementEditor(document.kind, document.name, document.title,
                            document.path);
  } else if (document.whole_file) {
    openDependencyFileEditor(document.path);
  }
}

void UrdfEditorState::setAutoPublish(const bool enabled) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.auto_publish = enabled;
  }
  notifyChanged();
}

void UrdfEditorState::setPublishTopic(const std::string &topic) {
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!topic.empty() && topic != state_.publish_topic) {
      state_.publish_topic = topic;
      publisher_.reset();
      changed = true;
    }
  }
  if (changed) {
    notifyChanged();
  }
}

bool UrdfEditorState::publishNow() {
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.expanded_urdf.empty()) {
      setErrorLocked("No expanded robot_description is available to publish.");
    } else {
      ok = publishRobotDescriptionLocked(activeRobotDescriptionLocked());
      if (ok) {
        setStatusLocked("Published robot_description on " + state_.publish_topic);
      }
    }
  }
  notifyChanged();
  return ok;
}

void UrdfEditorState::setHighlightColor(const std::string &color) {
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!color.empty() && state_.highlight_color != color) {
      state_.highlight_color = color;
      changed = true;
      if (!state_.selected_link.empty() && !state_.expanded_urdf.empty()) {
        publishRobotDescriptionLocked(activeRobotDescriptionLocked());
      }
    }
  }
  if (changed) {
    notifyChanged();
  }
}

void UrdfEditorState::setMeshAlphaMultiplier(const double multiplier) {
  bool changed = false;
  std::string robot_description;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto clamped = std::clamp(multiplier, 0.0, 1.0);
    if (std::abs(state_.mesh_alpha_multiplier - clamped) < 0.0001) {
      return;
    }
    state_.mesh_alpha_multiplier = clamped;
    robot_description = activeRobotDescriptionLocked();
    if (!robot_description.empty() && publisher_) {
      publishRobotDescriptionLocked(robot_description);
    }
    changed = true;
  }
  if (!robot_description.empty()) {
    RobotStatePublisherRuntime::instance().restartWithRobotDescription(
        robot_description);
  }
  if (changed) {
    notifyChanged();
  }
}

void UrdfEditorState::setLinkGeometryVisible(const std::string &link_name,
                                             const bool visible) {
  if (link_name.empty()) {
    return;
  }

  bool changed = false;
  std::string robot_description;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (visible) {
      changed = state_.hidden_geometry_links.erase(link_name) > 0;
    } else {
      changed = state_.hidden_geometry_links.insert(link_name).second;
    }
    if (!changed) {
      return;
    }
    robot_description = activeRobotDescriptionLocked();
    if (!robot_description.empty() && publisher_) {
      publishRobotDescriptionLocked(robot_description);
    }
  }
  if (!robot_description.empty()) {
    RobotStatePublisherRuntime::instance().restartWithRobotDescription(
        robot_description);
  }
  notifyChanged();
}

void UrdfEditorState::setAllLinkGeometryVisible(const bool visible) {
  bool changed = false;
  std::string robot_description;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.model.links.empty()) {
      return;
    }

    if (visible) {
      changed = !state_.hidden_geometry_links.empty();
      state_.hidden_geometry_links.clear();
    } else {
      const std::set<std::string> hidden_links(state_.model.links.begin(),
                                               state_.model.links.end());
      changed = hidden_links != state_.hidden_geometry_links;
      state_.hidden_geometry_links = hidden_links;
    }

    if (!changed) {
      return;
    }
    robot_description = activeRobotDescriptionLocked();
    if (!robot_description.empty() && publisher_) {
      publishRobotDescriptionLocked(robot_description);
    }
  }
  if (!robot_description.empty()) {
    RobotStatePublisherRuntime::instance().restartWithRobotDescription(
        robot_description);
  }
  notifyChanged();
}

void UrdfEditorState::setSelectedLink(const std::string &link_name) {
  bool changed = false;
  bool ok = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool deselect = !link_name.empty() && state_.selected_link == link_name;
    const auto next_link = deselect ? std::string{} : link_name;
    if (state_.selected_link != next_link || !state_.selected_mesh.uri.empty()) {
      state_.selected_link = next_link;
      state_.selected_mesh = {};
      if (state_.expanded_urdf.empty()) {
        ok = true;
      } else if (state_.selected_link.empty()) {
        ok = publishRobotDescriptionLocked(effectiveRobotDescriptionLocked());
      } else {
        ok = publishRobotDescriptionLocked(activeRobotDescriptionLocked());
      }
      changed = true;
    }
  }
  if (changed || !ok) {
    notifyChanged();
  }
}

bool UrdfEditorState::toggleSelectedMesh(const UrdfMeshSummary &mesh) {
  bool ok = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool deselect = state_.selected_mesh.uri == mesh.uri &&
                          state_.selected_mesh.owner == mesh.owner &&
                          state_.selected_mesh.role == mesh.role;
    if (deselect) {
      state_.selected_mesh = {};
      if (!state_.expanded_urdf.empty()) {
        ok = publishRobotDescriptionLocked(effectiveRobotDescriptionLocked());
      }
    } else {
      state_.selected_mesh = mesh;
      ok = publishRobotDescriptionLocked(activeRobotDescriptionLocked());
    }
  }
  notifyChanged();
  return ok;
}

std::uint64_t UrdfEditorState::addChangeCallback(ChangeCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto callback_id = next_callback_id_++;
  callbacks_[callback_id] = std::move(callback);
  return callback_id;
}

void UrdfEditorState::removeChangeCallback(const std::uint64_t callback_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  callbacks_.erase(callback_id);
}

std::string UrdfEditorState::resolveMeshPath(const std::string &uri,
                                             const std::string &source_path) {
  if (uri.empty()) {
    return {};
  }
  if (uri.rfind("package://", 0) == 0) {
    return resolvePackageUri(uri);
  }
  if (const auto resolved = resolvePackageShareSubstitution(uri);
      !resolved.empty()) {
    return resolved;
  }
  if (uri.rfind("file://", 0) == 0) {
    return uri.substr(std::string("file://").size());
  }

  std::filesystem::path path(uri);
  if (path.is_absolute()) {
    return path.string();
  }
  if (!source_path.empty()) {
    return (std::filesystem::path(source_path).parent_path() / path).lexically_normal().string();
  }
  return path.lexically_normal().string();
}

bool UrdfEditorState::rebuildLocked() {
  std::string expanded;
  std::string error;
  if (!expandXacroLocked(expanded, error)) {
    setErrorLocked(error);
    return false;
  }
  state_.expanded_urdf = expanded;
  indexSourceElementsLocked();
  parseExpandedUrdfLocked();
  discoverDependenciesLocked();
  publishEditorFrameTransformLocked();
  ++state_.revision;
  state_.last_error.clear();
  return true;
}

void UrdfEditorState::publishEditorFrameTransformLocked() {
  if (!node_ || state_.model.links.empty()) {
    return;
  }

  if (!editor_tf_broadcaster_) {
    editor_tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->get_clock()->now();
  transform.header.frame_id = "editor";
  transform.child_frame_id = rootLinkName(state_.model);
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  editor_tf_broadcaster_->sendTransform(transform);
}

bool UrdfEditorState::expandXacroLocked(std::string &expanded,
                                        std::string &error) const {
  if (state_.source_path.empty()) {
    expanded = state_.source_text;
    return true;
  }

  QProcess process;
  QStringList args;
  const auto source_path = std::filesystem::path(state_.source_path);
  QTemporaryDir overlay_dir;
  std::map<std::string, std::string> overlay_paths;
  if (!dependency_overlays_.empty() && !overlay_dir.isValid()) {
    error = "Failed to create temporary directory for edited xacro includes.";
    return false;
  }
  int overlay_index = 0;
  for (const auto &[path, text] : dependency_overlays_) {
    const auto temp_path =
        std::filesystem::path(overlay_dir.path().toStdString()) /
        ("include_" + std::to_string(overlay_index++) +
         std::filesystem::path(path).extension().string());
    overlay_paths[path] = temp_path.string();
  }
  for (const auto &[path, text] : dependency_overlays_) {
    const auto rewritten = sourceWithRewrittenIncludes(text, path, overlay_paths);
    const auto temp_path = overlay_paths[path];
    if (!writeFile(temp_path, rewritten)) {
      error = "Failed to write temporary xacro include.";
      return false;
    }
  }
  QTemporaryFile temp(QString::fromStdString(
      (source_path.parent_path() /
       (".rviz2_urdf_editor_XXXXXX" + source_path.extension().string()))
          .string()));
  temp.setAutoRemove(true);
  if (!temp.open()) {
    error = "Failed to create temporary source file for xacro expansion.";
    return false;
  }
  const auto source_text =
      sourceWithRewrittenIncludes(state_.source_text, state_.source_path,
                                  overlay_paths);
  temp.write(source_text.data(), static_cast<qint64>(source_text.size()));
  temp.flush();

  args << temp.fileName();
  for (const auto &[key, value] : state_.xacro_args) {
    args << QString::fromStdString(key + ":=" + value);
  }
  process.setWorkingDirectory(QString::fromStdString(source_path.parent_path().string()));
  process.start("xacro", args);
  if (!process.waitForFinished(10000)) {
    process.kill();
    error = "xacro expansion timed out.";
    return false;
  }
  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
    error = process.readAllStandardError().toStdString();
    if (error.empty()) {
      error = "xacro expansion failed.";
    }
    return false;
  }
  expanded = process.readAllStandardOutput().toStdString();
  return true;
}

void UrdfEditorState::indexSourceElementsLocked() {
  source_elements_ = scanSourceElements(state_.source_path, state_.source_text);

  pugi::xml_document doc;
  if (!doc.load_string(state_.source_text.c_str())) {
    return;
  }
  std::vector<std::string> include_paths;
  collectIncludedSourcePaths(doc, std::filesystem::path(state_.source_path).parent_path(),
                             state_.source_path, include_paths);
  std::set<std::string> seen{state_.source_path};
  for (std::size_t index = 0; index < include_paths.size(); ++index) {
    const auto path = include_paths[index];
    if (!seen.insert(path).second || !std::filesystem::exists(path)) {
      continue;
    }
    const auto overlay = dependency_overlays_.find(path);
    const auto text =
        overlay == dependency_overlays_.end() ? readFile(path) : overlay->second;
    auto included = scanSourceElements(path, text);
    source_elements_.insert(source_elements_.end(), included.begin(), included.end());

    pugi::xml_document include_doc;
    if (include_doc.load_string(text.c_str())) {
      collectIncludedSourcePaths(include_doc, std::filesystem::path(path).parent_path(),
                                 path, include_paths);
    }
  }
}

void UrdfEditorState::parseExpandedUrdfLocked() {
  state_.model = {};
  pugi::xml_document doc;
  const auto result = doc.load_string(state_.expanded_urdf.c_str());
  if (!result) {
    state_.last_error = std::string("URDF XML parse error: ") + result.description();
    return;
  }

  const auto robot = doc.child("robot");
  state_.model.robot_name = attr(robot, "name");
  for (const auto &material : robot.children("material")) {
    const std::string name = attr(material, "name");
    if (!name.empty()) {
      state_.model.materials.push_back(name);
    }
  }
  for (const auto &link : robot.children("link")) {
    const std::string name = attr(link, "name");
    if (!name.empty()) {
      state_.model.links.push_back(name);
    }
  }
  for (const auto &joint : robot.children("joint")) {
    UrdfJointSummary summary;
    summary.name = attr(joint, "name");
    summary.type = attr(joint, "type");
    summary.parent_link = attr(joint.child("parent"), "link");
    summary.child_link = attr(joint.child("child"), "link");
    const auto limit = joint.child("limit");
    if (limit) {
      summary.lower = parseDouble(attr(limit, "lower"), 0.0);
      summary.upper = parseDouble(attr(limit, "upper"), 0.0);
      summary.has_limits = limit.attribute("lower") && limit.attribute("upper");
    } else if (summary.type == "continuous") {
      summary.lower = -3.14159265358979323846;
      summary.upper = 3.14159265358979323846;
      summary.has_limits = true;
    }
    state_.model.joints.push_back(summary);
  }
  collectMeshes(robot, state_.source_path, state_.model);
}

void UrdfEditorState::discoverDependenciesLocked() {
  state_.dependencies.clear();
  if (!state_.source_path.empty()) {
    const bool exists = std::filesystem::exists(state_.source_path);
    state_.dependencies.push_back(
        {state_.source_path, "source", exists,
         dependencyIsSticky(state_, dependency_overlays_, state_.source_path)});
  }

  pugi::xml_document doc;
  if (!doc.load_string(state_.source_text.c_str())) {
    return;
  }

  const auto base = std::filesystem::path(state_.source_path).parent_path();
  collectIncludeDependencies(doc, base, state_.source_path, state_.dependencies);
  for (auto &dependency : state_.dependencies) {
    dependency.sticky =
        dependencyIsSticky(state_, dependency_overlays_, dependency.path);
  }
  state_.dirty = std::any_of(
      state_.dependencies.begin(), state_.dependencies.end(),
      [](const UrdfDependency &dependency) { return dependency.sticky; });
}

void UrdfEditorState::ensurePublisherLocked() {
  if (!node_) {
    publisher_.reset();
    return;
  }
  if (publisher_ && publisher_topic_ == state_.publish_topic) {
    return;
  }
  publisher_topic_ = state_.publish_topic;
  publisher_ = node_->create_publisher<std_msgs::msg::String>(
      state_.publish_topic, rclcpp::QoS(1).transient_local().reliable());
}

std::string UrdfEditorState::effectiveRobotDescriptionLocked() const {
  return removeHiddenLinkGeometry(
      applyMeshAlphaMultiplier(state_.expanded_urdf, state_.source_path,
                               state_.mesh_alpha_multiplier),
      state_.hidden_geometry_links);
}

std::string UrdfEditorState::activeRobotDescriptionLocked() const {
  if (!state_.selected_mesh.uri.empty()) {
    return selectedMeshRobotDescriptionLocked(state_.selected_mesh);
  }
  if (!state_.selected_link.empty()) {
    return highlightedLinkRobotDescriptionLocked(state_.selected_link);
  }
  return effectiveRobotDescriptionLocked();
}

bool UrdfEditorState::publishRobotDescriptionLocked(
    const std::string &robot_description) {
  ensurePublisherLocked();
  if (!publisher_) {
    setErrorLocked("No ROS node is available for publishing robot_description.");
    return false;
  }
  std_msgs::msg::String message;
  message.data = robot_description;
  publisher_->publish(message);
  return true;
}

std::string UrdfEditorState::selectedMeshRobotDescriptionLocked(
    const UrdfMeshSummary &mesh) const {
  const auto root_link = rootLinkName(state_.model);
  std::ostringstream urdf;
  urdf << "<robot name=\"selected_mesh_preview\">";
  urdf << "<link name=\"" << xmlEscape(root_link) << "\">";
  urdf << "<visual><geometry><mesh filename=\"" << xmlEscape(mesh.uri) << "\"";
  if (!mesh.scale.empty()) {
    urdf << " scale=\"" << xmlEscape(mesh.scale) << "\"";
  }
  urdf << "/></geometry><material name=\"dashboard_mesh_alpha\"><color rgba=\"1 1 1 "
       << std::clamp(state_.mesh_alpha_multiplier, 0.0, 1.0)
       << "\"/></material></visual>";
  urdf << "</link></robot>";
  return urdf.str();
}

std::string UrdfEditorState::highlightedLinkRobotDescriptionLocked(
    const std::string &link_name) const {
  const auto highlight_rgba = colorToRgba(state_.highlight_color);
  pugi::xml_document doc;
  if (!doc.load_string(state_.expanded_urdf.c_str())) {
    return state_.expanded_urdf;
  }

  const auto robot = doc.child("robot");
  for (auto link : robot.children("link")) {
    if (attr(link, "name") != link_name) {
      continue;
    }
    for (const std::string role : {"visual", "collision"}) {
      for (auto entry : link.children(role.c_str())) {
        auto mesh = entry.child("geometry").child("mesh");
        if (mesh) {
          const auto highlighted_uri =
              highlightedColladaUri(attr(mesh, "filename"), state_.source_path,
                                    highlight_rgba);
          mesh.attribute("filename").set_value(highlighted_uri.c_str());
        }
        entry.remove_child("material");
        auto material = entry.append_child("material");
        material.append_attribute("name") = "dashboard_selection_highlight";
        auto color = material.append_child("color");
        color.append_attribute("rgba") = highlight_rgba.c_str();
      }
    }
    break;
  }

  std::ostringstream output;
  doc.save(output);
  return removeHiddenLinkGeometry(
      applyMeshAlphaMultiplier(output.str(), state_.source_path,
                               state_.mesh_alpha_multiplier),
      state_.hidden_geometry_links);
}

void UrdfEditorState::notifyChanged() {
  std::vector<ChangeCallback> callbacks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks.reserve(callbacks_.size());
    for (const auto &[_, callback] : callbacks_) {
      callbacks.push_back(callback);
    }
  }
  for (const auto &callback : callbacks) {
    if (callback) {
      callback();
    }
  }
}

void UrdfEditorState::setErrorLocked(const std::string &message) {
  state_.last_error = message;
  state_.status = message;
}

void UrdfEditorState::setStatusLocked(const std::string &message) {
  state_.status = message;
  state_.last_error.clear();
}

} // namespace rviz2_urdf_editor
