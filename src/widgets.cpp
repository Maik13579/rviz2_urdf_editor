#include "rviz2_urdf_editor/widgets.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <utility>

#include <QBrush>
#include <QFileDialog>
#include <QFontDatabase>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSyntaxHighlighter>
#include <QTextBlock>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <pugixml.hpp>
#include <rcl/validate_topic_name.h>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace rviz2_urdf_editor {
namespace {

bool validateTopicName(const std::string &topic, std::string &error_message) {
  if (topic.empty()) {
    error_message = "Topic must not be empty.";
    return false;
  }
  int validation_result = RCL_TOPIC_NAME_VALID;
  size_t invalid_index = 0;
  if (rcl_validate_topic_name(topic.c_str(), &validation_result,
                              &invalid_index) != RCL_RET_OK ||
      validation_result != RCL_TOPIC_NAME_VALID) {
    error_message = "Invalid topic name.";
    return false;
  }
  return true;
}

YAML::Node emptyConfig() { return YAML::Node(YAML::NodeType::Map); }

YAML::Node normalizeConfig(const YAML::Node &config) {
  YAML::Node normalized = emptyConfig();
  normalized["path"] = "";
  normalized["robot_description_topic"] = "/robot_description";
  normalized["rsp_node_name"] = "rviz2_urdf_editor_robot_state_publisher";
  normalized["rsp_namespace"] = "";
  normalized["joint_states_topic"] = "/joint_states";
  normalized["frame_prefix"] = "";
  normalized["publish_frequency"] = 20.0;
  normalized["ignore_timestamp"] = false;
  normalized["auto_publish"] = true;
  normalized["mesh_alpha_multiplier"] = 1.0;
  normalized["highlight_color"] = "#ffd10d";
  normalized["topic"] = "/joint_states";
  if (config.IsMap()) {
    for (const auto &entry : config) {
      normalized[entry.first.as<std::string>()] = entry.second;
    }
  }
  return normalized;
}

bool requireNonEmptyString(const YAML::Node &config, const std::string &key,
                           std::string &error_message) {
  if (!config[key] || !config[key].IsScalar() ||
      config[key].as<std::string>().empty()) {
    error_message = key + " must not be empty.";
    return false;
  }
  return true;
}

bool requireDoubleRange(const YAML::Node &config, const std::string &key,
                        const double minimum, const double maximum,
                        std::string &error_message) {
  if (!config[key] || !config[key].IsScalar()) {
    error_message = key + " must be set.";
    return false;
  }
  const auto value = config[key].as<double>();
  if (!std::isfinite(value) || value < minimum || value > maximum) {
    error_message = key + " is outside the allowed range.";
    return false;
  }
  return true;
}

ConfigField makeFilePathField(const char *, const char *, bool, const char *) {
  return {};
}

ConfigField makeTopicField(const char *, const char *, bool, const char *,
                           const char *) {
  return {};
}

ConfigField makeStringField(const char *, const char *, bool = true,
                            const char * = "", const char * = "",
                            const char * = "") {
  return {};
}

ConfigField makeDoubleField(const char *, const char *, double, double, double,
                            double, int, bool) {
  return {};
}

ConfigField makeBoolField(const char *, const char *, const char *, bool) {
  return {};
}

ConfigField makeColorField(const char *, const char *, const char *, bool,
                           const char *) {
  return {};
}

std::string formatXmlWithTwoSpaceIndent(const std::string &xml,
                                        const bool whole_file) {
  pugi::xml_document doc;
  const auto parse_options =
      whole_file ? pugi::parse_default
                 : (pugi::parse_default | pugi::parse_fragment);
  const auto parse_input =
      whole_file
          ? xml
          : "<root xmlns:xacro=\"http://www.ros.org/wiki/xacro\">" + xml +
                "</root>";
  const auto result = doc.load_string(parse_input.c_str(), parse_options);
  if (!result) {
    return xml;
  }

  std::ostringstream output;
  if (whole_file) {
    doc.save(output, "  ", pugi::format_default, pugi::encoding_utf8);
    return output.str();
  }

  bool first = true;
  for (const auto child : doc.child("root").children()) {
    if (child.type() != pugi::node_element) {
      continue;
    }
    if (!first) {
      output << '\n';
    }
    child.print(output, "  ", pugi::format_default, pugi::encoding_utf8);
    first = false;
  }
  return output.str();
}

std::string compactEditorStatus(const UrdfXmlEditorDocument &document) {
  if (document.whole_file) {
    return "Editor is valid. Editing whole XML.";
  }
  if (!document.kind.empty()) {
    return "Editor is valid. Editing " + document.kind + " XML.";
  }
  return "Editor is valid.";
}

void setRobotModelAlpha(rviz_common::DisplayGroup *group, const double alpha) {
  if (group == nullptr) {
    return;
  }
  for (int i = 0; i < group->numDisplays(); ++i) {
    auto *display = group->getDisplayAt(i);
    if (display == nullptr) {
      continue;
    }
    if (display->getClassId() == "rviz_default_plugins/RobotModel") {
      display->subProp("Alpha")->setValue(alpha);
    }
    if (auto *child_group = group->getGroupAt(i)) {
      setRobotModelAlpha(child_group, alpha);
    }
  }
}

void clearLayout(QLayout *layout) {
  while (auto *item = layout->takeAt(0)) {
    if (auto *widget = item->widget()) {
      widget->deleteLater();
    }
    delete item;
  }
}

bool isMovableJoint(const UrdfJointSummary &joint) {
  return joint.type != "fixed" && joint.type != "floating" &&
         joint.type != "planar" && !joint.name.empty();
}

bool hasUsableSliderLimits(const UrdfJointSummary &joint) {
  constexpr double kMaxSliderAbsLimit = 1000.0;
  return joint.has_limits && std::isfinite(joint.lower) &&
         std::isfinite(joint.upper) && joint.upper > joint.lower &&
         std::abs(joint.lower) <= kMaxSliderAbsLimit &&
         std::abs(joint.upper) <= kMaxSliderAbsLimit;
}

double spinBoxStep(const UrdfJointSummary &joint) {
  if (joint.type == "prismatic") {
    return 0.01;
  }
  return 0.01;
}

QTableWidgetItem *readOnlyTableItem(const QString &text) {
  auto *item = new QTableWidgetItem(text);
  item->setFlags(item->flags() & ~Qt::ItemIsEditable);
  return item;
}

class FrameTreeWidget : public QTreeWidget {
public:
  explicit FrameTreeWidget(QWidget *parent = nullptr) : QTreeWidget(parent) {}

  void setFrameActivatedCallback(
      std::function<void(QTreeWidgetItem *)> callback) {
    frame_activated_callback_ = std::move(callback);
  }

protected:
  void mouseDoubleClickEvent(QMouseEvent *event) override {
    if (event != nullptr && event->button() == Qt::LeftButton) {
      if (auto *item = itemAt(event->pos())) {
        if (frame_activated_callback_) {
          frame_activated_callback_(item);
          event->accept();
          return;
        }
      }
    }
    QTreeWidget::mouseDoubleClickEvent(event);
  }

private:
  std::function<void(QTreeWidgetItem *)> frame_activated_callback_;
};

class SharedXmlHighlighter : public QSyntaxHighlighter {
public:
  SharedXmlHighlighter(QTextDocument *parent, const QColor &background)
      : QSyntaxHighlighter(parent) {
    setColorsForBackground(background);
  }

  void setColorsForBackground(const QColor &background) {
    const double luminance = 0.2126 * background.redF() +
                             0.7152 * background.greenF() +
                             0.0722 * background.blueF();
    const bool dark_background = luminance < 0.45;
    if (dark_background) {
      tag_format_.setForeground(QColor("#79c0ff"));
      name_format_.setForeground(QColor("#d2a8ff"));
      value_format_.setForeground(QColor("#a5d6ff"));
      comment_format_.setForeground(QColor("#8b949e"));
    } else {
      tag_format_.setForeground(QColor("#005cc5"));
      name_format_.setForeground(QColor("#6f42c1"));
      value_format_.setForeground(QColor("#032f62"));
      comment_format_.setForeground(QColor("#6a737d"));
    }
    comment_format_.setFontItalic(true);
    rehighlight();
  }

protected:
  void highlightBlock(const QString &text) override {
    constexpr int kInsideComment = 1;
    setCurrentBlockState(0);

    apply(QRegularExpression("</?[:A-Za-z_][-.:A-Za-z0-9_]*|/?>"), text,
          tag_format_);
    apply(QRegularExpression("\\b[:A-Za-z_][-.:A-Za-z0-9_]*(?=\\=)"), text,
          name_format_);
    apply(QRegularExpression("\"[^\"]*\"|'[^']*'"), text, value_format_);

    int start = previousBlockState() == kInsideComment ? 0 : text.indexOf("<!--");
    while (start >= 0) {
      const int end = text.indexOf("-->", start);
      const int length = end < 0 ? text.size() - start : end - start + 3;
      setFormat(start, length, comment_format_);
      if (end < 0) {
        setCurrentBlockState(kInsideComment);
        break;
      }
      start = text.indexOf("<!--", start + length);
    }
  }

private:
  void apply(const QRegularExpression &regex, const QString &text,
             const QTextCharFormat &format) {
    auto iterator = regex.globalMatch(text);
    while (iterator.hasNext()) {
      const auto match = iterator.next();
      setFormat(match.capturedStart(), match.capturedLength(), format);
    }
  }

  QTextCharFormat tag_format_;
  QTextCharFormat name_format_;
  QTextCharFormat value_format_;
  QTextCharFormat comment_format_;
};

class SelectionMarkerPlainTextEdit;

class XmlEditorGutter : public QWidget {
public:
  explicit XmlEditorGutter(SelectionMarkerPlainTextEdit *editor);

  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;

private:
  SelectionMarkerPlainTextEdit *editor_{nullptr};
};

class SelectionMarkerPlainTextEdit : public QPlainTextEdit {
public:
  explicit SelectionMarkerPlainTextEdit(QWidget *parent = nullptr)
      : QPlainTextEdit(parent), gutter_(new XmlEditorGutter(this)) {
    setObjectName("xml_editor");
    gutter_->setObjectName("xml_editor_gutter");
    connect(this, &QPlainTextEdit::blockCountChanged, this,
            &SelectionMarkerPlainTextEdit::updateGutterWidth);
    connect(this, &QPlainTextEdit::updateRequest, this,
            &SelectionMarkerPlainTextEdit::updateGutter);
    connect(this, &QPlainTextEdit::textChanged, this,
            &SelectionMarkerPlainTextEdit::recomputeFoldRegionsAfterEdit);
    updateGutterWidth();
  }

  void setHighlightedRange(const int begin, const int end) {
    highlight_begin_ = begin;
    highlight_end_ = end;
    gutter_->update();
  }

  void resetFolds() {
    for (QTextBlock block = document()->firstBlock(); block.isValid();
         block = block.next()) {
      block.setVisible(true);
      block.setLineCount(block.layout() == nullptr ? 0 : block.layout()->lineCount());
    }
    recomputeFoldRegions(false);
    document()->adjustSize();
    viewport()->update();
    gutter_->update();
  }

  int gutterWidth() const {
    int digits = 1;
    for (int max = std::max(1, blockCount()); max >= 10; max /= 10) {
      ++digits;
    }
    return 22 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;
  }

  void paintGutter(QPaintEvent *event) {
    QPainter painter(gutter_);
    painter.fillRect(event->rect(), palette().color(QPalette::Base).darker(104));

    const QColor text_color = palette().color(QPalette::Text);
    const QColor muted_color = text_color.lighter(145);
    const QColor accent("#d4a72c");
    const int number_right = gutter_->width() - 18;
    const int toggle_left = gutter_->width() - 14;

    for (QTextBlock block = firstVisibleBlock(); block.isValid();
         block = block.next()) {
      const auto geometry = blockBoundingGeometry(block).translated(contentOffset());
      const int top = static_cast<int>(geometry.top());
      const int bottom = static_cast<int>(geometry.bottom());
      if (top > event->rect().bottom()) {
        break;
      }
      if (bottom < event->rect().top()) {
        continue;
      }

      const bool highlighted = blockOverlapsHighlight(block);
      const QRect number_rect(0, top, number_right - 2, bottom - top);
      if (highlighted) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(accent);
        painter.drawRoundedRect(number_rect.adjusted(2, 1, -1, -1), 3, 3);
        painter.setPen(Qt::black);
      } else {
        painter.setPen(muted_color);
      }
      painter.drawText(number_rect, Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(block.blockNumber() + 1));

      const auto *region = foldRegionStartingAt(block.blockNumber());
      if (region == nullptr) {
        continue;
      }
      const int center_y = top + std::max(0, bottom - top) / 2;
      const QRect toggle_rect(toggle_left, center_y - 5, 10, 10);
      painter.setPen(text_color);
      painter.setBrush(region->collapsed ? accent.lighter(135) : Qt::NoBrush);
      painter.drawRect(toggle_rect.adjusted(0, 0, -1, -1));
      painter.drawLine(toggle_rect.left() + 2, center_y, toggle_rect.right() - 2,
                       center_y);
      if (region->collapsed) {
        painter.drawLine(toggle_rect.center().x(), toggle_rect.top() + 2,
                         toggle_rect.center().x(), toggle_rect.bottom() - 2);
      }
    }
  }

  void handleGutterClick(const QPoint &position) {
    const QTextBlock block = blockAtY(position.y());
    if (!block.isValid()) {
      return;
    }
    const int toggle_left = gutter_->width() - 16;
    if (position.x() < toggle_left) {
      return;
    }
    toggleFold(block.blockNumber());
  }

protected:
  void paintEvent(QPaintEvent *event) override {
    QPlainTextEdit::paintEvent(event);
  }

  void resizeEvent(QResizeEvent *event) override {
    QPlainTextEdit::resizeEvent(event);
    const QRect contents = contentsRect();
    gutter_->setGeometry(QRect(contents.left(), contents.top(), gutterWidth(),
                               contents.height()));
  }

private:
  struct FoldRegion {
    QString name;
    int start_block{0};
    int end_block{0};
    bool collapsed{false};
  };

  struct OpenTag {
    QString name;
    int start_block{0};
  };

  static bool isNameChar(const QChar ch) {
    return ch.isLetterOrNumber() || ch == '_' || ch == '-' || ch == ':' ||
           ch == '.';
  }

  static int findTagEnd(const QString &text, const int open) {
    QChar quote;
    for (int index = open + 1; index < text.size(); ++index) {
      const QChar ch = text.at(index);
      if (!quote.isNull()) {
        if (ch == quote) {
          quote = QChar();
        }
        continue;
      }
      if (ch == '"' || ch == '\'') {
        quote = ch;
      } else if (ch == '>') {
        return index + 1;
      }
    }
    return -1;
  }

  static QString tagNameAt(const QString &text, int pos) {
    if (pos >= text.size() || text.at(pos) != '<') {
      return {};
    }
    ++pos;
    if (pos < text.size() && text.at(pos) == '/') {
      ++pos;
    }
    const int begin = pos;
    while (pos < text.size() && isNameChar(text.at(pos))) {
      ++pos;
    }
    return text.mid(begin, pos - begin);
  }

  static bool isSelfClosingTag(const QString &text, const int tag_end) {
    int pos = tag_end - 1;
    while (pos > 0 && text.at(pos - 1).isSpace()) {
      --pos;
    }
    return pos > 0 && text.at(pos - 1) == '/';
  }

  void updateGutterWidth() { setViewportMargins(gutterWidth(), 0, 0, 0); }

  void updateGutter(const QRect &rect, const int dy) {
    if (dy != 0) {
      gutter_->scroll(0, dy);
    } else {
      gutter_->update(0, rect.y(), gutter_->width(), rect.height());
    }
    if (rect.contains(viewport()->rect())) {
      updateGutterWidth();
    }
  }

  QTextBlock blockAtY(const int y) const {
    for (QTextBlock block = firstVisibleBlock(); block.isValid();
         block = block.next()) {
      const auto geometry = blockBoundingGeometry(block).translated(contentOffset());
      if (geometry.top() <= y && y <= geometry.bottom()) {
        return block;
      }
      if (geometry.top() > y) {
        break;
      }
    }
    return {};
  }

  bool blockOverlapsHighlight(const QTextBlock &block) const {
    if (highlight_begin_ < 0 || highlight_end_ < highlight_begin_) {
      return false;
    }
    const int block_begin = block.position();
    const int block_end = block_begin + block.length();
    return block_end > highlight_begin_ && block_begin <= highlight_end_;
  }

  const FoldRegion *foldRegionStartingAt(const int block_number) const {
    const auto region =
        std::find_if(fold_regions_.begin(), fold_regions_.end(),
                     [&](const FoldRegion &candidate) {
                       return candidate.start_block == block_number;
                     });
    return region == fold_regions_.end() ? nullptr : &(*region);
  }

  void recomputeFoldRegionsAfterEdit() { recomputeFoldRegions(true); }

  void recomputeFoldRegions(const bool preserve_matching_collapsed) {
    const auto previous_regions = fold_regions_;
    fold_regions_.clear();
    const QString text = toPlainText();
    std::vector<OpenTag> stack;
    for (int pos = 0; pos < text.size();) {
      const int open = text.indexOf('<', pos);
      if (open < 0) {
        break;
      }
      if (text.mid(open, 4) == "<!--") {
        const int close = text.indexOf("-->", open + 4);
        pos = close < 0 ? text.size() : close + 3;
        continue;
      }
      if (open + 1 >= text.size()) {
        break;
      }
      const QChar next = text.at(open + 1);
      const int close = findTagEnd(text, open);
      if (close < 0) {
        break;
      }
      if (next == '?' || next == '!') {
        pos = close;
        continue;
      }
      const QString name = tagNameAt(text, open);
      if (name.isEmpty()) {
        pos = close;
        continue;
      }
      if (next == '/') {
        for (auto tag = stack.rbegin(); tag != stack.rend(); ++tag) {
          if (tag->name != name) {
            continue;
          }
          const int start_block = tag->start_block;
          const int end_block = document()->findBlock(open).blockNumber();
          stack.erase(std::next(tag).base(), stack.end());
          if (end_block > start_block) {
            FoldRegion region{name, start_block, end_block, false};
            if (preserve_matching_collapsed) {
              const auto previous =
                  std::find_if(previous_regions.begin(), previous_regions.end(),
                               [&](const FoldRegion &candidate) {
                                 return candidate.name == region.name &&
                                        candidate.start_block == region.start_block &&
                                        candidate.end_block == region.end_block;
                               });
              region.collapsed =
                  previous != previous_regions.end() && previous->collapsed;
            }
            fold_regions_.push_back(region);
          }
          break;
        }
      } else if (!isSelfClosingTag(text, close)) {
        stack.push_back({name, document()->findBlock(open).blockNumber()});
      }
      pos = close;
    }
    std::sort(fold_regions_.begin(), fold_regions_.end(),
              [](const FoldRegion &lhs, const FoldRegion &rhs) {
                return lhs.start_block < rhs.start_block;
              });
    applyFoldVisibility();
    gutter_->update();
  }

  void toggleFold(const int start_block) {
    auto region = std::find_if(fold_regions_.begin(), fold_regions_.end(),
                               [&](const FoldRegion &candidate) {
                                 return candidate.start_block == start_block;
                               });
    if (region == fold_regions_.end()) {
      return;
    }
    region->collapsed = !region->collapsed;
    applyFoldVisibility();
    document()->adjustSize();
    viewport()->update();
    gutter_->update();
  }

  void applyFoldVisibility() {
    std::set<int> hidden_blocks;
    for (const auto &region : fold_regions_) {
      if (!region.collapsed) {
        continue;
      }
      for (int block = region.start_block + 1; block <= region.end_block;
           ++block) {
        hidden_blocks.insert(block);
      }
    }
    for (QTextBlock block = document()->firstBlock(); block.isValid();
         block = block.next()) {
      const bool visible = hidden_blocks.count(block.blockNumber()) == 0;
      block.setVisible(visible);
      if (!visible) {
        block.setLineCount(0);
      } else if (block.layout() != nullptr && block.layout()->lineCount() <= 0) {
        block.setLineCount(1);
      }
    }
    document()->markContentsDirty(0, document()->characterCount());
  }

  XmlEditorGutter *gutter_{nullptr};
  int highlight_begin_{-1};
  int highlight_end_{-1};
  std::vector<FoldRegion> fold_regions_;
};

XmlEditorGutter::XmlEditorGutter(SelectionMarkerPlainTextEdit *editor)
    : QWidget(editor), editor_(editor) {}

QSize XmlEditorGutter::sizeHint() const {
  return QSize(editor_ == nullptr ? 0 : editor_->gutterWidth(), 0);
}

void XmlEditorGutter::paintEvent(QPaintEvent *event) {
  if (editor_ != nullptr) {
    editor_->paintGutter(event);
  }
}

void XmlEditorGutter::mousePressEvent(QMouseEvent *event) {
  if (editor_ != nullptr && event->button() == Qt::LeftButton) {
    editor_->handleGutterClick(event->pos());
    event->accept();
    return;
  }
  QWidget::mousePressEvent(event);
}

} // namespace

WidgetBase::~WidgetBase() {
  clearUrdfStateWatchers();
  if (root_widget_ != nullptr) {
    for (auto *timer : root_widget_->findChildren<QTimer *>()) {
      timer->stop();
    }
    QObject::disconnect(root_widget_, nullptr, nullptr, nullptr);
    for (auto *child : root_widget_->findChildren<QObject *>()) {
      QObject::disconnect(child, nullptr, nullptr, nullptr);
    }
  }
  if (root_widget_ != nullptr && root_widget_->parent() == nullptr) {
    delete root_widget_;
  }
  root_widget_ = nullptr;
}

void WidgetBase::clearUrdfStateWatchers() {
  for (const auto callback_id : urdf_state_callback_ids_) {
    UrdfEditorState::instance().removeChangeCallback(callback_id);
  }
  urdf_state_callback_ids_.clear();
}

void WidgetBase::initialize(const rclcpp::Node::SharedPtr &node,
                            const std::string &widget_id) {
  node_ = node;
  widget_id_ = widget_id;
  UrdfEditorState::instance().setNode(node);
}

void WidgetBase::setDisplayContext(rviz_common::DisplayContext *display_context) {
  display_context_ = display_context;
}

QWidget *WidgetBase::widget() { return root_widget_; }

std::string WidgetBase::category() const { return "URDF"; }

bool WidgetBase::validateConfig(const YAML::Node &, std::string &error) const {
  error.clear();
  return true;
}

bool WidgetBase::requireTopic(const YAML::Node &config, const std::string &key,
                              std::string &error_message) {
  if (!requireNonEmptyString(config, key, error_message)) {
    return false;
  }
  return validateTopicName(config[key].as<std::string>(), error_message);
}

void WidgetBase::watchUrdfState(UrdfEditorState::ChangeCallback callback) {
  if (!node_) {
    return;
  }
  const QPointer<QWidget> root_guard(root_widget_);
  urdf_state_callback_ids_.push_back(
      UrdfEditorState::instance().addChangeCallback(
          [root_guard, callback = std::move(callback)]() {
            if (!root_guard || !callback) {
              return;
            }
            QTimer::singleShot(0, root_guard.data(),
                               [root_guard, callback]() {
                                 if (root_guard) {
                                   callback();
                                 }
                               });
          }));
}

bool WidgetBase::hasUrdfStateWatchers() const {
  return !urdf_state_callback_ids_.empty();
}

UrdfXacroFileWidget::UrdfXacroFileWidget() {
  root_widget_ = new QWidget();
  auto *layout = new QVBoxLayout(root_widget_);
  layout->setContentsMargins(6, 6, 6, 6);

  path_edit_ = new QLineEdit(root_widget_);
  layout->addWidget(path_edit_);

  auto *button_row = new QHBoxLayout();
  auto *browse = new QPushButton("Browse", root_widget_);
  auto *load = new QPushButton("Load", root_widget_);
  auto *save = new QPushButton("Save", root_widget_);
  button_row->addWidget(browse);
  button_row->addWidget(load);
  button_row->addStretch(1);
  button_row->addWidget(save);
  layout->addLayout(button_row);

  auto *alpha_row = new QHBoxLayout();
  mesh_alpha_label_ = new QLabel("Mesh Alpha 100%", root_widget_);
  mesh_alpha_slider_ = new QSlider(Qt::Horizontal, root_widget_);
  mesh_alpha_slider_->setRange(0, 100);
  mesh_alpha_slider_->setValue(100);
  alpha_row->addWidget(mesh_alpha_label_);
  alpha_row->addWidget(mesh_alpha_slider_, 1);
  layout->addLayout(alpha_row);

  QObject::connect(browse, &QPushButton::clicked, [this]() {
    const auto path = QFileDialog::getOpenFileName(
        root_widget_, "Open URDF/Xacro", QString(),
        "URDF and Xacro (*.urdf *.xacro *.xml);;All Files (*)");
    if (!path.isEmpty()) {
      path_edit_->setText(path);
    }
  });
  QObject::connect(load, &QPushButton::clicked, [this]() {
    UrdfEditorState::instance().setPublishTopic(publish_topic_);
    UrdfEditorState::instance().setAutoPublish(auto_publish_);
    if (UrdfEditorState::instance().loadFile(path_edit_->text().toStdString(),
                                             {})) {
      RobotStatePublisherRuntime::instance().apply(rsp_settings_, true);
    }
    syncFromState();
  });
  QObject::connect(save, &QPushButton::clicked, [this]() {
    UrdfEditorState::instance().save();
    syncFromState();
  });
  QObject::connect(mesh_alpha_slider_, &QSlider::valueChanged, [this](int value) {
    mesh_alpha_multiplier_ = std::clamp(value / 100.0, 0.0, 1.0);
    mesh_alpha_label_->setText(QString("Mesh Alpha %1%").arg(value));
    updateRobotModelAlpha();
    if (!mesh_alpha_slider_->isSliderDown()) {
      commitMeshAlpha();
    }
  });
  QObject::connect(mesh_alpha_slider_, &QSlider::sliderReleased,
                   [this]() { commitMeshAlpha(); });
}

void UrdfXacroFileWidget::setDisplayContext(
    rviz_common::DisplayContext *display_context) {
  WidgetBase::setDisplayContext(display_context);
  updateRobotModelAlpha();
}

void UrdfXacroFileWidget::updateRobotModelAlpha() {
  if (display_context_ == nullptr) {
    return;
  }
  setRobotModelAlpha(display_context_->getRootDisplayGroup(),
                     mesh_alpha_multiplier_);
}

void UrdfXacroFileWidget::commitMeshAlpha() {
  UrdfEditorState::instance().setMeshAlphaMultiplier(mesh_alpha_multiplier_);
}

void UrdfXacroFileWidget::configure(const YAML::Node &config) {
  const YAML::Node normalized = normalizeConfig(config);
  path_edit_->setText(QString::fromStdString(normalized["path"].as<std::string>()));
  publish_topic_ = normalized["robot_description_topic"].as<std::string>();
  rsp_settings_.node_name = normalized["rsp_node_name"].as<std::string>();
  rsp_settings_.node_namespace = normalized["rsp_namespace"].as<std::string>();
  rsp_settings_.robot_description_topic = publish_topic_;
  rsp_settings_.joint_states_topic =
      normalized["joint_states_topic"].as<std::string>();
  rsp_settings_.frame_prefix = normalized["frame_prefix"].as<std::string>();
  rsp_settings_.publish_frequency = normalized["publish_frequency"].as<double>();
  rsp_settings_.ignore_timestamp = normalized["ignore_timestamp"].as<bool>();
  UrdfEditorState::instance().setPublishTopic(publish_topic_);
  auto_publish_ = normalized["auto_publish"].as<bool>();
  UrdfEditorState::instance().setAutoPublish(auto_publish_);
  mesh_alpha_multiplier_ =
      std::clamp(normalized["mesh_alpha_multiplier"].as<double>(), 0.0, 1.0);
  mesh_alpha_slider_->setValue(static_cast<int>(std::round(mesh_alpha_multiplier_ * 100.0)));
  mesh_alpha_label_->setText(
      QString("Mesh Alpha %1%").arg(mesh_alpha_slider_->value()));
  updateRobotModelAlpha();
  commitMeshAlpha();
  if (!hasUrdfStateWatchers()) {
    watchUrdfState([this]() { syncFromState(); });
  }
  syncFromState();
}

std::string UrdfXacroFileWidget::type() const {
  return "rviz2_urdf_editor/UrdfXacroFileWidget";
}
std::string UrdfXacroFileWidget::displayName() const { return "URDF/Xacro File"; }
std::string UrdfXacroFileWidget::description() const {
  return "Loads, saves, expands, and publishes source-safe URDF/Xacro files.";
}
YAML::Node UrdfXacroFileWidget::getDefaultConfig() const {
  YAML::Node config = emptyConfig();
  config["path"] = "";
  config["robot_description_topic"] = "/robot_description";
  config["rsp_node_name"] = "dashboard_robot_state_publisher";
  config["rsp_namespace"] = "";
  config["joint_states_topic"] = "/joint_states";
  config["frame_prefix"] = "";
  config["publish_frequency"] = 20.0;
  config["ignore_timestamp"] = false;
  config["auto_publish"] = true;
  config["mesh_alpha_multiplier"] = 1.0;
  return config;
}
std::vector<ConfigField> UrdfXacroFileWidget::getConfigFields() const {
  return {makeFilePathField("path", "Path", false, "URDF or xacro source file."),
          makeTopicField("robot_description_topic", "Robot Description Topic", true,
                         "Topic used for std_msgs/String robot_description.",
                         "/robot_description"),
          makeStringField("rsp_node_name", "RSP Node Name", true, "", "",
                          "dashboard_robot_state_publisher"),
          makeStringField("rsp_namespace", "RSP Namespace"),
          makeTopicField("joint_states_topic", "Joint States Topic", true,
                         "Topic consumed by the internal robot_state_publisher.",
                         "/joint_states"),
          makeStringField("frame_prefix", "Frame Prefix"),
          makeDoubleField("publish_frequency", "Publish Frequency", 20.0, 0.1,
                          1000.0, 1.0, 1, true),
          makeDoubleField("mesh_alpha_multiplier", "Mesh Alpha Multiplier", 1.0,
                          0.0, 1.0, 0.05, 2, true),
          makeBoolField("ignore_timestamp", "Ignore Timestamp", "", false),
          makeBoolField("auto_publish", "Auto Publish",
                        "Publish after successful load or save.", false)};
}
WidgetSizeHint UrdfXacroFileWidget::getDefaultSizeHint() const {
  return {2, 4};
}
bool UrdfXacroFileWidget::validateConfig(const YAML::Node &config,
                                         std::string &error) const {
  const auto normalized = normalizeConfig(config);
  return requireTopic(normalized, "robot_description_topic", error) &&
         requireNonEmptyString(normalized, "rsp_node_name", error) &&
         requireTopic(normalized, "joint_states_topic", error) &&
         requireDoubleRange(normalized, "publish_frequency", 0.1, 1000.0,
                            error) &&
         requireDoubleRange(normalized, "mesh_alpha_multiplier", 0.0, 1.0,
                            error);
}

void UrdfXacroFileWidget::syncFromState() {
  const auto state = UrdfEditorState::instance().snapshot();
  if (!state.source_path.empty()) {
    path_edit_->setText(QString::fromStdString(state.source_path));
  }
}

UrdfDependencyTreeWidget::UrdfDependencyTreeWidget() {
  root_widget_ = new QWidget();
  auto *layout = new QVBoxLayout(root_widget_);
  layout->setContentsMargins(6, 6, 6, 6);
  tree_ = new QTreeWidget(root_widget_);
  tree_->setHeaderLabels({"File", "Sticky", "Kind", "Status", "Path/URL"});
  tree_->setContextMenuPolicy(Qt::CustomContextMenu);
  layout->addWidget(tree_);
  QObject::connect(tree_, &QTreeWidget::itemDoubleClicked,
                   [this](QTreeWidgetItem *item, int) {
    if (item != nullptr) {
      openFileEditor(item->data(0, Qt::UserRole).toString().toStdString());
    }
  });
  QObject::connect(tree_, &QTreeWidget::customContextMenuRequested,
                   [this](const QPoint &position) { openContextMenu(position); });
}
void UrdfDependencyTreeWidget::configure(const YAML::Node &) {
  if (!hasUrdfStateWatchers()) {
    watchUrdfState([this]() { refresh(); });
  }
  refresh();
}
std::string UrdfDependencyTreeWidget::type() const {
  return "rviz2_urdf_editor/UrdfDependencyTreeWidget";
}
std::string UrdfDependencyTreeWidget::displayName() const { return "URDF Dependencies"; }
std::string UrdfDependencyTreeWidget::description() const {
  return "Shows source URDF/xacro files and xacro includes.";
}
YAML::Node UrdfDependencyTreeWidget::getDefaultConfig() const { return emptyConfig(); }
std::vector<ConfigField> UrdfDependencyTreeWidget::getConfigFields() const {
  return {};
}
WidgetSizeHint UrdfDependencyTreeWidget::getDefaultSizeHint() const {
  return {3, 3};
}
void UrdfDependencyTreeWidget::refresh() {
  tree_->clear();
  for (const auto &dependency : UrdfEditorState::instance().snapshot().dependencies) {
    auto *item = new QTreeWidgetItem(tree_);
    item->setText(0, QString::fromStdString(
                         std::filesystem::path(dependency.path).filename().string()));
    item->setText(1, dependency.sticky ? "Unsaved" : "");
    item->setText(2, QString::fromStdString(dependency.kind));
    item->setText(3, dependency.exists ? "OK" : "Missing");
    item->setText(4, QString::fromStdString(dependency.path));
    item->setData(0, Qt::UserRole, QString::fromStdString(dependency.path));
  }
  tree_->resizeColumnToContents(0);
  tree_->resizeColumnToContents(1);
  tree_->resizeColumnToContents(2);
  tree_->resizeColumnToContents(3);
}

void UrdfDependencyTreeWidget::openFileEditor(const std::string &path) {
  if (!path.empty()) {
    UrdfEditorState::instance().openDependencyFileEditor(path);
  }
}

void UrdfDependencyTreeWidget::openContextMenu(const QPoint &position) {
  auto *item = tree_->itemAt(position);
  if (item == nullptr) {
    return;
  }
  QMenu menu(tree_);
  auto *edit = menu.addAction("Edit XML File");
  if (menu.exec(tree_->viewport()->mapToGlobal(position)) == edit) {
    openFileEditor(item->data(0, Qt::UserRole).toString().toStdString());
  }
}

UrdfXmlEditorWidget::UrdfXmlEditorWidget() {
  root_widget_ = new QWidget();
  auto *layout = new QVBoxLayout(root_widget_);
  layout->setContentsMargins(6, 6, 6, 6);

  title_edit_ = new QLineEdit(root_widget_);
  title_edit_->setReadOnly(true);
  layout->addWidget(title_edit_);

  editor_ = new SelectionMarkerPlainTextEdit(root_widget_);
  editor_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  xml_highlighter_ =
      new SharedXmlHighlighter(editor_->document(),
                               editor_->palette().color(QPalette::Base));
  layout->addWidget(editor_, 1);

  auto *button_row = new QHBoxLayout();
  undo_button_ = new QPushButton("Undo", root_widget_);
  redo_button_ = new QPushButton("Redo", root_widget_);
  line_wrap_check_ = new QCheckBox("Wrap", root_widget_);
  line_wrap_check_->setChecked(false);
  line_wrap_check_->setToolTip("Wrap long XML lines to the editor width.");
  auto *validate_button = new QPushButton("Validate", root_widget_);
  apply_button_ = new QPushButton("Apply", root_widget_);
  reload_button_ = new QPushButton("Reload", root_widget_);
  button_row->addWidget(undo_button_);
  button_row->addWidget(redo_button_);
  button_row->addWidget(line_wrap_check_);
  button_row->addWidget(validate_button);
  button_row->addStretch(1);
  button_row->addWidget(reload_button_);
  button_row->addWidget(apply_button_);
  layout->addLayout(button_row);

  status_label_ = new QLabel(root_widget_);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  QObject::connect(validate_button, &QPushButton::clicked,
                   [this]() { validateCurrentXml(); });
  QObject::connect(undo_button_, &QPushButton::clicked, editor_,
                   &QPlainTextEdit::undo);
  QObject::connect(redo_button_, &QPushButton::clicked, editor_,
                   &QPlainTextEdit::redo);
  QObject::connect(editor_, &QPlainTextEdit::undoAvailable, undo_button_,
                   &QPushButton::setEnabled);
  QObject::connect(editor_, &QPlainTextEdit::redoAvailable, redo_button_,
                   &QPushButton::setEnabled);
  QObject::connect(line_wrap_check_, &QCheckBox::toggled, [this](bool enabled) {
    editor_->setLineWrapMode(enabled ? QPlainTextEdit::WidgetWidth
                                     : QPlainTextEdit::NoWrap);
  });
  editor_->setLineWrapMode(QPlainTextEdit::NoWrap);
  QObject::connect(apply_button_, &QPushButton::clicked,
                   [this]() { applyCurrentXml(); });
  QObject::connect(reload_button_, &QPushButton::clicked,
                   [this]() { reloadCurrentXml(); });
  QObject::connect(editor_, &QPlainTextEdit::textChanged, [this]() {
    if (!updating_from_state_) {
      const auto xml = editor_->toPlainText().toStdString();
      if (xml == loaded_editor_text_) {
        return;
      }
      UrdfEditorState::instance().setXmlEditorDraft(xml);
      validateCurrentXml();
    }
  });
  undo_button_->setEnabled(false);
  redo_button_->setEnabled(false);
}

void UrdfXmlEditorWidget::configure(const YAML::Node &config) {
  (void)config;
  if (!hasUrdfStateWatchers()) {
    watchUrdfState([this]() { refresh(); });
  }
  refresh();
}

std::string UrdfXmlEditorWidget::type() const {
  return "rviz2_urdf_editor/UrdfXmlEditorWidget";
}
std::string UrdfXmlEditorWidget::displayName() const { return "URDF XML Editor"; }
std::string UrdfXmlEditorWidget::description() const {
  return "Edits XML selected from the URDF model, xacro macro, or dependency widgets.";
}
YAML::Node UrdfXmlEditorWidget::getDefaultConfig() const {
  return emptyConfig();
}
std::vector<ConfigField> UrdfXmlEditorWidget::getConfigFields() const {
  return {};
}
WidgetSizeHint UrdfXmlEditorWidget::getDefaultSizeHint() const {
  return {5, 5};
}

void UrdfXmlEditorWidget::refresh() {
  const auto document = UrdfEditorState::instance().xmlEditorDocument();
  if (document.revision == seen_document_revision_ &&
      !title_edit_->text().isEmpty()) {
    return;
  }
  if (auto *highlighter = dynamic_cast<SharedXmlHighlighter *>(xml_highlighter_)) {
    highlighter->setColorsForBackground(editor_->palette().color(QPalette::Base));
  }
  seen_document_revision_ = document.revision;
  updating_from_state_ = true;
  title_edit_->setText(QString::fromStdString(document.title));
  loaded_editor_text_ = document.xml;
  const auto editor_text = QString::fromStdString(loaded_editor_text_);
  {
    const QSignalBlocker editor_blocker(editor_);
    const QSignalBlocker document_blocker(editor_->document());
    editor_->setPlainText(editor_text);
  }
  editor_->document()->setModified(false);
  xml_highlighter_->rehighlight();
  if (auto *selection_editor =
          dynamic_cast<SelectionMarkerPlainTextEdit *>(editor_)) {
    selection_editor->resetFolds();
    selection_editor->setHighlightedRange(-1, -1);
  }
  if (document.has_selection && document.selection_end >= document.selection_begin &&
      document.selection_end <= loaded_editor_text_.size()) {
    const auto selection_start = QString::fromStdString(
                                     loaded_editor_text_.substr(
                                         0, document.selection_begin))
                                     .size();
    const auto selection_end =
        QString::fromStdString(
            loaded_editor_text_.substr(0, document.selection_end))
            .size();
    QTextCursor cursor(editor_->document());
    cursor.setPosition(selection_start);
    editor_->setTextCursor(cursor);
    editor_->centerCursor();

    if (auto *selection_editor =
            dynamic_cast<SelectionMarkerPlainTextEdit *>(editor_)) {
      selection_editor->setHighlightedRange(selection_start, selection_end);
    }
  }
  editor_->setReadOnly(!document.editable);
  apply_button_->setEnabled(document.editable);
  reload_button_->setEnabled(document.editable || !document.xml.empty());
  status_label_->setText(QString::fromStdString(document.status));
  updating_from_state_ = false;
  validateCurrentXml();

}

void UrdfXmlEditorWidget::validateCurrentXml() {
  const auto document = UrdfEditorState::instance().xmlEditorDocument();
  const auto xml = editor_->toPlainText().toStdString();
  pugi::xml_document doc;
  pugi::xml_parse_result result;
  if (document.whole_file) {
    result = doc.load_string(xml.c_str());
  } else {
    const auto wrapped =
        "<root xmlns:xacro=\"http://www.ros.org/wiki/xacro\">" + xml + "</root>";
    result = doc.load_string(wrapped.c_str(),
                             pugi::parse_default | pugi::parse_fragment);
  }
  if (!result) {
    status_label_->setText(QString("XML parse error: %1").arg(result.description()));
    apply_button_->setEnabled(false);
    return;
  }
  if (!document.whole_file) {
    pugi::xml_node element;
    for (auto child : doc.child("root").children()) {
      if (child.type() != pugi::node_element) {
        continue;
      }
      if (element) {
        status_label_->setText("XML must contain exactly one element.");
        apply_button_->setEnabled(false);
        return;
      }
      element = child;
    }
    const auto expected = document.kind == "macro" ? "xacro:macro" : document.kind;
    if (!element || std::string(element.name()) != expected) {
      status_label_->setText(QString::fromStdString("Element must remain " + expected + "."));
      apply_button_->setEnabled(false);
      return;
    }
    const auto name = element.attribute("name");
    if (!name || name.as_string() != document.name) {
      status_label_->setText(QString::fromStdString(
          "Element must keep name=\"" + document.name + "\"."));
      apply_button_->setEnabled(false);
      return;
    }
  }
  status_label_->setText(QString::fromStdString(compactEditorStatus(document)));
  apply_button_->setEnabled(document.editable);
}

void UrdfXmlEditorWidget::applyCurrentXml() {
  validateCurrentXml();
  if (!apply_button_->isEnabled()) {
    return;
  }
  const auto document = UrdfEditorState::instance().xmlEditorDocument();
  const auto formatted_xml = formatXmlWithTwoSpaceIndent(
      editor_->toPlainText().toStdString(), document.whole_file);
  if (formatted_xml != editor_->toPlainText().toStdString()) {
    const QSignalBlocker editor_blocker(editor_);
    const QSignalBlocker document_blocker(editor_->document());
    editor_->setPlainText(QString::fromStdString(formatted_xml));
    xml_highlighter_->rehighlight();
  }
  if (!UrdfEditorState::instance().applyXmlEditorDocument(
          formatted_xml)) {
    status_label_->setText(QString::fromStdString(
        UrdfEditorState::instance().snapshot().last_error));
  }
}

void UrdfXmlEditorWidget::reloadCurrentXml() {
  UrdfEditorState::instance().reloadXmlEditorDocument();
}

UrdfModelTreeWidget::UrdfModelTreeWidget() {
  root_widget_ = new QWidget();
  auto *layout = new QVBoxLayout(root_widget_);
  layout->setContentsMargins(6, 6, 6, 6);
  select_all_geometry_check_ = new QCheckBox("Visuals", root_widget_);
  select_all_geometry_check_->setTristate(true);
  select_all_geometry_check_->setToolTip(
      "Show or hide all link visuals and collisions.");
  layout->addWidget(select_all_geometry_check_);
  auto *frame_tree = new FrameTreeWidget(root_widget_);
  tree_ = frame_tree;
  tree_->setColumnCount(2);
  tree_->setHeaderLabels({"Frame", ""});
  tree_->header()->setStretchLastSection(false);
  tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  tree_->header()->setSectionResizeMode(1, QHeaderView::Fixed);
  tree_->header()->resizeSection(1, 24);
  tree_->setColumnWidth(1, 24);
  tree_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  tree_->setContextMenuPolicy(Qt::CustomContextMenu);
  layout->addWidget(tree_);
  const auto open_item = [this](QTreeWidgetItem *item) {
    if (item == nullptr) {
      return;
    }
    openLinkEditor(item->data(0, Qt::UserRole).toString().toStdString());
  };
  frame_tree->setFrameActivatedCallback(open_item);
  QObject::connect(tree_, &QTreeWidget::itemClicked,
                   [this](QTreeWidgetItem *item, int) {
    if (item == nullptr) {
      return;
    }
    const auto link_name = item->data(0, Qt::UserRole).toString().toStdString();
    UrdfEditorState::instance().setSelectedLink(link_name);
    openLinkEditor(link_name);
  });
  QObject::connect(tree_, &QTreeWidget::itemDoubleClicked,
                   [open_item](QTreeWidgetItem *item, int) { open_item(item); });
  QObject::connect(tree_, &QTreeWidget::itemActivated,
                   [open_item](QTreeWidgetItem *item, int) { open_item(item); });
  QObject::connect(tree_, &QTreeWidget::itemChanged,
                   [this](QTreeWidgetItem *item, int column) {
    if (item == nullptr || column != 1) {
      return;
    }
    UrdfEditorState::instance().setLinkGeometryVisible(
        item->data(0, Qt::UserRole).toString().toStdString(),
        item->checkState(1) == Qt::Checked);
  });
  QObject::connect(select_all_geometry_check_, &QCheckBox::stateChanged,
                   [](int state) {
    if (state == Qt::PartiallyChecked) {
      return;
    }
    UrdfEditorState::instance().setAllLinkGeometryVisible(state == Qt::Checked);
  });
  QObject::connect(tree_, &QTreeWidget::customContextMenuRequested,
                   [this](const QPoint &position) { openContextMenu(position); });
}
void UrdfModelTreeWidget::configure(const YAML::Node &config) {
  const auto normalized = normalizeConfig(config);
  highlight_color_ =
      QColor(QString::fromStdString(normalized["highlight_color"].as<std::string>()));
  if (!highlight_color_.isValid()) {
    highlight_color_ = QColor("#ffd10d");
  }
  UrdfEditorState::instance().setHighlightColor(
      highlight_color_.name(QColor::HexRgb).toStdString());
  if (!hasUrdfStateWatchers()) {
    watchUrdfState([this]() { refresh(); });
  }
  refresh();
}
std::string UrdfModelTreeWidget::type() const {
  return "rviz2_urdf_editor/UrdfModelTreeWidget";
}
std::string UrdfModelTreeWidget::displayName() const { return "URDF Model Tree"; }
std::string UrdfModelTreeWidget::description() const {
  return "Displays links and opens source XML editors for links and joints.";
}
YAML::Node UrdfModelTreeWidget::getDefaultConfig() const {
  YAML::Node config = emptyConfig();
  config["highlight_color"] = "#ffd10d";
  return config;
}
std::vector<ConfigField> UrdfModelTreeWidget::getConfigFields() const {
  return {makeColorField("highlight_color", "Highlight Color", "#ffd10d", true,
                         "Color used for selected-frame highlighting.")};
}
WidgetSizeHint UrdfModelTreeWidget::getDefaultSizeHint() const {
  return {4, 3};
}
void UrdfModelTreeWidget::refresh() {
  const QSignalBlocker blocker(tree_);
  const QSignalBlocker select_all_blocker(select_all_geometry_check_);
  const auto scroll_position = tree_->verticalScrollBar()->value();
  QTreeWidgetItem *selected_item = nullptr;
  tree_->clear();
  const auto snapshot = UrdfEditorState::instance().snapshot();
  const auto model = snapshot.model;
  select_all_geometry_check_->setEnabled(!model.links.empty());
  if (model.links.empty() || snapshot.hidden_geometry_links.empty()) {
    select_all_geometry_check_->setCheckState(Qt::Checked);
  } else if (snapshot.hidden_geometry_links.size() >= model.links.size()) {
    select_all_geometry_check_->setCheckState(Qt::Unchecked);
  } else {
    select_all_geometry_check_->setCheckState(Qt::PartiallyChecked);
  }
  std::map<std::string, std::vector<UrdfJointSummary>> children_by_parent;
  std::set<std::string> child_links;
  for (const auto &joint : model.joints) {
    children_by_parent[joint.parent_link].push_back(joint);
    child_links.insert(joint.child_link);
  }

  const auto add_link_item = [&](auto &&self, QTreeWidgetItem *parent,
                                const std::string &link_name,
                                const std::string &joint_name,
                                const std::string &) -> void {
    auto *item = parent == nullptr ? new QTreeWidgetItem(tree_)
                                   : new QTreeWidgetItem(parent);
    item->setText(0, QString::fromStdString(link_name));
    item->setData(0, Qt::UserRole, QString::fromStdString(link_name));
    item->setData(0, Qt::UserRole + 1, QString::fromStdString(joint_name));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(1, snapshot.hidden_geometry_links.count(link_name) == 0
                               ? Qt::Checked
                               : Qt::Unchecked);
    if (!snapshot.selected_link.empty() && snapshot.selected_link == link_name) {
      selected_item = item;
    }
    for (const auto &joint : children_by_parent[link_name]) {
      self(self, item, joint.child_link, joint.name, joint.type);
    }
  };

  std::vector<std::string> roots;
  for (const auto &link : model.links) {
    if (child_links.count(link) == 0) {
      roots.push_back(link);
    }
  }
  if (roots.empty()) {
    roots = model.links;
  }
  for (const auto &root : roots) {
    add_link_item(add_link_item, nullptr, root, "", "");
  }
  tree_->expandAll();
  if (selected_item != nullptr) {
    tree_->setCurrentItem(selected_item, 0, QItemSelectionModel::ClearAndSelect);
  }
  tree_->verticalScrollBar()->setValue(scroll_position);
  QTimer::singleShot(0, tree_, [tree = QPointer<QTreeWidget>(tree_),
                               scroll_position]() {
    if (tree != nullptr) {
      tree->verticalScrollBar()->setValue(scroll_position);
    }
  });
}

void UrdfModelTreeWidget::openLinkEditor(const std::string &link_name) {
  if (link_name.empty()) {
    return;
  }
  UrdfEditorState::instance().openSourceElementEditor("link", link_name,
                                                       "Edit Link XML: " + link_name);
}

void UrdfModelTreeWidget::openIncomingJointEditor(const std::string &link_name) {
  const auto snapshot = UrdfEditorState::instance().snapshot();
  for (const auto &joint : snapshot.model.joints) {
    if (joint.child_link == link_name) {
      UrdfEditorState::instance().openSourceElementEditor(
          "joint", joint.name, "Edit Incoming Joint XML: " + joint.name);
      return;
    }
  }
}

void UrdfModelTreeWidget::openContextMenu(const QPoint &position) {
  auto *item = tree_->itemAt(position);
  if (item == nullptr) {
    return;
  }
  const auto link_name = item->data(0, Qt::UserRole).toString().toStdString();
  const auto joint_name =
      item->data(0, Qt::UserRole + 1).toString().toStdString();
  QMenu menu(tree_);
  auto *edit_link = menu.addAction("Edit Link XML");
  QAction *edit_joint = nullptr;
  if (!joint_name.empty()) {
    edit_joint = menu.addAction("Edit Incoming Joint XML");
  }
  QAction *edit_parent = nullptr;
  const auto snapshot = UrdfEditorState::instance().snapshot();
  const auto joint =
      std::find_if(snapshot.model.joints.begin(), snapshot.model.joints.end(),
                   [&](const UrdfJointSummary &candidate) {
                     return candidate.name == joint_name;
                   });
  if (joint != snapshot.model.joints.end() && !joint->parent_link.empty()) {
    edit_parent = menu.addAction("Edit Parent Link XML");
  }
  const auto *selected = menu.exec(tree_->viewport()->mapToGlobal(position));
  if (selected == edit_link) {
    openLinkEditor(link_name);
  } else if (selected == edit_joint) {
    openIncomingJointEditor(link_name);
  } else if (selected == edit_parent && joint != snapshot.model.joints.end()) {
    openLinkEditor(joint->parent_link);
  }
}

UrdfXacroMacroWidget::UrdfXacroMacroWidget() {
  root_widget_ = new QWidget();
  auto *layout = new QVBoxLayout(root_widget_);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(3);

  const auto add_tree = [&](const QString &title) {
    auto *label = new QLabel(title, root_widget_);
    label->setContentsMargins(0, 0, 0, 0);
    auto *tree = new QTreeWidget(root_widget_);
    tree->setColumnCount(1);
    tree->setHeaderHidden(true);
    tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree->setSelectionMode(QAbstractItemView::SingleSelection);
    tree->setContextMenuPolicy(Qt::CustomContextMenu);
    layout->addWidget(label);
    layout->addWidget(tree);
    QObject::connect(tree, &QTreeWidget::itemDoubleClicked,
                     [this](QTreeWidgetItem *item, int) {
                       openElementEditor(item);
                     });
    QObject::connect(tree, &QTreeWidget::customContextMenuRequested,
                     [this, tree](const QPoint &position) {
                       openContextMenu(tree, position);
                     });
    return tree;
  };

  macro_tree_ = add_tree("Macros");
  arg_tree_ = add_tree("Args");
  property_tree_ = add_tree("Properties");
}

void UrdfXacroMacroWidget::configure(const YAML::Node &config) {
  (void)config;
  if (!hasUrdfStateWatchers()) {
    watchUrdfState([this]() { refresh(); });
  }
  refresh();
}

std::string UrdfXacroMacroWidget::type() const {
  return "rviz2_urdf_editor/UrdfXacroMacroWidget";
}
std::string UrdfXacroMacroWidget::displayName() const { return "Xacro Macros"; }
std::string UrdfXacroMacroWidget::description() const {
  return "Lists source xacro macros, properties, and args.";
}
YAML::Node UrdfXacroMacroWidget::getDefaultConfig() const {
  return emptyConfig();
}
std::vector<ConfigField>
UrdfXacroMacroWidget::getConfigFields() const {
  return {};
}
WidgetSizeHint UrdfXacroMacroWidget::getDefaultSizeHint() const {
  return {3, 3};
}

void UrdfXacroMacroWidget::refresh() {
  macro_tree_->clear();
  arg_tree_->clear();
  property_tree_->clear();
  std::map<std::pair<QTreeWidget *, std::string>, QTreeWidgetItem *> file_items;
  for (const auto &element : UrdfEditorState::instance().sourceElements()) {
    QTreeWidget *tree = nullptr;
    if (element.kind == "macro") {
      tree = macro_tree_;
    } else if (element.kind == "arg") {
      tree = arg_tree_;
    } else if (element.kind == "property") {
      tree = property_tree_;
    } else {
      continue;
    }
    const auto key = std::make_pair(tree, element.source_path);
    auto file_item = file_items.find(key);
    if (file_item == file_items.end()) {
      auto *parent = new QTreeWidgetItem(tree);
      parent->setText(
          0, QString::fromStdString(
                 std::filesystem::path(element.source_path).filename().string()));
      parent->setData(0, Qt::UserRole + 2,
                      QString::fromStdString(element.source_path));
      parent->setExpanded(true);
      file_item = file_items.emplace(key, parent).first;
    }

    auto *item = new QTreeWidgetItem(file_item->second);
    item->setText(0, QString::fromStdString(element.name));
    item->setData(0, Qt::UserRole, QString::fromStdString(element.kind));
    item->setData(0, Qt::UserRole + 1, QString::fromStdString(element.name));
    item->setData(0, Qt::UserRole + 2, QString::fromStdString(element.source_path));
  }
  macro_tree_->expandAll();
  arg_tree_->expandAll();
  property_tree_->expandAll();
}

void UrdfXacroMacroWidget::openElementEditor(QTreeWidgetItem *item) {
  if (item == nullptr) {
    return;
  }
  const auto kind = item->data(0, Qt::UserRole).toString().toStdString();
  const auto name = item->data(0, Qt::UserRole + 1).toString().toStdString();
  const auto source_path =
      item->data(0, Qt::UserRole + 2).toString().toStdString();
  if (kind.empty() || name.empty()) {
    return;
  }
  UrdfEditorState::instance().openSourceElementEditor(
      kind, name, "Edit " + kind + " XML: " + name, source_path);
}

void UrdfXacroMacroWidget::openContextMenu(QTreeWidget *tree,
                                           const QPoint &position) {
  if (tree == nullptr) {
    return;
  }
  auto *item = tree->itemAt(position);
  if (item == nullptr) {
    return;
  }
  QMenu menu(tree);
  auto *edit = menu.addAction("Edit XML");
  if (menu.exec(tree->viewport()->mapToGlobal(position)) == edit) {
    openElementEditor(item);
  }
}

UrdfMeshListWidget::UrdfMeshListWidget() {
  root_widget_ = new QWidget();
  auto *layout = new QVBoxLayout(root_widget_);
  layout->setContentsMargins(6, 6, 6, 6);
  table_ = new QTableWidget(0, 4, root_widget_);
  table_->setHorizontalHeaderLabels({"Filename", "Role", "URI", "Owner"});
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
  table_->setColumnWidth(0, 220);
  layout->addWidget(table_);
  QObject::connect(table_, &QTableWidget::cellClicked,
                   [this](const int row, const int) {
    const auto meshes = UrdfEditorState::instance().snapshot().model.meshes;
    if (row >= 0 && row < static_cast<int>(meshes.size())) {
      UrdfEditorState::instance().toggleSelectedMesh(meshes.at(row));
      refresh();
    }
  });
}
void UrdfMeshListWidget::configure(const YAML::Node &) {
  if (!hasUrdfStateWatchers()) {
    watchUrdfState([this]() { refresh(); });
  }
  refresh();
}
std::string UrdfMeshListWidget::type() const {
  return "rviz2_urdf_editor/UrdfMeshListWidget";
}
std::string UrdfMeshListWidget::displayName() const { return "URDF Mesh List"; }
std::string UrdfMeshListWidget::description() const {
  return "Lists visual and collision mesh references with resolved paths.";
}
YAML::Node UrdfMeshListWidget::getDefaultConfig() const { return emptyConfig(); }
std::vector<ConfigField> UrdfMeshListWidget::getConfigFields() const {
  return {};
}
WidgetSizeHint UrdfMeshListWidget::getDefaultSizeHint() const {
  return {4, 4};
}
void UrdfMeshListWidget::refresh() {
  const auto snapshot = UrdfEditorState::instance().snapshot();
  const auto meshes = snapshot.model.meshes;
  table_->setRowCount(static_cast<int>(meshes.size()));
  for (int row = 0; row < static_cast<int>(meshes.size()); ++row) {
    const auto &mesh = meshes.at(row);
    const auto filename =
        std::filesystem::path(mesh.resolved_path.empty() ? mesh.uri
                                                         : mesh.resolved_path)
            .filename()
            .string();
    table_->setItem(row, 0,
                    readOnlyTableItem(QString::fromStdString(filename)));
    table_->setItem(row, 1,
                    readOnlyTableItem(QString::fromStdString(mesh.role)));
    table_->setItem(row, 2,
                    readOnlyTableItem(QString::fromStdString(mesh.uri)));
    table_->setItem(row, 3,
                    readOnlyTableItem(QString::fromStdString(mesh.owner)));
    const bool selected = snapshot.selected_mesh.uri == mesh.uri &&
                          snapshot.selected_mesh.owner == mesh.owner &&
                          snapshot.selected_mesh.role == mesh.role;
    if (selected) {
      for (int column = 0; column < table_->columnCount(); ++column) {
        table_->item(row, column)->setBackground(QBrush(QColor("#ffea80")));
      }
      table_->selectRow(row);
    }
  }
  table_->resizeColumnToContents(0);
  if (table_->columnWidth(0) < 220) {
    table_->setColumnWidth(0, 220);
  }
}

JointStatePublisherWidget::JointStatePublisherWidget() {
  root_widget_ = new QWidget();
  auto *layout = new QVBoxLayout(root_widget_);
  layout->setContentsMargins(6, 6, 6, 6);
  auto *top = new QHBoxLayout();
  auto_publish_check_ = new QCheckBox("Auto", root_widget_);
  auto *publish_button = new QPushButton("Publish", root_widget_);
  top->addStretch(1);
  top->addWidget(auto_publish_check_);
  top->addWidget(publish_button);
  layout->addLayout(top);

  auto *scroll = new QScrollArea(root_widget_);
  scroll->setWidgetResizable(true);
  slider_container_ = new QWidget(scroll);
  slider_container_->setLayout(new QVBoxLayout());
  scroll->setWidget(slider_container_);
  layout->addWidget(scroll, 1);
  status_label_ = new QLabel(root_widget_);
  layout->addWidget(status_label_);

  QObject::connect(publish_button, &QPushButton::clicked, [this]() { publish(); });

  state_timer_ = new QTimer(root_widget_);
  state_timer_->setInterval(250);
  QObject::connect(state_timer_, &QTimer::timeout, [this]() {
    syncModelFromState();
    if (auto_publish_check_->isChecked() && !joint_controls_.empty()) {
      publish();
    }
  });
  state_timer_->start();
}
void JointStatePublisherWidget::initialize(const rclcpp::Node::SharedPtr &node,
                                           const std::string &widget_id) {
  WidgetBase::initialize(node, widget_id);
}
void JointStatePublisherWidget::configure(const YAML::Node &config) {
  const auto normalized = normalizeConfig(config);
  topic_ = normalized["topic"].as<std::string>();
  auto_publish_ = normalized["auto_publish"].as<bool>();
  auto_publish_check_->setChecked(auto_publish_);
  syncModelFromState();
}
std::string JointStatePublisherWidget::type() const {
  return "rviz2_urdf_editor/JointStatePublisherWidget";
}
std::string JointStatePublisherWidget::displayName() const {
  return "Joint State Publisher";
}
std::string JointStatePublisherWidget::description() const {
  return "Publishes slider-controlled joint states for the loaded URDF.";
}
YAML::Node JointStatePublisherWidget::getDefaultConfig() const {
  YAML::Node config = emptyConfig();
  config["topic"] = "/joint_states";
  config["auto_publish"] = true;
  return config;
}
std::vector<ConfigField> JointStatePublisherWidget::getConfigFields() const {
  return {makeTopicField("topic", "Joint States Topic", true, "", "/joint_states"),
          makeBoolField("auto_publish", "Auto Publish", "", true)};
}
WidgetSizeHint JointStatePublisherWidget::getDefaultSizeHint() const {
  return {4, 3};
}
bool JointStatePublisherWidget::validateConfig(const YAML::Node &config,
                                               std::string &error) const {
  return requireTopic(normalizeConfig(config), "topic", error);
}
void JointStatePublisherWidget::rebuildSliders() {
  joint_controls_.clear();
  clearLayout(slider_container_->layout());
  const auto snapshot = UrdfEditorState::instance().snapshot();
  seen_state_revision_ = snapshot.revision;
  const auto joints = snapshot.model.joints;
  for (const auto &joint : joints) {
    if (!isMovableJoint(joint)) {
      continue;
    }
    auto *row_widget = new QWidget(slider_container_);
    auto *row = new QHBoxLayout(row_widget);
    auto *name = new QLabel(QString::fromStdString(joint.name), row_widget);
    row->addWidget(name, 1);

    if (hasUsableSliderLimits(joint)) {
      auto *slider = new QSlider(Qt::Horizontal, row_widget);
      auto *value = new QLabel("0.000", row_widget);
      slider->setRange(0, 1000);
      slider->setValue(500);
      row->addWidget(slider, 2);
      row->addWidget(value);
      slider_container_->layout()->addWidget(row_widget);
      joint_controls_.push_back({joint, slider, nullptr, value});
      QObject::connect(slider, &QSlider::valueChanged, [this]() {
        for (auto &row : joint_controls_) {
          if (row.value_label != nullptr) {
            row.value_label->setText(QString::number(jointValue(row), 'f', 3));
          }
        }
        if (auto_publish_check_->isChecked()) {
          publish();
        }
      });
    } else {
      auto *spin_box = new QDoubleSpinBox(row_widget);
      spin_box->setRange(-1000.0, 1000.0);
      spin_box->setDecimals(4);
      spin_box->setSingleStep(spinBoxStep(joint));
      spin_box->setValue(0.0);
      row->addWidget(spin_box, 2);
      slider_container_->layout()->addWidget(row_widget);
      joint_controls_.push_back({joint, nullptr, spin_box, nullptr});
      QObject::connect(spin_box, qOverload<double>(&QDoubleSpinBox::valueChanged),
                       [this](double) {
        if (auto_publish_check_->isChecked()) {
          publish();
        }
      });
    }
  }
  slider_container_->layout()->addItem(new QSpacerItem(
      1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  status_label_->setText(
      QString("%1 movable joints").arg(joint_controls_.size()));
  for (auto &row : joint_controls_) {
    if (row.value_label != nullptr) {
      row.value_label->setText(QString::number(jointValue(row), 'f', 3));
    }
  }
}
void JointStatePublisherWidget::syncModelFromState() {
  const auto snapshot = UrdfEditorState::instance().snapshot();
  if (snapshot.revision == seen_state_revision_) {
    return;
  }
  rebuildSliders();
}
void JointStatePublisherWidget::publish() {
  if (!node_) {
    status_label_->setText("No ROS node available.");
    return;
  }
  const auto topic = topic_;
  if (!publisher_ || publisher_topic_ != topic) {
    publisher_topic_ = topic;
    publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        publisher_topic_, rclcpp::QoS(10));
  }
  sensor_msgs::msg::JointState message;
  message.header.stamp = node_->now();
  for (const auto &row : joint_controls_) {
    message.name.push_back(row.joint.name);
    message.position.push_back(jointValue(row));
  }
  publisher_->publish(message);
  status_label_->setText(QString("Published %1 joints on %2")
                             .arg(message.name.size())
                             .arg(QString::fromStdString(topic)));
}
double JointStatePublisherWidget::jointValue(const JointControlRow &row) const {
  if (row.spin_box != nullptr) {
    return row.spin_box->value();
  }
  if (row.slider == nullptr) {
    return 0.0;
  }
  const double t =
      static_cast<double>(row.slider->value() - row.slider->minimum()) /
      static_cast<double>(row.slider->maximum() - row.slider->minimum());
  return row.joint.lower + (row.joint.upper - row.joint.lower) * t;
}

UrdfEditorPanel::UrdfEditorPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  file_widget_.widget()->setObjectName("urdf_file_widget");
  xml_editor_widget_.widget()->setObjectName("urdf_xml_editor_widget");
  file_widget_.configure(file_widget_.getDefaultConfig());
  xml_editor_widget_.configure(xml_editor_widget_.getDefaultConfig());
  layout->addWidget(file_widget_.widget());
  layout->addWidget(xml_editor_widget_.widget(), 1);
}

void UrdfEditorPanel::onInitialize() {
  rclcpp::Node::SharedPtr node;
  auto *display_context = getDisplayContext();
  if (display_context != nullptr) {
    if (auto rviz_node = display_context->getRosNodeAbstraction().lock()) {
      node = rviz_node->get_raw_node();
    }
  }
  initializeWidgets(node, display_context);
}

void UrdfEditorPanel::initializeWidgets(
    const rclcpp::Node::SharedPtr &node,
    rviz_common::DisplayContext *display_context) {
  file_widget_.initialize(node, "urdf_file_widget");
  file_widget_.setDisplayContext(display_context);
  file_widget_.configure(file_widget_.getDefaultConfig());
  xml_editor_widget_.initialize(node, "urdf_xml_editor_widget");
  xml_editor_widget_.configure(xml_editor_widget_.getDefaultConfig());
}

UrdfNavPanel::UrdfNavPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  tabs_ = new QTabWidget(this);
  tabs_->setObjectName("urdf_nav_tabs");
  model_tree_widget_.widget()->setObjectName("urdf_frames_widget");
  dependency_tree_widget_.widget()->setObjectName("urdf_files_widget");
  joint_state_widget_.widget()->setObjectName("urdf_joints_widget");
  xacro_macro_widget_.widget()->setObjectName("urdf_xacro_widget");
  mesh_list_widget_.widget()->setObjectName("urdf_meshes_widget");
  model_tree_widget_.configure(model_tree_widget_.getDefaultConfig());
  dependency_tree_widget_.configure(dependency_tree_widget_.getDefaultConfig());
  joint_state_widget_.configure(joint_state_widget_.getDefaultConfig());
  xacro_macro_widget_.configure(xacro_macro_widget_.getDefaultConfig());
  mesh_list_widget_.configure(mesh_list_widget_.getDefaultConfig());
  tabs_->addTab(model_tree_widget_.widget(), "Frames");
  tabs_->addTab(dependency_tree_widget_.widget(), "Files");
  tabs_->addTab(joint_state_widget_.widget(), "Joints");
  tabs_->addTab(xacro_macro_widget_.widget(), "Xacro");
  tabs_->addTab(mesh_list_widget_.widget(), "Meshes");
  layout->addWidget(tabs_);
}

void UrdfNavPanel::onInitialize() {
  rclcpp::Node::SharedPtr node;
  auto *display_context = getDisplayContext();
  if (display_context != nullptr) {
    if (auto rviz_node = display_context->getRosNodeAbstraction().lock()) {
      node = rviz_node->get_raw_node();
    }
  }
  initializeWidgets(node, display_context);
}

void UrdfNavPanel::initializeWidgets(
    const rclcpp::Node::SharedPtr &node,
    rviz_common::DisplayContext *display_context) {
  model_tree_widget_.initialize(node, "urdf_frames_widget");
  model_tree_widget_.configure(model_tree_widget_.getDefaultConfig());
  dependency_tree_widget_.initialize(node, "urdf_files_widget");
  dependency_tree_widget_.configure(dependency_tree_widget_.getDefaultConfig());
  joint_state_widget_.initialize(node, "urdf_joints_widget");
  joint_state_widget_.configure(joint_state_widget_.getDefaultConfig());
  xacro_macro_widget_.initialize(node, "urdf_xacro_widget");
  xacro_macro_widget_.configure(xacro_macro_widget_.getDefaultConfig());
  mesh_list_widget_.initialize(node, "urdf_meshes_widget");
  mesh_list_widget_.configure(mesh_list_widget_.getDefaultConfig());
  (void)display_context;
}

} // namespace rviz2_urdf_editor

PLUGINLIB_EXPORT_CLASS(rviz2_urdf_editor::UrdfEditorPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(rviz2_urdf_editor::UrdfNavPanel, rviz_common::Panel)
