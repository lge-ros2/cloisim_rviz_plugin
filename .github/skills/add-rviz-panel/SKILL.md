---
name: add-rviz-panel
description: "Add a new RViz panel to cloisim_rviz_plugin. Use when creating a new operator panel, adding panel-local ROS topic wiring, or registering an additional pluginlib panel class."
---

# Add a New RViz Panel

Use this workflow when adding a new panel to `cloisim_rviz_plugin`.

## When to Use

- Creating a new RViz dockable panel
- Adding a new panel class with Qt widgets
- Wiring a panel to ROS 2 publishers/subscribers/services
- Registering another `pluginlib` panel entry in RViz

## Expected Package Shape

A new panel usually touches all of these:

- `include/panels/<panel_name>.hpp`
- `src/panels/<panel_name>.cpp`
- `plugin_description.xml`
- `CMakeLists.txt`
- optionally `README.md`

## Required Architecture

Each panel should follow this shape:

1. Class derives from `rviz_common::Panel`
2. Header declares `Q_OBJECT`
3. Constructor builds or triggers UI setup
4. `onInitialize()` acquires the RViz ROS node and creates ROS interfaces
5. Source exports the class with `PLUGINLIB_EXPORT_CLASS(..., rviz_common::Panel)`

## Minimal Header Pattern

```cpp
#ifndef CLOISIM_RVIZ_PLUGIN__PANELS__EXAMPLE_PANEL_HPP_
#define CLOISIM_RVIZ_PLUGIN__PANELS__EXAMPLE_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class QPushButton;
class QLineEdit;

namespace cloisim_rviz_plugin
{
class ExamplePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ExamplePanel(QWidget * parent = nullptr);
  ~ExamplePanel() override;

protected:
  void onInitialize() override;

private:
  void initializeLayout();
  void handleSend();

private:
  QLineEdit * input_edit_;
  QPushButton * send_button_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}  // namespace cloisim_rviz_plugin

#endif
```

## Minimal Source Pattern

```cpp
#include "panels/example_panel.hpp"

#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <std_msgs/msg/string.hpp>

namespace cloisim_rviz_plugin
{
ExamplePanel::ExamplePanel(QWidget * parent)
: rviz_common::Panel(parent),
  input_edit_(nullptr),
  send_button_(nullptr)
{
  initializeLayout();
}

ExamplePanel::~ExamplePanel() = default;

void ExamplePanel::onInitialize()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = raw_node->create_publisher<std_msgs::msg::String>("example_topic", 10);
}

void ExamplePanel::initializeLayout()
{
  input_edit_ = new QLineEdit();
  send_button_ = new QPushButton(tr("Send"));
  connect(send_button_, &QPushButton::released, this, &ExamplePanel::handleSend);

  auto * layout = new QVBoxLayout();
  layout->addWidget(input_edit_);
  layout->addWidget(send_button_);
  setLayout(layout);
}

void ExamplePanel::handleSend()
{
  if (!publisher_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = input_edit_->text().toStdString();
  publisher_->publish(msg);
}
}  // namespace cloisim_rviz_plugin

PLUGINLIB_EXPORT_CLASS(cloisim_rviz_plugin::ExamplePanel, rviz_common::Panel)
```

## Required Integration Steps

### 1. Add the new files

- Put the public declaration in `include/panels/`
- Put the implementation in `src/panels/`

### 2. Update `CMakeLists.txt`

If the header uses `Q_OBJECT`, add it to the MOC header list.

Also add the new source file to the library source list.

### 3. Update `plugin_description.xml`

Register the new class with:
- library path
- fully qualified type
- base class `rviz_common::Panel`
- human-readable description

### 4. Keep install/export behavior intact

Do not break the existing shared library export and include installation rules.

## Design Guidance

- Keep one panel focused on one operator workflow.
- Prefer explicit topic fields when topic names may differ per robot.
- Validate user input before publishing commands.
- Use ROS logging for runtime failures and Qt feedback only for operator-facing validation.
- Be cautious with background threads in panels; use them only when a timer or direct callback model is insufficient.

## Validation Checklist

- [ ] Panel class derives from `rviz_common::Panel`
- [ ] Header uses `Q_OBJECT`
- [ ] `onInitialize()` creates required ROS interfaces
- [ ] `CMakeLists.txt` includes the header in MOC input if needed
- [ ] `CMakeLists.txt` includes the new `.cpp`
- [ ] `plugin_description.xml` registers the panel
- [ ] Plugin still builds as a shared library
- [ ] RViz can discover and load the panel

## Common Failure Modes

- Missing MOC registration for a `Q_OBJECT` header
- Panel compiled but not registered in `plugin_description.xml`
- Wrong exported class namespace in `PLUGINLIB_EXPORT_CLASS`
- ROS interfaces created before the RViz display context is ready
- Hidden ownership bugs from manually deleting Qt child widgets
