#include "panels/jog_control_panel.hpp"
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QScrollArea>
#include <QSlider>
#include <QDoubleValidator>
#include <tinyxml2.h>
#include <rviz_common/display_context.hpp>

#define INFO_STREAM RCLCPP_INFO_STREAM
#define ERROR_STREAM RCLCPP_ERROR_STREAM

using namespace cloisim_rviz_plugin;
using std::cout;
using std::endl;
using std::string;

// namespace fs = std::filesystem;

JogControlPanel::JogControlPanel(QWidget *parent)
    : rviz_common::Panel(parent),
      //  joint_state_form_(nullptr),
      grid_(nullptr),
      namespace_topic_edit_(nullptr),
      robot_desc_topic_edit_(nullptr),
      joint_state_topic_edit_(nullptr),
      joint_command_topic_edit_(nullptr)
{
  initializeLayout();
}

JogControlPanel::~JogControlPanel()
{
  delete grid_;
  grid_ = nullptr;

  delete namespace_topic_edit_;
  namespace_topic_edit_ = nullptr;

  delete robot_desc_topic_edit_;
  robot_desc_topic_edit_ = nullptr;

  delete joint_state_topic_edit_;
  joint_state_topic_edit_ = nullptr;

  delete joint_command_topic_edit_;
  joint_command_topic_edit_ = nullptr;
}

void JogControlPanel::onInitialize()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  namespace_topic_edit_->setText(tr(raw_node->get_namespace()));

  resetSubscriptionJointStates();

  resetPublisherJointCommand();

  resetSubscriptionRobotDesc();
}

void JogControlPanel::resetSubscriptionJointStates()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // for (auto i = 0; i < joint_state_form_->rowCount(); i++)
  //   joint_state_form_->removeRow(i);

  QLayoutItem *item = nullptr;
  while ((item = grid_->takeAt(4)) != nullptr)
    delete item;

  sub_joint_states_.reset();
  sub_joint_states_ = raw_node->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_edit_->text().toStdString(),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)
      {
        handleJointStates(msg);
      });
}

void JogControlPanel::resetPublisherJointCommand()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_joint_jog_.reset();
  pub_joint_jog_ = raw_node->create_publisher<control_msgs::msg::JointJog>(
      joint_command_topic_edit_->text().toStdString(),
      rclcpp::SensorDataQoS());
}

void JogControlPanel::resetSubscriptionRobotDesc()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_robot_desc_.reset();
  pub_robot_desc_ = raw_node->create_subscription<std_msgs::msg::String>(
      robot_desc_topic_edit_->text().toStdString(),
      rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg)
      {
        parseRobotDescription(msg->data);
      });
}

void JogControlPanel::initializeLayout()
{
  static const auto group_style = "QGroupBox{width: 100%; font-size: 13px; font-weight: bold;}";
  static const auto button_style = "QPushButton{ font-size: 11px;}";

  auto groupBoxA = new QGroupBox(tr("Topics"));
  {
    namespace_topic_edit_ = new QLineEdit();
    namespace_topic_edit_->setReadOnly(true);

    robot_desc_topic_edit_ = new QLineEdit();
    robot_desc_topic_edit_->setReadOnly(false);
    robot_desc_topic_edit_->setText(tr("robot_description"));

    joint_state_topic_edit_ = new QLineEdit();
    joint_state_topic_edit_->setReadOnly(false);
    joint_state_topic_edit_->setText(tr("joint_states"));

    joint_command_topic_edit_ = new QLineEdit();
    joint_command_topic_edit_->setReadOnly(false);
    joint_command_topic_edit_->setText(tr("joint_command"));

    auto form = new QFormLayout();
    form->setContentsMargins(5, 5, 5, 5);
    form->setLabelAlignment(Qt::AlignRight);
    form->addRow(tr("Namespace:"), namespace_topic_edit_);
    form->addRow(tr("Robot Description:"), robot_desc_topic_edit_);
    form->addRow(tr("Joint State:"), joint_state_topic_edit_);
    form->addRow(tr("Joint Command:"), joint_command_topic_edit_);

    groupBoxA->setLayout(form);
    groupBoxA->setStyleSheet(group_style);
  }
  auto groupBoxC = new QGroupBox("Joints States && Command");
  {
    grid_ = new QGridLayout();
    grid_->setContentsMargins(5, 5, 5, 5);
    grid_->setSizeConstraint(QLayout::SetMinimumSize);

    auto label_col0 = new QLabel("Joint Name");
    label_col0->setAlignment(Qt::AlignCenter);
    auto label_col1 = new QLabel("States\n(Read-only)");
    label_col1->setAlignment(Qt::AlignCenter);
    auto label_col2 = new QLabel("Command");
    label_col2->setAlignment(Qt::AlignCenter);
    auto label_col3 = new QLabel("");
    grid_->addWidget(label_col0, 0, 0, Qt::AlignCenter);
    grid_->addWidget(label_col1, 0, 1, Qt::AlignCenter);
    grid_->addWidget(label_col2, 0, 2, Qt::AlignCenter);
    grid_->addWidget(label_col3, 0, 3, Qt::AlignCenter);

    auto reset_btn = new QPushButton(tr("Reset"));
    reset_btn->setStyleSheet(button_style);
    reset_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(reset_btn, &QPushButton::released, this, &JogControlPanel::handleResetButton);

    auto run_btn = new QPushButton(tr("Run"));
    run_btn->setStyleSheet(button_style);
    run_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(run_btn, &QPushButton::released, this, &JogControlPanel::handleRunButton);

    auto joint_command_vbox = new QVBoxLayout();
    joint_command_vbox->setContentsMargins(3, 3, 3, 3);
    joint_command_vbox->setAlignment(Qt::AlignTop);
    joint_command_vbox->addWidget(reset_btn);
    joint_command_vbox->addWidget(run_btn);
    joint_command_vbox->addLayout(grid_);

    groupBoxC->setLayout(joint_command_vbox);
    groupBoxC->setAlignment(Qt::AlignTop);
    groupBoxC->setStyleSheet(group_style);
  }

  auto main_layout = new QVBoxLayout();
  main_layout->setContentsMargins(3, 3, 3, 3);
  // main_layout->setSizeConstraint(QLayout::SetMinimumSize);
  main_layout->setSizeConstraint(QLayout::SetMinAndMaxSize);
  main_layout->addWidget(groupBoxA);
  main_layout->addWidget(groupBoxC);

  auto scrollarea_widget = new QWidget();
  scrollarea_widget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  scrollarea_widget->setLayout(main_layout);

  auto main_scroll_area = new QScrollArea();
  main_scroll_area->setWidget(scrollarea_widget);
  main_scroll_area->setFrameShape(QFrame::NoFrame);
  main_scroll_area->setWidgetResizable(true);
  main_scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  main_scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  auto scroll_container = new QVBoxLayout();
  scroll_container->addWidget(main_scroll_area);
  setLayout(scroll_container);
}

void JogControlPanel::handleJointStates(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  static auto qdv = new QDoubleValidator(-6.28, 6.28, 10);

  for (auto i = 0u; i < msg->name.size(); i++)
  {
    // cout << " " << msg->name[i] << endl;
    const auto &joint_name = msg->name[i];
    const auto &joint_position = msg->position[i];
    QLineEdit *pos_line_edit = nullptr;
    if (joints_map_.count(joint_name) > 0)
    {
      pos_line_edit = std::get<0>(joints_map_[joint_name]);
    }
    else
    {
      pos_line_edit = new QLineEdit();
      pos_line_edit->setReadOnly(true);
      pos_line_edit->setFocusPolicy(Qt::ClickFocus);
      // joint_state_form_->addRow(tr((joint_name).c_str()), pos_line_edit);
      std::get<0>(joints_map_[joint_name]) = pos_line_edit;

      auto label = new QLabel(tr(joint_name.c_str()));
      auto cmd_line_edit = new QLineEdit(QString::number(joint_position));
      cmd_line_edit->setValidator(qdv);
      cmd_line_edit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
      std::get<1>(joints_map_[joint_name]) = cmd_line_edit;

      auto slider = new QSlider(Qt::Horizontal);
      slider->setRange(-3.14 * slider_decimal_fraction, 3.14 * slider_decimal_fraction);
      slider->setValue(joint_position * slider_decimal_fraction);
      slider->setSingleStep(1);
      connect(slider, &QSlider::valueChanged,
              [cmd_line_edit](const int &val) -> void
              {
                cmd_line_edit->setText(QString::number(val / slider_decimal_fraction));
              });
      std::get<2>(joints_map_[joint_name]) = slider;

      const auto row_index = i + 1;
      grid_->addWidget(label, row_index, 0, Qt::AlignRight);
      grid_->addWidget(pos_line_edit, row_index, 1, Qt::AlignLeft);
      grid_->addWidget(cmd_line_edit, row_index, 2, Qt::AlignRight);
      grid_->addWidget(slider, row_index, 3, Qt::AlignLeft);
    }

    if (pos_line_edit != nullptr)
      pos_line_edit->setText(QString::number(joint_position));
  }
}

void JogControlPanel::handleRunButton()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  control_msgs::msg::JointJog msg;

  msg.header.stamp = raw_node->now();

  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    const auto line_edit = std::get<1>(it->second);
    msg.joint_names.push_back(it->first);
    msg.displacements.push_back(line_edit->text().toDouble());
  }

  pub_joint_jog_->publish(msg);
}

void JogControlPanel::handleResetButton()
{
  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    const auto state_line_edit = std::get<0>(it->second);
    auto cmd_line_edit = std::get<1>(it->second);
    cmd_line_edit->setText(state_line_edit->text());
    auto slider = std::get<2>(it->second);
    slider->setValue(state_line_edit->text().toDouble() * slider_decimal_fraction);
  }
}

void JogControlPanel::parseRobotDescription(const std::string &data)
{
  // cout << data.c_str() << endl;
  tinyxml2::XMLDocument doc;
  doc.Parse(data.c_str());

  const auto model_elem = doc.FirstChildElement("sdf")->FirstChildElement("model");

  for (auto node = model_elem->FirstChildElement("joint"); node; node = node->NextSiblingElement("joint"))
  {
    const auto joint_name = node->Attribute("name");

    const auto axis = node->FirstChildElement("axis");
    if (axis == nullptr)
      continue;

    const auto axis_limit = axis->FirstChildElement("limit");
    if (axis_limit == nullptr)
      continue;

    auto slider = std::get<2>(joints_map_[joint_name]);

    if (slider == nullptr)
      continue;

    const auto lower_elem = axis_limit->FirstChildElement("lower");
    const auto lower = slider_decimal_fraction * ((lower_elem != nullptr) ? std::stod(lower_elem->GetText()) : -3.14f);

    const auto upper_elem = axis_limit->FirstChildElement("upper");
    const auto upper = slider_decimal_fraction * ((upper_elem != nullptr) ? std::stod(upper_elem->GetText()) : 3.14f);

    // cout << joint_name << " = " << lower << ", " << upper << endl;
    slider->setRange(lower, upper);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cloisim_rviz_plugin::JogControlPanel, rviz_common::Panel)