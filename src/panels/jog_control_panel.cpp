#include "panels/jog_control_panel.hpp"
#include <QDoubleValidator>
#include <QFrame>
#include <QGroupBox>
#include <QIntValidator>
#include <QLabel>
#include <QMessageBox>
#include <QScrollArea>
#include <QSlider>
#include <rviz_common/display_context.hpp>
#include <tinyxml2.h>

#define INFO_STREAM RCLCPP_INFO_STREAM
#define ERROR_STREAM RCLCPP_ERROR_STREAM

using namespace cloisim_rviz_plugin;
using namespace std::chrono_literals;
using std::cout;
using std::endl;
using std::string;

JogControlPanel::JogControlPanel(QWidget *parent)
    : rviz_common::Panel(parent)
    , worker_running_(true)
    , motion_active_(false)
    , move_start_time_(0)
    , motion_timer_(nullptr)
    , joint_rows_layout_(nullptr)
    , form_(nullptr)
    , namespace_topic_edit_(nullptr)
    , robot_desc_topic_edit_(nullptr)
    , joint_state_topic_edit_(nullptr)
    , joint_command_topic_edit_(nullptr)
    , control_freq_line_edit_(nullptr)
    , move_duration_line_edit_(nullptr)
    , apply_now_btn_(nullptr)
    , move_btn_(nullptr)
    , stop_btn_(nullptr)
{
  initializeLayout();
}

JogControlPanel::~JogControlPanel()
{
  worker_running_ = false;
  handleStopButton();
}

void JogControlPanel::onInitialize()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  namespace_topic_edit_->setText(tr(raw_node->get_namespace()));

  resetSubscriptionJointStates();

  resetSubscriptionRobotDesc();

  resetPublisherJointCommand();

  motion_timer_ = new QTimer(this);
  motion_timer_->setInterval(10);
  connect(motion_timer_, &QTimer::timeout, this, &JogControlPanel::processMotionQueue);
}

void JogControlPanel::resetSubscriptionJointStates()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  while (joint_rows_layout_ != nullptr && joint_rows_layout_->count() > 0)
  {
    auto item = joint_rows_layout_->takeAt(0);
    if (item == nullptr)
      continue;

    if (item->layout() != nullptr)
      delete item->layout();
    delete item;
  }

  joints_map_.clear();

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

  sub_robot_desc_.reset();
  sub_robot_desc_ = raw_node->create_subscription<std_msgs::msg::String>(
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
    groupBoxA->setFixedHeight(140);
  }

  auto groupBoxC = new QGroupBox("Joints States && Command");
  {
    auto title_font = font();
    title_font.setBold(true);

    auto label_col1 = new QLabel("Current\n(Read-only)");
    label_col1->setFont(title_font);
    label_col1->setAlignment(Qt::AlignCenter);
    auto label_col2 = new QLabel("Target");
    label_col2->setFont(title_font);
    label_col2->setAlignment(Qt::AlignCenter);

    auto joint_name_title = new QLabel("Joint Name");
    joint_name_title->setFont(title_font);
    joint_name_title->setAlignment(Qt::AlignCenter);

    auto joint_header_layout = new QHBoxLayout();
    joint_header_layout->setContentsMargins(5, 5, 5, 0);
    joint_header_layout->setSpacing(6);
    joint_header_layout->addWidget(joint_name_title, 1);
    joint_header_layout->addWidget(label_col1, 1);
    joint_header_layout->addWidget(label_col2, 3);

    auto reset_btn = new QPushButton(tr("Center Targets"));
    reset_btn->setStyleSheet(button_style);
    reset_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(reset_btn, &QPushButton::released, this, &JogControlPanel::handleCenterTargetsButton);

    auto reset_zero_btn = new QPushButton(tr("Zero All"));
    reset_zero_btn->setStyleSheet(button_style);
    reset_zero_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(reset_zero_btn, &QPushButton::released, this, &JogControlPanel::handleResetZeroButton);

    apply_now_btn_ = new QPushButton(tr("Apply Now"));
    apply_now_btn_->setStyleSheet(button_style);
    apply_now_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(apply_now_btn_, &QPushButton::released, this, &JogControlPanel::handleSetButton);

    move_duration_line_edit_ = new QLineEdit(QString::number(5));
    move_duration_line_edit_->setValidator(new QDoubleValidator(0, 1000000, 2, move_duration_line_edit_));

    control_freq_line_edit_ = new QLineEdit(QString::number(50));
    control_freq_line_edit_->setValidator(new QIntValidator(1, 100, control_freq_line_edit_));

    auto move_inputs_row = new QHBoxLayout();
    move_inputs_row->setContentsMargins(5, 8, 5, 0);
    move_inputs_row->setSpacing(6);

    auto control_freq_label = new QLabel(tr("Control Frequency"));
    control_freq_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    control_freq_line_edit_->setMaximumWidth(64);

    auto move_duration_label = new QLabel(tr("Duration"));
    move_duration_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    move_duration_line_edit_->setMaximumWidth(64);

    move_inputs_row->addWidget(control_freq_label);
    move_inputs_row->addWidget(control_freq_line_edit_);
    move_inputs_row->addSpacing(8);
    move_inputs_row->addWidget(move_duration_label);
    move_inputs_row->addWidget(move_duration_line_edit_);
    move_inputs_row->addStretch(1);

    move_btn_ = new QPushButton(tr("Move"));
    move_btn_->setStyleSheet(button_style);
    move_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(move_btn_, &QPushButton::released, this, &JogControlPanel::handleMoveButton);

    stop_btn_ = new QPushButton(tr("Stop"));
    stop_btn_->setStyleSheet(button_style);
    stop_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    stop_btn_->setEnabled(false);
    connect(stop_btn_, &QPushButton::released, this, &JogControlPanel::handleStopButton);

    joint_rows_layout_ = new QVBoxLayout();
    joint_rows_layout_->setContentsMargins(5, 5, 5, 5);
    joint_rows_layout_->setSpacing(4);
    joint_rows_layout_->setAlignment(Qt::AlignTop);

    auto joint_rows_widget = new QWidget();
    joint_rows_widget->setLayout(joint_rows_layout_);

    auto joint_rows_scroll_area = new QScrollArea();
    joint_rows_scroll_area->setWidget(joint_rows_widget);
    joint_rows_scroll_area->setWidgetResizable(true);
    joint_rows_scroll_area->setFrameShape(QFrame::NoFrame);
    joint_rows_scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    joint_rows_scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    joint_rows_scroll_area->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto target_helper_row = new QHBoxLayout();
    target_helper_row->setContentsMargins(0, 0, 0, 0);
    target_helper_row->setSpacing(6);
    target_helper_row->addWidget(reset_btn);
    target_helper_row->addStretch(1);
    target_helper_row->addWidget(reset_zero_btn);

    auto motion_action_row = new QHBoxLayout();
    motion_action_row->setContentsMargins(0, 0, 0, 0);
    motion_action_row->setSpacing(6);
    motion_action_row->addWidget(move_btn_);
    motion_action_row->addWidget(stop_btn_);

    apply_now_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    auto move_group = new QGroupBox(tr("Timed Motion"));
    move_group->setStyleSheet(group_style);

    auto move_group_layout = new QVBoxLayout();
    move_group_layout->setContentsMargins(3, 3, 3, 3);
    move_group_layout->setSpacing(6);
    move_group_layout->addLayout(move_inputs_row);
    move_group_layout->addLayout(motion_action_row);
    move_group->setLayout(move_group_layout);

    auto joint_command_vbox = new QVBoxLayout();
    joint_command_vbox->setContentsMargins(3, 3, 3, 3);
    joint_command_vbox->setAlignment(Qt::AlignTop);
    joint_command_vbox->addWidget(apply_now_btn_);
    joint_command_vbox->addWidget(move_group);
    joint_command_vbox->addLayout(joint_header_layout);
    joint_command_vbox->addWidget(joint_rows_scroll_area, 1);
    joint_command_vbox->addLayout(target_helper_row);

    groupBoxC->setLayout(joint_command_vbox);
    groupBoxC->setAlignment(Qt::AlignTop);
    groupBoxC->setStyleSheet(group_style);
  }

  auto main_layout = new QVBoxLayout();
  main_layout->setContentsMargins(3, 3, 3, 3);
  main_layout->addWidget(groupBoxA);
  main_layout->addWidget(groupBoxC, 1);
  setLayout(main_layout);
}

void JogControlPanel::handleJointStates(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  for (auto i = 0u; i < msg->name.size(); i++)
  {
    const auto &joint_name = msg->name[i];
    const auto &joint_position = msg->position[i];
    QLineEdit *pos_line_edit = nullptr;

    if (joints_map_.count(joint_name) > 0)
    {
      pos_line_edit = joints_map_[joint_name].state_edit;

      if (joints_range_map_.find(joint_name) != joints_range_map_.end())
      {
        const auto joint_minmax = joints_range_map_[joint_name];

        auto cmd_line_edit = joints_map_[joint_name].command_edit;
        if (cmd_line_edit != nullptr)
        {
          auto validator = reinterpret_cast<const QDoubleValidator *>(cmd_line_edit->validator());
          auto double_validator = const_cast<QDoubleValidator *>(validator);
          double_validator->setBottom(joint_minmax.min);
          double_validator->setTop(joint_minmax.max);
        }

        auto slider = joints_map_[joint_name].slider;
        if (slider != nullptr)
        {
          slider->setRange(kSliderDecimalFraction * joint_minmax.min, kSliderDecimalFraction * joint_minmax.max);
        }
      }
    }
    else
    {
      pos_line_edit = new QLineEdit();
      pos_line_edit->setReadOnly(true);
      pos_line_edit->setFocusPolicy(Qt::ClickFocus);
      pos_line_edit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
      pos_line_edit->setMinimumWidth(25);
      joints_map_[joint_name].state_edit = pos_line_edit;

      auto cmd_line_edit = new QLineEdit(QString::number(joint_position));
      cmd_line_edit->setValidator(new QDoubleValidator(-6.28, 6.28, 3, cmd_line_edit));
      cmd_line_edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
      joints_map_[joint_name].command_edit = cmd_line_edit;

      auto slider = new QSlider(Qt::Horizontal);
      slider->setRange(-M_PI * kSliderDecimalFraction, M_PI * kSliderDecimalFraction);
      slider->setValue(joint_position * kSliderDecimalFraction);
      slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
      slider->setSingleStep(1);
      connect(slider, &QSlider::valueChanged,
              [cmd_line_edit](const int &val) -> void
              {
                cmd_line_edit->setText(QString::number(val / kSliderDecimalFraction));
              });
      joints_map_[joint_name].slider = slider;

      auto name_label = new QLabel(tr(joint_name.c_str()));
      name_label->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);

      auto target_layout = new QVBoxLayout();
      target_layout->setContentsMargins(0, 0, 0, 0);
      target_layout->setSpacing(2);
      target_layout->addWidget(cmd_line_edit);
      target_layout->addWidget(slider);

      auto target_widget = new QWidget();
      target_widget->setLayout(target_layout);

      auto form_row = new QHBoxLayout();
      form_row->setContentsMargins(0, 0, 0, 0);
      form_row->setSpacing(6);
      form_row->addWidget(name_label, 1);
      form_row->addWidget(pos_line_edit, 1);
      form_row->addWidget(target_widget, 3);
      joint_rows_layout_->addLayout(form_row);

      auto divider = new QFrame();
      divider->setFrameShape(QFrame::HLine);
      divider->setFrameShadow(QFrame::Plain);
      divider->setLineWidth(1);
      divider->setContentsMargins(0, 5, 0, 5);
      divider->setStyleSheet("color: rgba(180, 180, 180, 95);");
      joint_rows_layout_->addWidget(divider);
    }

    if (pos_line_edit != nullptr)
      pos_line_edit->setText(QString::number(joint_position));
  }
}

void JogControlPanel::handleCenterTargetsButton()
{
  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    auto cmd_line_edit = it->second.command_edit;
    auto slider = it->second.slider;

    auto range_it = joints_range_map_.find(it->first);
    const auto min_value = (range_it != joints_range_map_.end()) ? range_it->second.min : -M_PI;
    const auto max_value = (range_it != joints_range_map_.end()) ? range_it->second.max : M_PI;
    const auto midpoint = (min_value + max_value) * 0.5;

    cmd_line_edit->setText(QString::number(midpoint));
    if (slider != nullptr)
      slider->setValue(midpoint * kSliderDecimalFraction);
  }
}

void JogControlPanel::handleResetZeroButton()
{
  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    auto slider = it->second.slider;
    slider->setValue(0 * kSliderDecimalFraction);
  }
}

void JogControlPanel::handleSetButton()
{
  if (motion_active_)
    return;

  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  control_msgs::msg::JointJog msg;
  msg.header.stamp = raw_node->now();

  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    const auto line_edit = it->second.command_edit;
    msg.joint_names.push_back(it->first);
    msg.displacements.push_back(line_edit->text().toDouble());
  }

  pub_joint_jog_->publish(msg);
}

void JogControlPanel::processMotionQueue()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  if (worker_running_ == false)
  {
    if (motion_timer_ != nullptr)
      motion_timer_->stop();
    return;
  }

  if (motion_active_ == false ||
      planned_motion_.empty())
    return;

  auto move_msg = planned_motion_.back();

  const auto time_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

  if (time_now - move_start_time_ > planned_motion_.back().time)
  {
    move_msg.jog_msg.header.stamp = raw_node->now();
    pub_joint_jog_->publish(move_msg.jog_msg);
    planned_motion_.pop_back();
  }

  if (planned_motion_.empty())
    handleStopButton();
}

bool JogControlPanel::generateSimpleMotionPlan()
{
  const double control_frequency = control_freq_line_edit_->text().toDouble();
  const double control_frequency_period = 1 / control_frequency;

  const auto duration_value = move_duration_line_edit_->text();
  const auto duration = duration_value.toDouble();

  // std::cout << "control_frequency=" << control_frequency << ", duration=" << duration << std::endl;

  // initial, goal, movement step, current
  std::map<std::string, std::tuple<double, double, double, double>> joints_movements_map;

  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    const auto joint_name = it->first;
    const auto last_state_line_edit = it->second.state_edit;
    const auto start = last_state_line_edit->text().toDouble();
    const auto command_line_edit = it->second.command_edit;
    const auto goal = command_line_edit->text().toDouble();

    const double diff = (goal >= start) ? (goal - start) : (start - goal);
    const auto movement_step = ((goal >= start) ? 1 : -1) * (diff / duration * control_frequency_period);

    std::get<0>(joints_movements_map[joint_name]) = start;
    std::get<1>(joints_movements_map[joint_name]) = goal;
    std::get<2>(joints_movements_map[joint_name]) = movement_step;
    std::get<3>(joints_movements_map[joint_name]) = start + movement_step;

    std::cout << "\t" << joint_name << ": " << start << ", " << goal << ", " << movement_step << std::endl;
  }

  planned_motion_.clear();

  auto jog_time = control_frequency_period;
  while (jog_time <= (duration + std::numeric_limits<double>::epsilon()))
  {
    MoveJointJog move_msg;
    move_msg.time = jog_time;

    // std::cout << "============================== " << std::endl
    //           << jog_time << std::endl;

    for (auto it = joints_movements_map.begin(); it != joints_movements_map.end(); it++)
    {
      const auto movement_step = std::get<2>(it->second);

      const auto next_step = std::get<3>(it->second);
      move_msg.jog_msg.joint_names.push_back(it->first);
      move_msg.jog_msg.displacements.push_back(next_step);

      // std::cout << it->first << ": " << it->first << " = " << next_step << std::endl;
      // std::cout << next_step + movement_step << std::endl;
      std::get<3>(it->second) = next_step + movement_step;
    }

    planned_motion_.push_back(move_msg);

    jog_time += control_frequency_period;
  }

  reverse(planned_motion_.begin(), planned_motion_.end());

  return true;
}

void JogControlPanel::handleMoveButton()
{
  std::cout << "Start Move" << std::endl;
  if (generateSimpleMotionPlan())
  {
    move_btn_->setEnabled(false);
    if (apply_now_btn_ != nullptr)
      apply_now_btn_->setEnabled(false);
    if (stop_btn_ != nullptr)
      stop_btn_->setEnabled(true);
    motion_active_ = true;
    move_start_time_ = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (motion_timer_ != nullptr)
      motion_timer_->start();
  }
  else
  {
    QMessageBox::warning(this,
                         tr("Move Jogs"),
                         tr("Invalid range detected."));
  }
}

void JogControlPanel::handleStopButton()
{
  std::cout << "Stop Move" << std::endl;
  motion_active_ = false;
  move_start_time_ = 0;
  planned_motion_.clear();
  if (motion_timer_ != nullptr)
    motion_timer_->stop();
  move_btn_->setEnabled(true);
  if (apply_now_btn_ != nullptr)
    apply_now_btn_->setEnabled(true);
  if (stop_btn_ != nullptr)
    stop_btn_->setEnabled(false);
}

void JogControlPanel::parseRobotDescription(const std::string &data)
{
  tinyxml2::XMLDocument doc;
  const auto parse_result = doc.Parse(data.c_str());
  if (parse_result != tinyxml2::XML_SUCCESS)
  {
    ERROR_STREAM(rclcpp::get_logger("JogControlPanel"),
                 "Failed to parse robot_description XML: " << doc.ErrorStr());
    return;
  }

  joints_range_map_.clear();

  auto *robot_elem = doc.FirstChildElement("robot");
  if (robot_elem == nullptr)
  {
    ERROR_STREAM(rclcpp::get_logger("JogControlPanel"),
                 "robot_description root is not URDF <robot>");
    return;
  }

  for (auto *node = robot_elem->FirstChildElement("joint"); node; node = node->NextSiblingElement("joint"))
  {
    const auto joint_name = node->Attribute("name");
    if (joint_name == nullptr)
      continue;

    const auto joint_type = node->Attribute("type");
    if (joint_type == nullptr)
      continue;

    if (std::strcmp(joint_type, "fixed") == 0 ||
        std::strcmp(joint_type, "floating") == 0 ||
        std::strcmp(joint_type, "planar") == 0)
      continue;

    if (std::strcmp(joint_type, "continuous") == 0)
    {
      joints_range_map_[joint_name] = MinMax(-M_PI, M_PI);
      continue;
    }

    const auto *limit = node->FirstChildElement("limit");
    if (limit == nullptr)
      continue;

    const auto *lower_attr = limit->Attribute("lower");
    const auto lower = ((lower_attr != nullptr) ? std::stod(lower_attr) : -M_PI);

    const auto *upper_attr = limit->Attribute("upper");
    const auto upper = ((upper_attr != nullptr) ? std::stod(upper_attr) : M_PI);

    joints_range_map_[joint_name] = MinMax(lower, upper);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cloisim_rviz_plugin::JogControlPanel, rviz_common::Panel)