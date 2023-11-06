#include "panels/jog_control_panel.hpp"
#include <QDoubleValidator>
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
    , is_run_thread_(true)
    , is_move_joint_(false)
    , move_start_time_(0)
    , form_(nullptr)
    , namespace_topic_edit_(nullptr)
    , robot_desc_topic_edit_(nullptr)
    , joint_state_topic_edit_(nullptr)
    , joint_command_topic_edit_(nullptr)
    , control_freq_line_edit_(nullptr)
    , move_duration_line_edit_(nullptr)
    , move_btn_(nullptr)
{
  initializeLayout();
}

JogControlPanel::~JogControlPanel()
{
  handleStopButton();

  delete form_;
  form_ = nullptr;

  delete namespace_topic_edit_;
  namespace_topic_edit_ = nullptr;

  delete robot_desc_topic_edit_;
  robot_desc_topic_edit_ = nullptr;

  delete joint_state_topic_edit_;
  joint_state_topic_edit_ = nullptr;

  delete joint_command_topic_edit_;
  joint_command_topic_edit_ = nullptr;

  delete control_freq_line_edit_;
  control_freq_line_edit_ = nullptr;

  delete move_duration_line_edit_;
  move_duration_line_edit_ = nullptr;

  delete move_btn_;
  move_btn_ = nullptr;

  is_run_thread_ = false;

  if (move_thread_.joinable())
    move_thread_.join();
}

void JogControlPanel::onInitialize()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  namespace_topic_edit_->setText(tr(raw_node->get_namespace()));

  resetSubscriptionJointStates();

  resetPublisherJointCommand();

  resetSubscriptionRobotDesc();

  move_thread_ = std::thread(
      [this]()
      {
        moveThread();
      });
}

void JogControlPanel::resetSubscriptionJointStates()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  for (auto i = 1; i < form_->rowCount(); i++)
    form_->removeRow(i);

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
    form_ = new QFormLayout();
    form_->setContentsMargins(5, 5, 5, 5);

    auto label_col1 = new QLabel("States\n(Read-only)");
    label_col1->setAlignment(Qt::AlignCenter);
    auto label_col2 = new QLabel("Command");
    label_col2->setAlignment(Qt::AlignCenter);
    auto label_col3 = new QLabel("");

    auto form_first_row = new QHBoxLayout();
    form_first_row->addWidget(label_col1);
    form_first_row->addWidget(label_col2);
    form_first_row->addWidget(label_col3);

    form_->addRow(tr("Joint Name"), form_first_row);

    auto reset_btn = new QPushButton(tr("Reset to current state"));
    reset_btn->setStyleSheet(button_style);
    reset_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(reset_btn, &QPushButton::released, this, &JogControlPanel::handleResetLastStateButton);

    auto reset_zero_btn = new QPushButton(tr("Reset to zero"));
    reset_zero_btn->setStyleSheet(button_style);
    reset_zero_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(reset_zero_btn, &QPushButton::released, this, &JogControlPanel::handleResetZeroButton);

    auto set_btn = new QPushButton(tr("Set"));
    set_btn->setStyleSheet(button_style);
    set_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(set_btn, &QPushButton::released, this, &JogControlPanel::handleSetButton);

    move_duration_line_edit_ = new QLineEdit(QString::number(5));
    move_duration_line_edit_->setValidator(new QDoubleValidator(0, 1000000, 2, move_duration_line_edit_));

    control_freq_line_edit_ = new QLineEdit(QString::number(50));
    control_freq_line_edit_->setValidator(new QIntValidator(1, 100, control_freq_line_edit_));

    auto move_form = new QFormLayout();
    move_form->setContentsMargins(5, 20, 5, 0);
    move_form->addRow(tr("Control Frequency:"), control_freq_line_edit_);
    move_form->addRow(tr("Move Duration:"), move_duration_line_edit_);

    move_btn_ = new QPushButton(tr("Move"));
    move_btn_->setStyleSheet(button_style);
    move_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(move_btn_, &QPushButton::released, this, &JogControlPanel::handleMoveButton);

    auto stop_btn = new QPushButton(tr("Stop"));
    stop_btn->setStyleSheet(button_style);
    stop_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    connect(stop_btn, &QPushButton::released, this, &JogControlPanel::handleStopButton);

    auto joint_command_vbox = new QVBoxLayout();
    joint_command_vbox->setContentsMargins(3, 3, 3, 3);
    joint_command_vbox->setAlignment(Qt::AlignTop);
    joint_command_vbox->addWidget(reset_btn);
    joint_command_vbox->addWidget(reset_zero_btn);
    joint_command_vbox->addWidget(set_btn);
    joint_command_vbox->addLayout(move_form);
    joint_command_vbox->addWidget(move_btn_);
    joint_command_vbox->addWidget(stop_btn);
    joint_command_vbox->addLayout(form_);

    groupBoxC->setLayout(joint_command_vbox);
    groupBoxC->setAlignment(Qt::AlignTop);
    groupBoxC->setStyleSheet(group_style);
  }

  auto main_layout = new QVBoxLayout();
  main_layout->setContentsMargins(3, 3, 3, 3);
  // main_layout->setSizeConstraint(QLayout::SetMinimumSize);
  // main_layout->setSizeConstraint(QLayout::SetMinAndMaxSize);
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
      pos_line_edit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
      pos_line_edit->setMinimumWidth(25);
      std::get<0>(joints_map_[joint_name]) = pos_line_edit;

      auto cmd_line_edit = new QLineEdit(QString::number(joint_position));
      cmd_line_edit->setValidator(new QDoubleValidator(-6.28, 6.28, 3, cmd_line_edit));
      cmd_line_edit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
      cmd_line_edit->setMinimumWidth(25);
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

      auto form_row = new QHBoxLayout();
      form_row->addWidget(pos_line_edit);
      form_row->addWidget(cmd_line_edit);
      form_row->addWidget(slider);
      form_->addRow(tr(joint_name.c_str()), form_row);
    }

    if (pos_line_edit != nullptr)
      pos_line_edit->setText(QString::number(joint_position));
  }
}

void JogControlPanel::handleResetLastStateButton()
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

void JogControlPanel::handleResetZeroButton()
{
  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    auto slider = std::get<2>(it->second);
    slider->setValue(0 * slider_decimal_fraction);
  }
}

void JogControlPanel::handleSetButton()
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

void JogControlPanel::moveThread()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  while (is_run_thread_)
  {
    if (is_move_joint_ == false ||
        move_msgs.size() == 0)
    {
      std::this_thread::sleep_for(100us);
      continue;
    }

    auto move_msg = move_msgs.back();

    const auto time_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

    // std::cout << "running thread " << move_msgs.back().time << ", " << (time_now - move_start_time_) << std::endl;

    if (time_now - move_start_time_ > move_msgs.back().time)
    {
      move_msg.jog_msg.header.stamp = raw_node->now();
      pub_joint_jog_->publish(move_msg.jog_msg);
      std::this_thread::sleep_for(10ms);
      move_msgs.pop_back();
    }

    if (move_msgs.size() == 0)
      handleStopButton();

    std::this_thread::sleep_for(1us);
  }
}

bool JogControlPanel::generateSimpleMotionPlan()
{
  const double control_frequency = control_freq_line_edit_->text().toDouble();
  const double control_frequency_period = 1 / control_frequency;

  const auto duration_value = move_duration_line_edit_->text();
  const auto duration = duration_value.toDouble();

  // std::cout << "control_frequency=" << control_frequency << ", duration=" << duration << std::endl;

  // initial, goal, movement step, current
  std::map<std::string, std::tuple<double, double, double, double>> joints_movments_map;

  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++)
  {
    const auto joint_name = it->first;
    const auto last_state_line_edit = std::get<0>(it->second);
    const auto start = last_state_line_edit->text().toDouble();
    const auto command_line_edit = std::get<1>(it->second);
    const auto goal = command_line_edit->text().toDouble();

    const double diff = (goal >= start) ? (goal - start) : (start - goal);
    const auto movement_step = ((goal >= start) ? 1 : -1) * (diff / duration * control_frequency_period);

    std::get<0>(joints_movments_map[joint_name]) = start;
    std::get<1>(joints_movments_map[joint_name]) = goal;
    std::get<2>(joints_movments_map[joint_name]) = movement_step;
    std::get<3>(joints_movments_map[joint_name]) = start + movement_step;

    std::cout << "\t" << joint_name << ": " << start << ", " << goal << ", " << movement_step << std::endl;
  }

  move_msgs.clear();

  auto jog_time = control_frequency_period;
  while (jog_time <= (duration + std::numeric_limits<double>::epsilon()))
  {
    MoveJointJog move_msg;
    move_msg.time = jog_time;

    // std::cout << "============================== " << std::endl
    //           << jog_time << std::endl;

    for (auto it = joints_movments_map.begin(); it != joints_movments_map.end(); it++)
    {
      const auto movement_step = std::get<2>(it->second);

      const auto next_step = std::get<3>(it->second);
      move_msg.jog_msg.joint_names.push_back(it->first);
      move_msg.jog_msg.displacements.push_back(next_step);

      // std::cout << it->first << ": " << it->first << " = " << next_step << std::endl;
      // std::cout << next_step + movement_step << std::endl;
      std::get<3>(it->second) = next_step + movement_step;
    }

    move_msgs.push_back(move_msg);

    jog_time += control_frequency_period;
  }

  reverse(move_msgs.begin(), move_msgs.end());

  return true;
}

void JogControlPanel::handleMoveButton()
{
  std::cout << "Start Move" << std::endl;
  if (generateSimpleMotionPlan())
  {
    move_btn_->setEnabled(false);
    is_move_joint_ = true;
    move_start_time_ = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
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
  is_move_joint_ = false;
  move_start_time_ = 0;
  move_btn_->setEnabled(true);
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

    auto cmd_line_edit = std::get<1>(joints_map_[joint_name]);
    auto validator = reinterpret_cast<const QDoubleValidator *>(cmd_line_edit->validator());
    auto double_validator = const_cast<QDoubleValidator *>(validator);

    auto slider = std::get<2>(joints_map_[joint_name]);

    if (slider == nullptr || double_validator == nullptr)
      continue;

    const auto lower_elem = axis_limit->FirstChildElement("lower");
    const auto lower = ((lower_elem != nullptr) ? std::stod(lower_elem->GetText()) : -3.14f);

    const auto upper_elem = axis_limit->FirstChildElement("upper");
    const auto upper = ((upper_elem != nullptr) ? std::stod(upper_elem->GetText()) : 3.14f);

    // cout << joint_name << " = " << lower << ", " << upper << endl;
    slider->setRange(slider_decimal_fraction * lower, slider_decimal_fraction * upper);
    double_validator->setBottom(lower);
    double_validator->setTop(upper);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cloisim_rviz_plugin::JogControlPanel, rviz_common::Panel)