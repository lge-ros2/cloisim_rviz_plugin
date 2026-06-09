#ifndef CLOISIM_RVIZ_PLUGINS__PANELS__JOG_CONTROL_PANEL_HPP_
#define CLOISIM_RVIZ_PLUGINS__PANELS__JOG_CONTROL_PANEL_HPP_

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QtMath>
#include <control_msgs/msg/joint_jog.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace cloisim_rviz_plugin
{
class JogControlPanel : public rviz_common::Panel  // QMainWindow
{
  Q_OBJECT

 public:
  explicit JogControlPanel(QWidget *parent = nullptr);
  virtual ~JogControlPanel();

 protected:
  void onInitialize() override;

 private:
  void initializeLayout();

  void handleJointStates(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void handleResetLastStateButton();
  void handleResetZeroButton();
  void handleSetButton();
  void handleMoveButton();
  void handleStopButton();

  void parseRobotDescription(const std::string &data);

  bool generateSimpleMotionPlan();

  void resetSubscriptionJointStates();
  void resetPublisherJointCommand();
  void resetSubscriptionRobotDesc();

  void moveThread();

 private:
  static constexpr float kSliderDecimalFraction = 1000.0;

  bool worker_running_;
  bool motion_active_;
  double move_start_time_;

  struct MoveJointJog
  {
    double time;
    control_msgs::msg::JointJog jog_msg;
  };

  struct MinMax
  {
    double min;
    double max;

    MinMax(const double min = -M_PI, const double max = M_PI)
    {
      this->min = min;
      this->max = max;
    }
  };

  std::vector<MoveJointJog> planned_motion_;

  std::thread move_thread_;

  std::map<std::string, std::tuple<QLineEdit *, QLineEdit *, QSlider *>> joints_map_;
  std::map<std::string, MinMax> joints_range_map_;

  QFormLayout *form_;

  QLineEdit *namespace_topic_edit_;
  QLineEdit *robot_desc_topic_edit_;
  QLineEdit *joint_state_topic_edit_;
  QLineEdit *joint_command_topic_edit_;

  QLineEdit *control_freq_line_edit_;
  QLineEdit *move_duration_line_edit_;

  QPushButton *move_btn_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr pub_joint_jog_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_desc_;
};

}  // namespace cloisim_rviz_plugin
#endif