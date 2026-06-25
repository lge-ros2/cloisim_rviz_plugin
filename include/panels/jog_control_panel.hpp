#ifndef CLOISIM_RVIZ_PLUGINS__PANELS__JOG_CONTROL_PANEL_HPP_
#define CLOISIM_RVIZ_PLUGINS__PANELS__JOG_CONTROL_PANEL_HPP_

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QtMath>
#include <array>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
  void handleCenterTargetsButton();
  void handleResetZeroButton();
  void handleSetButton();
  void handleMoveButton();
  void handleStopButton();

  void parseRobotDescription(const std::string &data);

  bool generateSimpleMotionPlan();

  void resetSubscriptionJointStates();
  void resetPublisherJointCommand();
  void resetSubscriptionRobotDesc();

  void processMotionQueue();

  void tryCreateInteractiveMarker(const std::string &joint_name);
  void createInteractiveMarker(const std::string &joint_name);
  void handleInteractiveMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

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

  struct JointInfo
  {
    std::string type;
    std::string child_link;
    std::array<double, 3> axis = {0.0, 0.0, 1.0};
  };

  struct JointWidgets
  {
    QLineEdit *state_edit = nullptr;
    QLineEdit *command_edit = nullptr;
    QSlider *slider = nullptr;
  };

  std::vector<MoveJointJog> planned_motion_;

  QTimer *motion_timer_;

  std::map<std::string, JointWidgets> joints_map_;
  std::map<std::string, MinMax> joints_range_map_;
  std::map<std::string, JointInfo> joints_info_map_;
  std::map<std::string, double> drag_start_angles_;
  std::map<std::string, geometry_msgs::msg::Pose> drag_start_poses_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  QVBoxLayout *joint_rows_layout_;
  QFormLayout *form_;

  QLineEdit *namespace_topic_edit_;
  QLineEdit *robot_desc_topic_edit_;
  QLineEdit *joint_state_topic_edit_;
  QLineEdit *joint_command_topic_edit_;

  QLineEdit *control_freq_line_edit_;
  QLineEdit *move_duration_line_edit_;

  QPushButton *apply_now_btn_;
  QPushButton *move_btn_;
  QPushButton *stop_btn_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr pub_joint_jog_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_desc_;
};

}  // namespace cloisim_rviz_plugin
#endif