#ifndef CLOISIM_RVIZ_PLUGINS__PANELS__JOG_CONTROL_PANEL_HPP_
#define CLOISIM_RVIZ_PLUGINS__PANELS__JOG_CONTROL_PANEL_HPP_

#include <QLineEdit>
#include <QFormLayout>
#include <QGridLayout>
#include <QSlider>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cloisim_rviz_plugin
{
  class JogControlPanel : public rviz_common::Panel // QMainWindow
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
    void handleRunButton();
    void handleResetButton();

    void parseRobotDescription(const std::string &data);

    void resetSubscriptionJointStates();
    void resetPublisherJointCommand();
    void resetSubscriptionRobotDesc();

  private slots:

  private:
    static constexpr float slider_decimal_fraction = 1000.0;

    std::map<std::string, std::tuple<QLineEdit *, QLineEdit *, QSlider *>> joints_map_;

    QGridLayout *grid_;

    QLineEdit *namespace_topic_edit_;
    QLineEdit *robot_desc_topic_edit_;
    QLineEdit *joint_state_topic_edit_;
    QLineEdit *joint_command_topic_edit_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr pub_joint_jog_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pub_robot_desc_;
  };

} // namespace cloisim_rviz_plugin

#endif