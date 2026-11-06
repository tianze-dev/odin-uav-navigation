#pragma once
#include <QElapsedTimer>
#include <QJsonObject>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uav_interfaces/msg/navigation_health.hpp>
#include <uav_interfaces/msg/navigation_status.hpp>
#include <uav_interfaces/srv/pilot_command.hpp>
#include <uav_interfaces/srv/set_goal.hpp>
class QLabel;
class QPushButton;
class QDoubleSpinBox;
class QTimer;
namespace uav_rviz_plugins {
class NavigationPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit NavigationPanel(QWidget* parent = nullptr);
  ~NavigationPanel() override;
  void onInitialize() override;

 private:
  using Trigger = std_srvs::srv::Trigger;
  using Goal = uav_interfaces::srv::SetGoal;
  using Pilot = uav_interfaces::srv::PilotCommand;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<uav_interfaces::msg::NavigationStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<uav_interfaces::msg::NavigationHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goals_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr duration_sub_;
  rclcpp::Client<Goal>::SharedPtr goal_client_;
  rclcpp::Client<Pilot>::SharedPtr pilot_client_;
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> clients_;
  std::map<std::string, QPushButton*> buttons_;
  QLabel *mission_, *px4_, *health_, *position_, *planner_, *pilot_, *result_;
  QDoubleSpinBox *x_, *y_, *z_, *height_;
  QTimer* timer_;
  QElapsedTimer diag_age_, status_age_, health_age_, operation_age_;
  QJsonObject diagnostics_;
  QString phase_;
  double px_{0}, py_{0}, pz_{0};
  bool busy_{false}, has_goal_{false}, connected_{false}, have_status_{false};
  bool source_healthy_{false};
  uint64_t serial_{0};
  std::function<void()> cancel_;
  void updateButtons();
  void finish(const QString& text, bool ok);
  void setGoal(bool preview, bool start_after = false);
  void trigger(const std::string& action);
  void pilotCommand(const std::string& action);
  void view(const std::string& kind);
  void syncGoalPlaneHeight();
};
}  // namespace uav_rviz_plugins
