#pragma once
#include <rviz_common/tool.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
namespace uav_rviz_plugins {
class GoalPlaneTool : public rviz_common::Tool {
 public:
  GoalPlaneTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override {}
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
 private:
  rviz_common::properties::FloatProperty* height_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
};
}
