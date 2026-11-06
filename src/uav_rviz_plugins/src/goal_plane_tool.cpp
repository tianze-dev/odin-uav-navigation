#include "uav_rviz_plugins/goal_plane_tool.hpp"
#include "uav_rviz_plugins/goal_projection.hpp"
#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreViewport.h>
#include <QCursor>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/render_window.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace uav_rviz_plugins {
GoalPlaneTool::GoalPlaneTool() {
  shortcut_key_='p';
  height_=new rviz_common::properties::FloatProperty(
      "Height",2.5,"目标高度（odom / 米）；鼠标射线与此水平面相交，不依赖点云或地图表面。",getPropertyContainer());
}
void GoalPlaneTool::onInitialize() {
  setName("导航目标点");
  node_=context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_=node_->create_publisher<geometry_msgs::msg::PointStamped>("/navigation/ui/clicked_point",10);
  setCursor(QCursor(Qt::CrossCursor));
}
void GoalPlaneTool::activate() {
  setStatus("点击选择目标（可在蓝框外）；高度由面板 Z 设置。只暂存目标，不自动起飞或导航。");
}
int GoalPlaneTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  if (!event.panel || !publisher_) return 0;
  if (context_->getFixedFrame()!="odom") {
    setStatus("导航选点需要 Fixed Frame = odom");return 0;
  }
  auto* viewport=rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  if (!viewport || !viewport->getActualWidth() || !viewport->getActualHeight()) return 0;
  const auto ray=viewport->getCamera()->getCameraToViewportRay(
      double(event.x)/viewport->getActualWidth(), double(event.y)/viewport->getActualHeight());
  const auto& o=ray.getOrigin();const auto& d=ray.getDirection();
  const auto p=goalOnHeightPlane({o.x,o.y,o.z},{d.x,d.y,d.z},height_->getFloat());
  if (!p) { setStatus("视线未与目标高度平面相交，请调整视角后选点");return 0; }
  setStatus(QString("目标 (%1, %2, %3) m；点击暂存，开始导航需单独操作")
      .arg((*p)[0],0,'f',2).arg((*p)[1],0,'f',2).arg((*p)[2],0,'f',2));
  if (event.leftUp()) {
    geometry_msgs::msg::PointStamped point;point.header.frame_id="odom";point.header.stamp=node_->now();
    point.point.x=(*p)[0];point.point.y=(*p)[1];point.point.z=(*p)[2];publisher_->publish(point);
    return Finished|Render;
  }
  return 0;
}
}
PLUGINLIB_EXPORT_CLASS(uav_rviz_plugins::GoalPlaneTool,rviz_common::Tool)
