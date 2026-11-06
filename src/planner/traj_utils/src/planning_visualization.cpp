#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner {
PlanningVisualization::PlanningVisualization(const rclcpp::Node::SharedPtr& node) {
  node_ = node;

  goal_point_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/planning/goal_point", rclcpp::QoS(20).transient_local());
  global_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/planning/global_list", rclcpp::QoS(20).transient_local());
  init_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/planning/init_list", rclcpp::QoS(20).transient_local());
  optimal_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/planning/optimal_list", rclcpp::QoS(20).transient_local());
  a_star_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/planning/a_star_list", rclcpp::QoS(20).transient_local());
  failed_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/planning/failed_list", rclcpp::QoS(20).transient_local());
  accepted_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/planning/accepted", rclcpp::QoS(2).transient_local());
}

void PlanningVisualization::displayMarkerList(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id,
    bool show_sphere /* = true */) {
  visualization_msgs::msg::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "odom";
  sphere.header.stamp = line_strip.header.stamp = node_->get_clock()->now();
  sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = color(0);
  sphere.color.g = line_strip.color.g = color(1);
  sphere.color.b = line_strip.color.b = color(2);
  sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  line_strip.scale.x = scale / 2;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    if (show_sphere) sphere.points.push_back(pt);
    line_strip.points.push_back(pt);
  }
  if (show_sphere) pub->publish(sphere);
  pub->publish(line_strip);
}

void PlanningVisualization::generatePathDisplayArray(visualization_msgs::msg::MarkerArray& array,
                                                     const vector<Eigen::Vector3d>& list,
                                                     double scale, Eigen::Vector4d color, int id) {
  visualization_msgs::msg::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "odom";
  sphere.header.stamp = line_strip.header.stamp = node_->get_clock()->now();
  sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = color(0);
  sphere.color.g = line_strip.color.g = color(1);
  sphere.color.b = line_strip.color.b = color(2);
  sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  line_strip.scale.x = scale / 3;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    sphere.points.push_back(pt);
    line_strip.points.push_back(pt);
  }
  array.markers.push_back(sphere);
  array.markers.push_back(line_strip);
}

void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::msg::MarkerArray& array,
                                                      const vector<Eigen::Vector3d>& list,
                                                      double scale, Eigen::Vector4d color, int id) {
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "odom";
  arrow.header.stamp = node_->get_clock()->now();
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;

  arrow.color.r = color(0);
  arrow.color.g = color(1);
  arrow.color.b = color(2);
  arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  arrow.scale.x = scale;
  arrow.scale.y = 2 * scale;
  arrow.scale.z = 2 * scale;

  geometry_msgs::msg::Point start, end;
  for (int i = 0; i < int(list.size() / 2); i++) {
    start.x = list[2 * i](0);
    start.y = list[2 * i](1);
    start.z = list[2 * i](2);
    end.x = list[2 * i + 1](0);
    end.y = list[2 * i + 1](1);
    end.z = list[2 * i + 1](2);
    arrow.points.clear();
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.id = i + id * 1000;

    array.markers.push_back(arrow);
  }
}

void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color,
                                             const double scale, int id) {
  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = "odom";
  sphere.header.stamp = node_->get_clock()->now();
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.id = id;

  sphere.pose.orientation.w = 1.0;
  sphere.color.r = color(0);
  sphere.color.g = color(1);
  sphere.color.b = color(2);
  sphere.color.a = color(3);
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  sphere.pose.position.x = goal_point(0);
  sphere.pose.position.y = goal_point(1);
  sphere.pose.position.z = goal_point(2);

  goal_point_pub->publish(sphere);
}

void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts,
                                                  const double scale, int id) {
  if (global_list_pub->get_subscription_count() == 0) {
    return;
  }

  Eigen::Vector4d color(0.6, 0.65, 0.75, 0.8);
  displayMarkerList(global_list_pub, init_pts, scale, color, id);
}

void PlanningVisualization::displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs,
                                                     const double scale) {
  if (init_list_pub->get_subscription_count() == 0) {
    return;
  }

  static thread_local int last_nums = 0;

  for (int id = 0; id < last_nums; id++) {
    Eigen::Vector4d color(0, 0, 0, 0);
    vector<Eigen::Vector3d> blank;
    displayMarkerList(init_list_pub, blank, scale, color, id, false);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
  }
  last_nums = 0;

  for (int id = 0; id < (int)init_trajs.size(); id++) {
    Eigen::Vector4d color(0, 0, 1, 0.7);
    displayMarkerList(init_list_pub, init_trajs[id], scale, color, id, false);
    rclcpp::sleep_for(std::chrono::milliseconds(1));

    last_nums++;
  }
}

void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts,
                                                const double scale, int id) {
  if (init_list_pub->get_subscription_count() == 0) {
    return;
  }

  Eigen::Vector4d color(0.25, 0.5, 1, 1);
  displayMarkerList(init_list_pub, init_pts, scale, color, id);
}

void PlanningVisualization::displayMultiOptimalPathList(
    vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale)  // zxzxzx
{
  if (optimal_list_pub->get_subscription_count() == 0) {
    return;
  }

  static thread_local int last_nums = 0;

  for (int id = 0; id < last_nums; id++) {
    Eigen::Vector4d color(0, 0, 0, 0);
    vector<Eigen::Vector3d> blank;
    displayMarkerList(optimal_list_pub, blank, scale, color, id + 10, false);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
  }
  last_nums = 0;

  for (int id = 0; id < (int)optimal_trajs.size(); id++) {
    Eigen::Vector4d color(1, 0, 0, 0.7);
    displayMarkerList(optimal_list_pub, optimal_trajs[id], scale, color, id + 10, false);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
    last_nums++;
  }
}

void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id) {
  if (optimal_list_pub->get_subscription_count() == 0) {
    return;
  }

  vector<Eigen::Vector3d> list;
  for (int i = 0; i < optimal_pts.cols(); i++) {
    Eigen::Vector3d pt = optimal_pts.col(i).transpose();
    list.push_back(pt);
  }
  Eigen::Vector4d color(1, 0.25, 0.65, 1);
  displayMarkerList(optimal_list_pub, list, 0.05, color, id, false);
}

void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id) {
  if (failed_list_pub->get_subscription_count() == 0) {
    return;
  }

  vector<Eigen::Vector3d> list;
  for (int i = 0; i < failed_pts.cols(); i++) {
    Eigen::Vector3d pt = failed_pts.col(i).transpose();
    list.push_back(pt);
  }
  Eigen::Vector4d color(0.3, 0, 0, 1);
  displayMarkerList(failed_list_pub, list, 0.15, color, id);
}

void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> paths,
                                             int id) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = node_->now();
  m.ns = "astar";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.action = paths.empty() ? visualization_msgs::msg::Marker::DELETE
                           : visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1;
  m.scale.x = 0.055;
  m.color.r = 1;
  m.color.g = 0.75;
  m.color.b = 0.1;
  m.color.a = 1;
  for (const auto& path : paths)
    for (size_t i = 1; i < path.size(); ++i)
      for (const auto& p : {path[i - 1], path[i]}) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        m.points.push_back(point);
      }
  a_star_list_pub->publish(m);
}

void PlanningVisualization::displayAcceptedPath(const vector<Eigen::Vector3d>& points) {
  if (points.empty()) {
    visualization_msgs::msg::Marker m;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    accepted_pub->publish(m);
    return;
  }
  displayMarkerList(accepted_pub, points, 0.05, Eigen::Vector4d(0.1, 1, 0.7, 1), 0, false);
}

void PlanningVisualization::clearActiveGoal() {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.id = 1;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  goal_point_pub->publish(marker);
}

void PlanningVisualization::clearProcess() {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = node_->now();
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  for (auto& publisher :
       {global_list_pub, init_list_pub, optimal_list_pub, a_star_list_pub, failed_list_pub})
    publisher->publish(m);
}

void PlanningVisualization::displayArrowList(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id) {
  visualization_msgs::msg::MarkerArray array;
  pub->publish(array);

  generateArrowDisplayArray(array, list, scale, color, id);

  pub->publish(array);
}

}  // namespace ego_planner
