#include "plan_env/grid_map.h"

#include "plan_env/inflation_kernel.h"
#include <sstream>
#include <iomanip>

std::string GridMap::streamHealth() const {
  std::ostringstream out;
  out << "map_cloud_age=" << (last_cloud_ > 0 ? node_->now().seconds() - last_cloud_ : -1.)
      << "s, pose_history=" << sensor_history_.size()
      << ", cloud_waiting_for_pose=" << (pending_cloud_ ? "true" : "false");
  return out.str();
}

std::shared_ptr<GridMap> GridMap::snapshot() const {
  auto copy = std::make_shared<GridMap>();
  copy->node_ = node_;
  copy->mp_ = mp_;
  copy->md_ = md_;
  copy->inflation_offsets_ = inflation_offsets_;
  copy->observed_ = observed_;
  copy->last_cloud_ = last_cloud_;
  copy->sensor_stamp_ = sensor_stamp_;
  copy->sensor_origin_ = sensor_origin_;
  copy->max_cloud_range_ = max_cloud_range_;
  return copy;
}

std::string GridMap::describeCell(const Eigen::Vector3d& point) {
  std::ostringstream out;
  out << std::setprecision(12) << "{\"point\":[" << point.x() << "," << point.y() << ","
      << point.z() << "],\"in_buffer\":" << (isInBuf(point) ? "true" : "false")
      << ",\"raw_occupancy\":" << getOccupancy(point)
      << ",\"inflated_occupancy\":" << getInflateOccupancy(point)
      << ",\"known_free\":" << (isKnownFree(point) ? "true" : "false")
      << ",\"cloud_stamp\":" << last_cloud_ << ",\"sensor_stamp\":" << sensor_stamp_
      << ",\"sensor_origin\":[" << sensor_origin_.x() << "," << sensor_origin_.y() << ","
      << sensor_origin_.z() << "],\"inflation_sources\":[";
  int count = 0;
  const auto cell = pos2GlobalIdx(point);
  for (const auto& offset : inflation_offsets_) {
    const Eigen::Vector3i id = cell - offset;
    if (!isInBuf(id) || getOccupancy(globalIdx2Pos(id)) != 1) continue;
    const Eigen::Vector3d p = globalIdx2Pos(id);
    if (count++) out << ",";
    out << "[" << p.x() << "," << p.y() << "," << p.z() << "]";
    if (count == 8) break;
  }
  out << "]}";
  return out.str();
}

std::string GridMap::describeObstruction(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                        bool allow_unknown) {
  auto blocked = [&](const Eigen::Vector3d& p) {
    return !isCollisionFree(p) || (!allow_unknown && !isKnownFree(p));
  };
  if (blocked(a)) return describeCell(a);
  if (blocked(b)) return describeCell(b);
  RayCaster ray;
  ray.setInput(a * mp_.resolution_inv_, b * mp_.resolution_inv_);
  Eigen::Vector3d cell;
  while (ray.step(cell)) {
    const Eigen::Vector3d center = (cell + Eigen::Vector3d::Constant(.5)) * mp_.resolution_;
    if (blocked(center)) return describeCell(center);
  }
  return describeCell(a);
}

void GridMap::initMap(rclcpp::Node::SharedPtr node) {
  node_ = node;

  node_->declare_parameter("grid_map/frame_id", "world");

  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/local_update_range_x", -1.0);
  node_->declare_parameter("grid_map/local_update_range_y", -1.0);
  node_->declare_parameter("grid_map/local_update_range_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
  node_->declare_parameter("grid_map/enable_virtual_wall", false);
  node_->declare_parameter("grid_map/virtual_ceil", -0.1);
  node_->declare_parameter("grid_map/virtual_ground", -0.1);
  node_->declare_parameter("grid_map/p_hit", 0.70);
  node_->declare_parameter("grid_map/p_miss", 0.35);
  node_->declare_parameter("grid_map/p_min", 0.12);
  node_->declare_parameter("grid_map/p_max", 0.97);
  node_->declare_parameter("grid_map/p_occ", 0.80);
  node_->declare_parameter("grid_map/fading_time", 1000.0);
  node_->declare_parameter("grid_map/show_occ_time", false);

  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);

  node_->get_parameter("grid_map/resolution", mp_.resolution_);
  node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range3d_(0));
  node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range3d_(1));
  node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range3d_(2));
  node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/enable_virtual_wall", mp_.enable_virtual_walll_);
  node_->get_parameter("grid_map/virtual_ceil", mp_.virtual_ceil_);
  node_->get_parameter("grid_map/virtual_ground", mp_.virtual_ground_);
  node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
  node_->get_parameter("grid_map/p_min", mp_.p_min_);
  node_->get_parameter("grid_map/p_max", mp_.p_max_);
  node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
  node_->get_parameter("grid_map/fading_time", mp_.fading_time_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  max_cloud_range_ = node_->declare_parameter("grid_map/max_cloud_range", 12.0);
  if (!std::isfinite(max_cloud_range_) || max_cloud_range_ <= 0)
    throw std::invalid_argument("Invalid point cloud mapping range");

  if (!std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0 ||
      !mp_.local_update_range3d_.allFinite() || (mp_.local_update_range3d_.array() <= 0).any() ||
      !std::isfinite(mp_.obstacles_inflation_) || mp_.obstacles_inflation_ < 0)
    throw std::invalid_argument("Invalid grid_map resolution/ranges/inflation; load planner.yaml");
  const auto shape = node_->declare_parameter<std::string>("grid_map/inflation_shape", "box");
  const double above = node_->declare_parameter("grid_map/body_above", mp_.obstacles_inflation_);
  const double below = node_->declare_parameter("grid_map/body_below", mp_.obstacles_inflation_);
  inflation_offsets_ =
      scut::inflationKernel(mp_.resolution_, mp_.obstacles_inflation_, shape, above, below);
  mp_.inf_grid_ = 0;
  for (const auto& offset : inflation_offsets_)
    mp_.inf_grid_ = std::max(mp_.inf_grid_, offset.cwiseAbs().maxCoeff());
  RCLCPP_INFO(node_->get_logger(),
              "Inflation %s: xy=%.3f, above=%.3f, below=%.3f m; %zu cells, resolution=%.3f m",
              shape.c_str(), mp_.obstacles_inflation_, above, below, inflation_offsets_.size(),
              mp_.resolution_);

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.local_update_range3i_ =
      (mp_.local_update_range3d_ * mp_.resolution_inv_).array().ceil().cast<int>();
  mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;
  md_.ringbuffer_inf_size3i_ =
      md_.ringbuffer_size3i_ +
      Eigen::Vector3i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_, 2 * mp_.inf_grid_);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // initialize data buffers
  Eigen::Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
  int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) *
                        (map_voxel_num3i(1) + 2 * mp_.inf_grid_) *
                        (map_voxel_num3i(2) + 2 * mp_.inf_grid_);
  md_.ringbuffer_origin3i_ = Eigen::Vector3i(0, 0, 0);
  md_.ringbuffer_inf_origin3i_ = Eigen::Vector3i(0, 0, 0);

  observed_.assign(buffer_size, 0.0);
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_);
  md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_inf_size, 0);

  sensor_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "grid_map/sensor_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr p) {
        if (p->header.frame_id != mp_.frame_id_) return;
        sensor_history_.push_back(*p);
        while (sensor_history_.size() > 20) sensor_history_.pop_front();
        processPendingCloud();
      });
  // use odometry and point cloud
  indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "grid_map/cloud", 1, std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", 1, std::bind(&GridMap::odomCallback, this, std::placeholders::_1));

  const double visualization_rate = node_->declare_parameter("grid_map/visualization_rate", 1. / .11);
  if (!std::isfinite(visualization_rate) || visualization_rate <= 0 || visualization_rate > 30)
    throw std::invalid_argument("Invalid grid_map/visualization_rate");
  vis_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1. / visualization_rate),
                                        std::bind(&GridMap::visCallback, this));
  if (mp_.fading_time_ > 0)
    fading_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.5),
                                             std::bind(&GridMap::fadingCallback, this));

  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);
  map_bounds_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("grid_map/bounds", 1);

  md_.has_odom_ = false;
}

void GridMap::visCallback() {
  if (!mp_.have_initialized_) return;

  rclcpp::Time t0 = node_->get_clock()->now();
  publishMap(true);
  publishMap(false);
  publishMapBounds();
  rclcpp::Time t1 = node_->get_clock()->now();

  if (mp_.show_occ_time_) {
    printf("Visualization(ms):%f\n", (t1 - t0).seconds() * 1000);
  }
}

void GridMap::fadingCallback() {
  const double reduce = (mp_.clamp_max_log_ - mp_.min_occupancy_log_) /
                        (mp_.fading_time_ * 2);  // function called at 2Hz
  const double low_thres = mp_.clamp_min_log_ + reduce;

  rclcpp::Time t0 = node_->get_clock()->now();
  for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i) {
    if (md_.occupancy_buffer_[i] > low_thres) {
      bool obs_flag = md_.occupancy_buffer_[i] >= mp_.min_occupancy_log_;
      md_.occupancy_buffer_[i] -= reduce;
      if (obs_flag && md_.occupancy_buffer_[i] < mp_.min_occupancy_log_) {
        Eigen::Vector3i idx = BufIdx2GlobalIdx(i);
        int inf_buf_idx = globalIdx2InfBufIdx(idx);
        changeInfBuf(false, inf_buf_idx, idx);
      }
    }
  }
  rclcpp::Time t1 = node_->get_clock()->now();

  if (mp_.show_occ_time_) {
    printf("Fading(ms):%f\n", (t1 - t0).seconds() * 1000);
  }
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  md_.body_pos_ = Eigen::Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y,
                                  odom->pose.pose.position.z);
  md_.has_odom_ = true;
  processPendingCloud();
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  pending_cloud_ = msg;
  processPendingCloud();
}

void GridMap::processPendingCloud() {
  if (!pending_cloud_) return;
  const auto msg = pending_cloud_;
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();
  const double age = node_->now().seconds() - stamp;
  if (msg->header.frame_id != mp_.frame_id_ || stamp <= last_cloud_ || age > .5 || age < -.05) {
    pending_cloud_.reset();
    return;
  }
  // Pose and large cloud have independent DDS delivery/callback ordering.
  // Match by acquisition stamp instead of comparing against only the last pose.
  const geometry_msgs::msg::PoseStamped* matched = nullptr;
  double gap = .03;
  for (const auto& pose : sensor_history_) {
    const double dt = std::abs(rclcpp::Time(pose.header.stamp).seconds() - stamp);
    if (dt < gap) { matched = &pose; gap = dt; }
  }
  if (!matched || !md_.has_odom_) return;
  sensor_origin_ = Eigen::Vector3d(matched->pose.position.x, matched->pose.position.y,
                                 matched->pose.position.z);
  sensor_stamp_ = rclcpp::Time(matched->header.stamp).seconds();
  pending_cloud_.reset();
  integrateCloud(msg);
}

void GridMap::integrateCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();
  if (!md_.has_odom_ || msg->header.frame_id != mp_.frame_id_ ||
      std::abs(stamp - sensor_stamp_) > 0.03 || node_->now().seconds() - stamp > 0.5)
    return;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (msg->width > 0 && msg->height > 0) pcl::fromROSMsg(*msg, cloud);
  if (!sensor_origin_.allFinite()) return;
  moveRingBuffer();
  if (cloud.empty()) {
    // Valid no-return frame: refresh stream liveness without clearing obstacles
    // or declaring previously unseen cells free.
    last_cloud_ = stamp;
    return;
  }
  // Dense per-frame flags avoid millions of hash lookups/allocations while
  // preserving exactly the same rays, hit-wins rule and one update per cell.
  std::vector<uint8_t> flags(md_.occupancy_buffer_.size(), 0);
  std::vector<Eigen::Vector3i> touched;
  touched.reserve(cloud.size() * 4);
  auto mark = [&](const Eigen::Vector3i& idx, uint8_t flag) {
    const int id = globalIdx2BufIdx(idx);
    if (!flags[id]) touched.push_back(idx);
    flags[id] |= flag;
  };
  RayCaster ray;
  for (const auto& pt : cloud) {
    Eigen::Vector3d end(pt.x, pt.y, pt.z);
    if (!end.allFinite()) continue;
    const double length = (end - sensor_origin_).norm();
    if (length < 0.1) continue;
    const bool hit = isInBuf(end) && length < max_cloud_range_;
    if (length >= max_cloud_range_)
      end = sensor_origin_ + (end - sensor_origin_) * (max_cloud_range_ / length);
    if (!isInBuf(end)) end = closetPointInMap(end, sensor_origin_);
    Eigen::Vector3d cell;
    ray.setInput(sensor_origin_ * mp_.resolution_inv_, end * mp_.resolution_inv_);
    while (ray.step(cell)) {
      Eigen::Vector3i idx = cell.cast<int>();
      if (!isInBuf(idx)) continue;
      mark(idx, 1);
    }
    if (hit) {
      Eigen::Vector3i idx = pos2GlobalIdx(end);
      mark(idx, 2);
    }
  }
  for (const auto& idx : touched) {
    const int id = globalIdx2BufIdx(idx);
    bool was = md_.occupancy_buffer_[id] >= mp_.min_occupancy_log_;
    if (flags[id] & 2)
      md_.occupancy_buffer_[id] = mp_.clamp_max_log_;
    else
      md_.occupancy_buffer_[id] =
          std::max(mp_.clamp_min_log_, md_.occupancy_buffer_[id] + mp_.prob_miss_log_);
    bool now = md_.occupancy_buffer_[id] >= mp_.min_occupancy_log_;
    if (was != now) changeInfBuf(now, globalIdx2InfBufIdx(idx), idx);
    observed_[id] = stamp;
  }
  // The body currently occupies this measured footprint. Seed only that
  // footprint to bridge the sensor/body lever arm; occupied cells still win.
  // This does not declare surrounding unseen space free.
  for (double x = -0.30; x <= 0.30; x += mp_.resolution_)
    for (double y = -0.30; y <= 0.30; y += mp_.resolution_)
      for (double z = -0.10; z <= 0.10; z += mp_.resolution_) {
        Eigen::Vector3d p = md_.body_pos_ + Eigen::Vector3d(x, y, z);
        if (isInBuf(p)) observed_[globalIdx2BufIdx(pos2GlobalIdx(p))] = stamp;
      }
  if (isInBuf(md_.body_pos_)) observed_[globalIdx2BufIdx(pos2GlobalIdx(md_.body_pos_))] = stamp;
  last_cloud_ = stamp;
}

void GridMap::moveRingBuffer() {
  if (!mp_.have_initialized_) initMapBoundary();

  Eigen::Vector3i center_new = pos2GlobalIdx(md_.body_pos_);
  Eigen::Vector3i ringbuffer_lowbound3i_new = center_new - mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_new =
      ringbuffer_lowbound3i_new.cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_upbound3i_new = center_new + mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_upbound3d_new =
      ringbuffer_upbound3i_new.cast<double>() * mp_.resolution_;
  ringbuffer_upbound3i_new -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
  Eigen::Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_upbound3i_new + inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_upbound3d_new + inf_grid3d;

  if (center_new(0) < md_.center_last3i_(0)) clearBuffer(0, ringbuffer_upbound3i_new(0));
  if (center_new(0) > md_.center_last3i_(0)) clearBuffer(1, ringbuffer_lowbound3i_new(0));
  if (center_new(1) < md_.center_last3i_(1)) clearBuffer(2, ringbuffer_upbound3i_new(1));
  if (center_new(1) > md_.center_last3i_(1)) clearBuffer(3, ringbuffer_lowbound3i_new(1));
  if (center_new(2) < md_.center_last3i_(2)) clearBuffer(4, ringbuffer_upbound3i_new(2));
  if (center_new(2) > md_.center_last3i_(2)) clearBuffer(5, ringbuffer_lowbound3i_new(2));

  for (int i = 0; i < 3; ++i) {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i)) {
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    }
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i)) {
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    }

    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i)) {
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    }
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i)) {
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
    }
  }

  md_.center_last3i_ = center_new;
  md_.ringbuffer_lowbound3i_ = ringbuffer_lowbound3i_new;
  md_.ringbuffer_lowbound3d_ = ringbuffer_lowbound3d_new;
  md_.ringbuffer_upbound3i_ = ringbuffer_upbound3i_new;
  md_.ringbuffer_upbound3d_ = ringbuffer_upbound3d_new;
  md_.ringbuffer_inf_lowbound3i_ = ringbuffer_inf_lowbound3i_new;
  md_.ringbuffer_inf_lowbound3d_ = ringbuffer_inf_lowbound3d_new;
  md_.ringbuffer_inf_upbound3i_ = ringbuffer_inf_upbound3i_new;
  md_.ringbuffer_inf_upbound3d_ = ringbuffer_inf_upbound3d_new;
}

void GridMap::initMapBoundary() {
  mp_.have_initialized_ = true;

  md_.center_last3i_ = pos2GlobalIdx(md_.body_pos_);

  md_.ringbuffer_lowbound3i_ = md_.center_last3i_ - mp_.local_update_range3i_;
  md_.ringbuffer_lowbound3d_ = md_.ringbuffer_lowbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ = md_.center_last3i_ + mp_.local_update_range3i_;
  md_.ringbuffer_upbound3d_ = md_.ringbuffer_upbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_inf_lowbound3i_ = md_.ringbuffer_lowbound3i_ - inf_grid3i;
  md_.ringbuffer_inf_lowbound3d_ = md_.ringbuffer_lowbound3d_ - inf_grid3d;
  md_.ringbuffer_inf_upbound3i_ = md_.ringbuffer_upbound3i_ + inf_grid3i;
  md_.ringbuffer_inf_upbound3d_ = md_.ringbuffer_upbound3d_ + inf_grid3d;

  // cout << "md_.ringbuffer_lowbound3i_=" << md_.ringbuffer_lowbound3i_.transpose() << "
  // md_.ringbuffer_lowbound3d_=" << md_.ringbuffer_lowbound3d_.transpose() << "
  // md_.ringbuffer_upbound3i_=" << md_.ringbuffer_upbound3i_.transpose() << "
  // md_.ringbuffer_upbound3d_=" << md_.ringbuffer_upbound3d_.transpose() << endl;

  for (int i = 0; i < 3; ++i) {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i)) {
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    }
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i)) {
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    }

    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i)) {
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    }
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i)) {
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
    }
  }
}

void GridMap::clearBuffer(char casein, int bound) {
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0));
       x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1));
         y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2));
           z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++z) {
        Eigen::Vector3i id_global(x, y, z);
        int id_buf = globalIdx2BufIdx(id_global);
        int id_buf_inf = globalIdx2InfBufIdx(id_global);

        md_.occupancy_buffer_[id_buf] = mp_.clamp_min_log_;
        observed_[id_buf] = 0;

        if (md_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG) {
          changeInfBuf(false, id_buf_inf, id_global);
        }
      }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d& pt,
                                          const Eigen::Vector3d& origin) {
  Eigen::Vector3d diff = pt - origin;
  Eigen::Vector3d max_tc = md_.ringbuffer_upbound3d_ - origin;
  Eigen::Vector3d min_tc = md_.ringbuffer_lowbound3d_ - origin;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return origin + (min_t - 1e-3) * diff;
}

void GridMap::publishMap(bool inflated) {
  const auto& publisher = inflated ? map_inf_pub_ : map_pub_;
  if (publisher->get_subscription_count() <= 0) return;

  // 地图是滚动窗口内的三维状态，不应再按深度相机的前向视锥裁剪。
  // 按全局栅格索引发布真实体素中心，避免高度裁剪改变采样网格的相位。
  const Eigen::Vector3i& lower =
      inflated ? md_.ringbuffer_inf_lowbound3i_ : md_.ringbuffer_lowbound3i_;
  const Eigen::Vector3i& upper =
      inflated ? md_.ringbuffer_inf_upbound3i_ : md_.ringbuffer_upbound3i_;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int x = lower.x(); x <= upper.x(); ++x)
    for (int y = lower.y(); y <= upper.y(); ++y)
      for (int z = lower.z(); z <= upper.z(); ++z) {
        const Eigen::Vector3i idx(x, y, z);
        const bool occupied =
            inflated ? md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(idx)] != 0
                     : md_.occupancy_buffer_[globalIdx2BufIdx(idx)] >= mp_.min_occupancy_log_;
        if (!occupied) continue;
        const Eigen::Vector3d position = globalIdx2Pos(idx);
        if (mp_.enable_virtual_walll_ &&
            (position.z() <= mp_.virtual_ground_ || position.z() >= mp_.virtual_ceil_))
          continue;
        cloud.push_back(pcl::PointXYZ(position.x(), position.y(), position.z()));
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = mp_.frame_id_;
  message.header.stamp = node_->now();
  publisher->publish(message);
}

void GridMap::publishMapBounds() {
  if (map_bounds_pub_->get_subscription_count() <= 0) return;
  Eigen::Vector3d lower = md_.ringbuffer_lowbound3d_;
  Eigen::Vector3d upper = md_.ringbuffer_upbound3d_;
  if (mp_.enable_virtual_walll_) {
    lower.z() = std::max(lower.z(), mp_.virtual_ground_);
    upper.z() = std::min(upper.z(), mp_.virtual_ceil_);
  }
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "local_map_window";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = upper.z() > lower.z() ? visualization_msgs::msg::Marker::ADD
                                        : visualization_msgs::msg::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.025;
  marker.color.r = 0.3;
  marker.color.g = 0.85;
  marker.color.b = 1.0;
  marker.color.a = 0.8;
  std::vector<geometry_msgs::msg::Point> corners(8);
  for (int i = 0; i < 8; ++i) {
    corners[i].x = (i & 1) ? upper.x() : lower.x();
    corners[i].y = (i & 2) ? upper.y() : lower.y();
    corners[i].z = (i & 4) ? upper.z() : lower.z();
  }
  for (int i = 0; i < 8; ++i)
    for (int axis = 0; axis < 3; ++axis)
      if (!(i & (1 << axis))) {
        marker.points.push_back(corners[i]);
        marker.points.push_back(corners[i | (1 << axis)]);
      }
  map_bounds_pub_->publish(marker);
}
