#ifndef _GRID_MAP_
#define _GRID_MAP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/raycast.h>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tuple>
#include <unordered_set>
#include <visualization_msgs/msg/marker.hpp>

#define logit(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767

using namespace std;

// constant parameters

struct MappingParameters {
  bool have_initialized_ = false;

  /* map properties */
  Eigen::Vector3d local_update_range3d_;
  Eigen::Vector3i local_update_range3i_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  int inf_grid_;
  string frame_id_;
  bool enable_virtual_walll_;
  double virtual_ceil_, virtual_ground_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;  // logit of occupancy probability
  double fading_time_;

  /* visualization and computation time display */
  bool show_occ_time_;
};

// intermediate mapping data for fusion

struct MappingData {
  Eigen::Vector3i center_last3i_;
  // Eigen::Vector3d ringbuffer_origin3d_;
  Eigen::Vector3i ringbuffer_origin3i_;
  // Eigen::Vector3d ringbuffer_division3d_;
  // Eigen::Vector3i ringbuffer_division3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_;
  Eigen::Vector3i ringbuffer_lowbound3i_;
  Eigen::Vector3d ringbuffer_upbound3d_;
  Eigen::Vector3i ringbuffer_upbound3i_;
  // Eigen::Vector3d ringbuffer_size3d_;
  Eigen::Vector3i ringbuffer_size3i_;
  Eigen::Vector3i ringbuffer_inf_origin3i_;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_;
  Eigen::Vector3d ringbuffer_inf_upbound3d_;
  Eigen::Vector3i ringbuffer_inf_upbound3i_;
  Eigen::Vector3i ringbuffer_inf_size3i_;

  // main map data, occupancy of each voxel

  std::vector<double> occupancy_buffer_;
  std::vector<uint16_t> occupancy_buffer_inflate_;

  // Body position anchors the rolling map; rays use the timestamped sensor origin.
  Eigen::Vector3d body_pos_;
  bool has_odom_ = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
  friend struct GridMapRegressionAccess;

 public:
  GridMap() {}
  bool isKnownFree(const Eigen::Vector3d& p) {
    if (!isInBuf(p) || getInflateOccupancy(p)) return false;
    const int idx = globalIdx2BufIdx(pos2GlobalIdx(p));
    // Observation memory belongs to the rolling map. Do not turn a measured
    // free cell into unknown merely because it is temporarily outside the FOV.
    // New rays update occupancy; buffer rolling/reset clears observation state.
    // Whole-source freshness is independently enforced by freshCloud().
    return observed_[idx] > 0;
  }
  bool isKnownFreeSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    if (!a.allFinite() || !b.allFinite() || !isKnownFree(a) || !isKnownFree(b)) return false;
    RayCaster ray;
    ray.setInput(a * mp_.resolution_inv_, b * mp_.resolution_inv_);
    Eigen::Vector3d cell;
    while (ray.step(cell)) {
      const Eigen::Vector3d center = (cell + Eigen::Vector3d::Constant(0.5)) * mp_.resolution_;
      if (!isKnownFree(center)) return false;
    }
    return true;
  }
  // Optimistic EGO semantics: unknown is traversable, occupied is not. Virtual
  // flight bounds apply even when a query is outside the rolling buffer.
  bool isCollisionFree(const Eigen::Vector3d& p) {
    if (!p.allFinite() ||
        (mp_.enable_virtual_walll_ && (p.z() <= mp_.virtual_ground_ || p.z() >= mp_.virtual_ceil_)))
      return false;
    return getInflateOccupancy(p) == 0;
  }
  bool isCollisionFreeSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    if (!isCollisionFree(a) || !isCollisionFree(b)) return false;
    RayCaster ray;
    ray.setInput(a * mp_.resolution_inv_, b * mp_.resolution_inv_);
    Eigen::Vector3d cell;
    while (ray.step(cell)) {
      const Eigen::Vector3d p = (cell + Eigen::Vector3d::Constant(.5)) * mp_.resolution_;
      if (!isCollisionFree(p)) return false;
    }
    return true;
  }
  bool freshCloud() const {
    return last_cloud_ > 0 && node_->now().seconds() - last_cloud_ < 0.5;
  }
  std::string streamHealth() const;

  ~GridMap() {}

  void initMap(rclcpp::Node::SharedPtr node);
  // Read-only planning copy: no subscriptions/timers or shared mutable buffers.
  std::shared_ptr<GridMap> snapshot() const;
  std::string describeCell(const Eigen::Vector3d& point);
  std::string describeObstruction(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                  bool allow_unknown);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  inline double getResolution();
  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  std::vector<Eigen::Vector3i> inflation_offsets_;
  std::vector<double> observed_;
  double last_cloud_{0}, sensor_stamp_{0};
  Eigen::Vector3d sensor_origin_{Eigen::Vector3d::Zero()};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_sub_;
  MappingParameters mp_;
  MappingData md_;

  inline Eigen::Vector3d globalIdx2Pos(const Eigen::Vector3i& id);   // 1.69ns
  inline Eigen::Vector3i pos2GlobalIdx(const Eigen::Vector3d& pos);  // 0.13ns
  inline int globalIdx2BufIdx(const Eigen::Vector3i& id);            // 2.2ns
  inline int globalIdx2InfBufIdx(const Eigen::Vector3i& id);         // 2.2ns
  inline Eigen::Vector3i BufIdx2GlobalIdx(size_t address);           // 10.18ns
  inline Eigen::Vector3i infBufIdx2GlobalIdx(size_t address);        // 10.18ns
  inline bool isInBuf(const Eigen::Vector3d& pos);
  inline bool isInBuf(const Eigen::Vector3i& idx);
  inline bool isInInfBuf(const Eigen::Vector3d& pos);
  inline bool isInInfBuf(const Eigen::Vector3i& idx);

  void publishMap(bool inflated);
  void publishMapBounds();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& img);
  void processPendingCloud();
  void integrateCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& img);
  std::deque<geometry_msgs::msg::PoseStamped> sensor_history_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  // update occupancy by raycasting
  void visCallback();
  void fadingCallback();

  void clearBuffer(char casein, int bound);

  // main update process
  void moveRingBuffer();

  inline void changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& origin);
  void initMapBoundary();

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr indep_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr indep_odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  double max_cloud_range_{12.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_bounds_pub_;

  rclcpp::TimerBase::SharedPtr vis_timer_;
  rclcpp::TimerBase::SharedPtr fading_timer_;
};

/* ============================== definition of inline function
 * ============================== */

inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx,
                                  const Eigen::Vector3i global_idx) {
  // Validate a removal before mutating any counters. Never erase the entire
  // obstacle map to conceal a reference-count error.
  if (!dir) {
    if (md_.occupancy_buffer_inflate_[inf_buf_idx] < GRID_MAP_OBS_FLAG)
      throw std::runtime_error("Removing an unregistered occupied voxel");
    for (const auto& offset : inflation_offsets_) {
      const auto value = md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(global_idx + offset)];
      if (value % GRID_MAP_OBS_FLAG == 0)
        throw std::runtime_error("Inflation reference count underflow");
    }
  }
  md_.occupancy_buffer_inflate_[inf_buf_idx] += dir ? GRID_MAP_OBS_FLAG : -GRID_MAP_OBS_FLAG;
  for (const auto& offset : inflation_offsets_) {
    auto& value = md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(global_idx + offset)];
    if (dir)
      ++value;
    else
      --value;
  }
}

inline int GridMap::globalIdx2BufIdx(const Eigen::Vector3i& id) {
  int x_buffer = (id(0) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(0);
  int y_buffer = (id(1) - md_.ringbuffer_origin3i_(1)) % md_.ringbuffer_size3i_(1);
  int z_buffer = (id(2) - md_.ringbuffer_origin3i_(2)) % md_.ringbuffer_size3i_(2);
  if (x_buffer < 0) x_buffer += md_.ringbuffer_size3i_(0);
  if (y_buffer < 0) y_buffer += md_.ringbuffer_size3i_(1);
  if (z_buffer < 0) z_buffer += md_.ringbuffer_size3i_(2);

  return md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1) * z_buffer +
         md_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
}

inline int GridMap::globalIdx2InfBufIdx(const Eigen::Vector3i& id) {
  int x_buffer = (id(0) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(0);
  int y_buffer = (id(1) - md_.ringbuffer_inf_origin3i_(1)) % md_.ringbuffer_inf_size3i_(1);
  int z_buffer = (id(2) - md_.ringbuffer_inf_origin3i_(2)) % md_.ringbuffer_inf_size3i_(2);
  if (x_buffer < 0) x_buffer += md_.ringbuffer_inf_size3i_(0);
  if (y_buffer < 0) y_buffer += md_.ringbuffer_inf_size3i_(1);
  if (z_buffer < 0) z_buffer += md_.ringbuffer_inf_size3i_(2);

  return md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1) * z_buffer +
         md_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Eigen::Vector3i GridMap::BufIdx2GlobalIdx(size_t address) {
  const int ringbuffer_xysize = md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / md_.ringbuffer_size3i_(0);
  int xid_in_buffer = address % md_.ringbuffer_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_origin3i_(0);
  if (xid_global > md_.ringbuffer_upbound3i_(0)) xid_global -= md_.ringbuffer_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_origin3i_(1);
  if (yid_global > md_.ringbuffer_upbound3i_(1)) yid_global -= md_.ringbuffer_size3i_(1);
  int zid_global = zid_in_buffer + md_.ringbuffer_origin3i_(2);
  if (zid_global > md_.ringbuffer_upbound3i_(2)) zid_global -= md_.ringbuffer_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline Eigen::Vector3i GridMap::infBufIdx2GlobalIdx(size_t address) {
  const int ringbuffer_xysize = md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / md_.ringbuffer_inf_size3i_(0);
  int xid_in_buffer = address % md_.ringbuffer_inf_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_inf_origin3i_(0);
  if (xid_global > md_.ringbuffer_inf_upbound3i_(0)) xid_global -= md_.ringbuffer_inf_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_inf_origin3i_(1);
  if (yid_global > md_.ringbuffer_inf_upbound3i_(1)) yid_global -= md_.ringbuffer_inf_size3i_(1);
  int zid_global = zid_in_buffer + md_.ringbuffer_inf_origin3i_(2);
  if (zid_global > md_.ringbuffer_inf_upbound3i_(2)) zid_global -= md_.ringbuffer_inf_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!pos.allFinite()) return 1;
  if (!isInBuf(pos)) return 0;

  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  return md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(pos))] > mp_.min_occupancy_log_ ? 1
                                                                                              : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!pos.allFinite()) return 1;
  if (!isInInfBuf(pos)) return 0;

  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  return int(md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}

inline bool GridMap::isInBuf(const Eigen::Vector3d& pos) {
  // The upper face is EXCLUSIVE: floor(upper / resolution) is the first
  // out-of-buffer index. A modulo lookup there aliases the opposite edge.
  return pos.allFinite() && (pos.array() >= md_.ringbuffer_lowbound3d_.array()).all() &&
         (pos.array() < md_.ringbuffer_upbound3d_.array()).all() && isInBuf(pos2GlobalIdx(pos));
}

inline bool GridMap::isInBuf(const Eigen::Vector3i& idx) {
  if (idx(0) < md_.ringbuffer_lowbound3i_(0) || idx(1) < md_.ringbuffer_lowbound3i_(1) ||
      idx(2) < md_.ringbuffer_lowbound3i_(2)) {
    return false;
  }
  if (idx(0) > md_.ringbuffer_upbound3i_(0) || idx(1) > md_.ringbuffer_upbound3i_(1) ||
      idx(2) > md_.ringbuffer_upbound3i_(2)) {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3d& pos) {
  return pos.allFinite() && (pos.array() >= md_.ringbuffer_inf_lowbound3d_.array()).all() &&
         (pos.array() < md_.ringbuffer_inf_upbound3d_.array()).all() &&
         isInInfBuf(pos2GlobalIdx(pos));
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3i& idx) {
  if (idx(0) < md_.ringbuffer_inf_lowbound3i_(0) || idx(1) < md_.ringbuffer_inf_lowbound3i_(1) ||
      idx(2) < md_.ringbuffer_inf_lowbound3i_(2)) {
    return false;
  }
  if (idx(0) > md_.ringbuffer_inf_upbound3i_(0) || idx(1) > md_.ringbuffer_inf_upbound3i_(1) ||
      idx(2) > md_.ringbuffer_inf_upbound3i_(2)) {
    return false;
  }
  return true;
}

inline Eigen::Vector3d GridMap::globalIdx2Pos(const Eigen::Vector3i& id)  // t ~ 0us
{
  return Eigen::Vector3d((id(0) + 0.5) * mp_.resolution_, (id(1) + 0.5) * mp_.resolution_,
                         (id(2) + 0.5) * mp_.resolution_);
}

inline Eigen::Vector3i GridMap::pos2GlobalIdx(const Eigen::Vector3d& pos) {
  return (pos * mp_.resolution_inv_)
      .array()
      .floor()
      .cast<int>();  // more than twice faster than std::floor()
}

inline double GridMap::getResolution() {
  return mp_.resolution_;
}

#endif
