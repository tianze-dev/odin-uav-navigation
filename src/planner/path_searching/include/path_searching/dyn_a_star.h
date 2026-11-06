#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <plan_env/grid_map.h>

#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <limits>
#include <rclcpp/rclcpp.hpp>

constexpr double inf = std::numeric_limits<double>::infinity();
struct GridNode;
typedef GridNode* GridNodePtr;

enum ASTAR_RET { SUCCESS, INIT_ERR, SEARCH_ERR };

struct GridNode {
  enum enum_state { OPENSET = 1, CLOSEDSET = 2, UNDEFINED = 3 };

  int rounds{0};  // Distinguish every call
  enum enum_state state { UNDEFINED };
  Eigen::Vector3i index;

  double gScore{inf}, fScore{inf};
  GridNodePtr cameFrom{NULL};
};

class NodeComparator {
 public:
  bool operator()(GridNodePtr node1, GridNodePtr node2) {
    return node1->fScore > node2->fScore;
  }
};

class AStar {
 private:
  GridMap::Ptr grid_map_;

  inline void coord2gridIndexFast(const double x, const double y, const double z, int& id_x,
                                  int& id_y, int& id_z);

  double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
  double getManhHeu(GridNodePtr node1, GridNodePtr node2);
  double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
  inline double getHeu(GridNodePtr node1, GridNodePtr node2);

  bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt,
                                             const Eigen::Vector3d end_pt,
                                             Eigen::Vector3i& start_idx, Eigen::Vector3i& end_idx);

  inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i& index) const;
  inline bool Coord2Index(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) const;

  // bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

  inline int checkOccupancy(const Eigen::Vector3d& pos) {
    if (!grid_map_->isCollisionFree(pos)) return 1;
    if (clearance_ > 0 && (pos-query_start_).norm()>.45 && (pos-query_goal_).norm()>.45)
      for (int axis=0;axis<3;++axis)
        for (int sign : {-1,1}) {
          Eigen::Vector3d q=pos;q[axis]+=sign*clearance_;
          if (!grid_map_->isCollisionFree(q)) return 1;
        }
    return 0;
  }

  std::vector<GridNodePtr> retrievePath(GridNodePtr current);

  double step_size_, inv_step_size_;
  Eigen::Vector3d center_;
  Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
  const double tie_breaker_ = 1.0 + 1.0 / 10000;

  std::vector<GridNodePtr> gridPath_;

  GridNodePtr*** GridNodeMap_;
  struct QueueEntry {double f, g; GridNodePtr node;};
  struct QueueCompare {bool operator()(const QueueEntry& a,const QueueEntry& b) const {return a.f>b.f;}};
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> openSet_;
  double clearance_{0};
  Eigen::Vector3d query_start_, query_goal_;

  int rounds_{0};
  std::chrono::steady_clock::time_point deadline_{std::chrono::steady_clock::time_point::max()};

 public:
  typedef std::shared_ptr<AStar> Ptr;

  AStar(){};
  ~AStar();

  void setDeadline(std::chrono::steady_clock::time_point t) {
    deadline_ = t;
  }
  void setClearance(double value) { clearance_=value; }
  void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);
  void setEnvironment(GridMap::Ptr map) { grid_map_ = std::move(map); }

  ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

  std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2) {
  return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i& index) const {
  return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) const {
  idx =
      ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).array().floor().cast<int>().matrix() + CENTER_IDX_;

  if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) ||
      idx(2) < 0 || idx(2) >= POOL_SIZE_(2)) {
    RCLCPP_ERROR(rclcpp::get_logger("AStar"), "Ran out of pool, index=%d %d %d", idx(0), idx(1),
                 idx(2));
    return false;
  }

  return true;
};

#endif
