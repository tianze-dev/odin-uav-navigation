#include "Utils.hh"
#include "odin_simulation_gz/Contract.hh"
#include <gz/math/Matrix4.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <limits>
#include <set>
#include <vector>
namespace odin_sim {
class DepthSensor : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate,
                    public gz::sim::ISystemPostUpdate,
                    public gz::sim::ISystemReset {
  gz::sim::Entity entity = 0;
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  unsigned width = 256, height = 192;
  double rate = 10, hfov = 2 * M_PI / 3, vfov = M_PI / 2;
  int64_t period = 100000000, next = 0, last = -1;
  bool ready = false, removed = false;
  std::string frame;
  std::vector<rgl_mat3x4f> rays;
  std::vector<uint8_t> buffer;
  rgl_node_t raysNode = nullptr, rangeNode = nullptr, poseNode = nullptr,
             traceNode = nullptr, localNode = nullptr, formatNode = nullptr;
  std::vector<std::pair<rgl_node_t, rgl_node_t>> edges;
  bool ok(rgl_status_t status) { return CheckRGL(status); }
  bool connect(rgl_node_t a, rgl_node_t b) {
    if (!ok(rgl_graph_node_add_child(a, b)))
      return false;
    edges.emplace_back(a, b);
    return true;
  }
  void destroy() {
    ready = false;
    std::set<rgl_node_t> alive{raysNode,  rangeNode, poseNode,
                               traceNode, localNode, formatNode};
    alive.erase(nullptr);
    while (!alive.empty()) {
      auto root = *alive.begin();
      std::vector<rgl_node_t> pending{root};
      alive.erase(root);
      while (!pending.empty()) {
        auto n = pending.back();
        pending.pop_back();
        for (auto [a, b] : edges) {
          auto other = a == n ? b : b == n ? a : nullptr;
          if (other && alive.erase(other))
            pending.push_back(other);
        }
      }
      ok(rgl_graph_destroy(root));
    }
    raysNode = rangeNode = poseNode = traceNode = localNode = formatNode =
        nullptr;
    edges.clear();
  }

public:
  ~DepthSensor() override { destroy(); }
  void Configure(const gz::sim::Entity &e,
                 const std::shared_ptr<const sdf::Element> &s,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override {
    entity = e;
    try {
      width = s->Get<unsigned>("width");
      height = s->Get<unsigned>("height");
      rate = s->Get<double>("update_rate");
      hfov = s->Get<double>("horizontal_fov");
      vfov = s->Get<double>("vertical_fov");
      frame = s->Get<std::string>("frame");
      const auto near = s->Get<double>("range_min"),
                 far = s->Get<double>("range_max");
      if (width < 2 || height < 2 || uint64_t(width) * height > 1000000 ||
          !std::isfinite(rate) || rate <= 0 || rate > 1000 ||
          !(hfov > 0 && hfov < M_PI) || !(vfov > 0 && vfov < M_PI) ||
          !std::isfinite(near) || !std::isfinite(far) || near < 0 ||
          far <= near)
        throw std::runtime_error("Invalid Odin depth parameters");
      period = std::llround(1e9 / rate);
      rays.reserve(width * height);
      for (unsigned v = 0; v < height; v++)
        for (unsigned u = 0; u < width; u++) {
          auto d = ray(u, v, width, height, hfov, vfov);
          gz::math::Quaterniond q(0, std::acos(d[2]), std::atan2(d[1], d[0]));
          gz::math::Matrix4d m(q);
          rgl_mat3x4f r{};
          for (int a = 0; a < 3; a++)
            for (int b = 0; b < 4; b++)
              r.value[a][b] = m(a, b);
          rays.push_back(r);
        }
      rgl_mat3x4f identity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
      rgl_vec2f ranges = {float(near), float(far)};
      const rgl_field_t fields[] = {
          RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_IS_HIT_I32, RGL_FIELD_RAY_IDX_U32};
      if (!ok(rgl_node_rays_from_mat3x4f(&raysNode, rays.data(),
                                         rays.size())) ||
          !ok(rgl_node_rays_set_range(&rangeNode, &ranges, 1)) ||
          !ok(rgl_node_rays_transform(&poseNode, &identity)) ||
          !ok(rgl_node_raytrace(&traceNode, nullptr)) ||
          !ok(rgl_node_points_transform(&localNode, &identity)) ||
          !ok(rgl_node_points_format(&formatNode, fields, 3)))
        throw std::runtime_error("RGL graph creation failed");
      if (!connect(raysNode, rangeNode) || !connect(rangeNode, poseNode) ||
          !connect(poseNode, traceNode) || !connect(traceNode, localNode) ||
          !connect(localNode, formatNode))
        throw std::runtime_error("RGL graph connection failed");
      pub = node.Advertise<gz::msgs::PointCloudPacked>(
          s->Get<std::string>("topic"));
      if (!pub)
        throw std::runtime_error("Invalid topic");
      buffer.resize(width * height * 20);
      ready = true;
      gzmsg << "Odin depth ready: " << width << "x" << height << " @ " << rate
            << " Hz\n";
    } catch (const std::exception &ex) {
      gzerr << ex.what() << "\n";
      destroy();
    }
  }
  void Reset(const gz::sim::UpdateInfo &,
             gz::sim::EntityComponentManager &) override {
    next = 0;
    last = -1;
  }
  void PostUpdate(const gz::sim::UpdateInfo &,
                  const gz::sim::EntityComponentManager &ecm) override {
    ecm.EachRemoved<>([this](const gz::sim::Entity &e) {
      if (e == entity)
        removed = true;
      return true;
    });
  }
  void PreUpdate(const gz::sim::UpdateInfo &i,
                 gz::sim::EntityComponentManager &ecm) override {
    if (removed) {
      destroy();
      removed = false;
      return;
    }
    if (!ready || i.paused)
      return;
    int64_t now =
        std::chrono::duration_cast<std::chrono::nanoseconds>(i.simTime).count();
    if (now < last)
      next = now;
    last = now;
    if (now < next)
      return;
    next = now + period;
    auto world = FindWorldPose(entity, ecm);
    auto a = IgnPose3dToRglMatrix(world),
         b = IgnPose3dToRglMatrix(world.Inverse());
    if (!ok(rgl_node_rays_transform(&poseNode, &a)) ||
        !ok(rgl_node_points_transform(&localNode, &b)) ||
        !ok(rgl_graph_run(traceNode)))
      return;
    int32_t count = 0, size = 0;
    if (!ok(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT,
                                      &count, &size)))
      return;
    if (count != int(width * height) || size != 20) {
      gzerr << "Odin organized ray layout mismatch\n";
      return;
    }
    if (!ok(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT,
                                      buffer.data())))
      return;
    gz::msgs::PointCloudPacked msg;
    *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(i.simTime);
    auto h = msg.mutable_header()->add_data();
    h->set_key("frame_id");
    h->add_value(frame);
    msg.set_width(width);
    msg.set_height(height);
    msg.set_point_step(20);
    msg.set_row_step(width * 20);
    msg.set_is_bigendian(false);
    msg.set_is_dense(false);
    const char *names[] = {"x", "y", "z", "hit", "ray_index"};
    for (unsigned k = 0; k < 5; k++) {
      auto f = msg.add_field();
      f->set_name(names[k]);
      f->set_offset(k * 4);
      f->set_count(1);
      f->set_datatype(k < 3    ? gz::msgs::PointCloudPacked::Field::FLOAT32
                      : k == 3 ? gz::msgs::PointCloudPacked::Field::INT32
                               : gz::msgs::PointCloudPacked::Field::UINT32);
    }
    msg.set_data(buffer.data(), buffer.size());
    pub.Publish(msg);
  }
};
} // namespace odin_sim
GZ_ADD_PLUGIN(odin_sim::DepthSensor, gz::sim::System,
              odin_sim::DepthSensor::ISystemConfigure,
              odin_sim::DepthSensor::ISystemPreUpdate,
              odin_sim::DepthSensor::ISystemPostUpdate,
              odin_sim::DepthSensor::ISystemReset)
