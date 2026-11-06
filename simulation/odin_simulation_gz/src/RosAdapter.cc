#include "odin_simulation_gz/Contract.hh"
#include <bit>
#include <cmath>
#include <cstring>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>

class Adapter : public rclcpp::Node {
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  std::string prefix;
  double hfov, vfov;
  int64_t imuPeriod = 2500000, nextImu = 0, lastImu = -1;
  // Destroy transport / callbacks before publishers during normal shutdown.
  gz::transport::Node transport;
  static builtin_interfaces::msg::Time stamp(const gz::msgs::Header &h) {
    builtin_interfaces::msg::Time t;
    t.sec = h.stamp().sec();
    t.nanosec = h.stamp().nsec();
    return t;
  }
  void OnCloud(const gz::msgs::PointCloudPacked &raw) {
    const auto count = static_cast<size_t>(raw.width()) * raw.height();
    if (raw.is_bigendian() || raw.point_step() != 20 ||
        raw.row_step() != raw.width() * 20 || raw.data().size() != count * 20 ||
        raw.field_size() != 5)
      return;
    const char *names[] = {"x", "y", "z", "hit", "ray_index"};
    for (int k = 0; k < 5; ++k)
      if (raw.field(k).name() != names[k] || raw.field(k).offset() != k * 4)
        return;
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp(raw.header());
    msg.header.frame_id = prefix + "lidar";
    msg.width = raw.width();
    msg.height = raw.height();
    msg.point_step = odin_sim::kPointStep;
    msg.row_step = msg.width * msg.point_step;
    msg.is_dense = false;
    msg.is_bigendian = false;
    const char *fields[] = {"x",         "y",          "z",
                            "intensity", "confidence", "offset_time"};
    const uint32_t offsets[] = {0, 4, 8, 12, 13, 15};
    const uint8_t types[] = {7, 7, 7, 2, 4, 7};
    for (int i = 0; i < 6; ++i) {
      sensor_msgs::msg::PointField f;
      f.name = fields[i];
      f.offset = offsets[i];
      f.datatype = types[i];
      f.count = 1;
      msg.fields.push_back(f);
    }
    msg.data.assign(count * msg.point_step, 0);
    for (size_t i = 0; i < count; ++i) {
      const char *p = raw.data().data() + i * 20;
      float xyz[3];
      int32_t hit;
      uint32_t index;
      std::memcpy(xyz, p, 12);
      std::memcpy(&hit, p + 12, 4);
      std::memcpy(&index, p + 16, 4);
      if (index >= count) {
        RCLCPP_ERROR(get_logger(), "Invalid ray index");
        return;
      }
      odin_sim::packPoint(msg.data.data() + index * msg.point_step, xyz[0],
                          xyz[1], xyz[2], hit != 0);
    }
    cloud->publish(std::move(msg));
  }
  void OnImage(const gz::msgs::Image &raw) {
    const bool rgb = raw.pixel_format_type() == gz::msgs::RGB_INT8;
    const bool bgr = raw.pixel_format_type() == gz::msgs::BGR_INT8;
    if ((!rgb && !bgr) || raw.step() < raw.width() * 3 ||
        raw.data().size() < static_cast<size_t>(raw.step()) * raw.height())
      return;
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp(raw.header());
    msg.header.frame_id = prefix + "camera_0";
    msg.width = raw.width();
    msg.height = raw.height();
    msg.encoding = "bgr8";
    msg.step = msg.width * 3;
    msg.is_bigendian = false;
    msg.data.resize(msg.step * msg.height);
    for (unsigned v = 0; v < msg.height; ++v)
      for (unsigned u = 0; u < msg.width; ++u) {
        const auto *in = reinterpret_cast<const uint8_t *>(raw.data().data()) +
                         v * raw.step() + u * 3;
        auto *out = msg.data.data() + v * msg.step + u * 3;
        out[0] = in[rgb ? 2 : 0];
        out[1] = in[1];
        out[2] = in[rgb ? 0 : 2];
      }
    sensor_msgs::msg::CameraInfo ci;
    ci.header = msg.header;
    ci.width = msg.width;
    ci.height = msg.height;
    ci.distortion_model = "plumb_bob";
    ci.d.assign(5, 0.0);
    const double fx = msg.width / (2 * std::tan(hfov / 2)),
                 fy = msg.height / (2 * std::tan(vfov / 2));
    const double cx = (msg.width - 1) * .5, cy = (msg.height - 1) * .5;
    ci.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    ci.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    ci.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
    image->publish(std::move(msg));
    info->publish(std::move(ci));
  }
  void OnImu(const gz::msgs::IMU &raw) {
    const int64_t now =
        raw.header().stamp().sec() * 1000000000LL + raw.header().stamp().nsec();
    if (now < lastImu)
      nextImu = now;
    lastImu = now;
    if (now < nextImu)
      return;
    // Sample genuine physics-step data, not interpolated or re-stamped samples.
    // Avoid the millisecond period truncation in older gz-sensors 8 builds.
    nextImu = now + imuPeriod;
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp(raw.header());
    msg.header.frame_id = prefix + "imu";
    msg.orientation.w = 1;
    msg.orientation_covariance[0] = -1;
    msg.angular_velocity.x = raw.angular_velocity().x();
    msg.angular_velocity.y = raw.angular_velocity().y();
    msg.angular_velocity.z = raw.angular_velocity().z();
    msg.linear_acceleration.x = raw.linear_acceleration().x();
    msg.linear_acceleration.y = raw.linear_acceleration().y();
    msg.linear_acceleration.z = raw.linear_acceleration().z();
    imu->publish(std::move(msg));
  }

public:
  Adapter() : Node("odin_ros_adapter") {
    static_assert(std::endian::native == std::endian::little);
    prefix = declare_parameter<std::string>("frame_prefix", "");
    auto base = declare_parameter<std::string>("gz_prefix", "/odin_sim/odin1");
    hfov = declare_parameter<double>("rgb_hfov", 129.0) * M_PI / 180;
    vfov = declare_parameter<double>("rgb_vfov", 104.0) * M_PI / 180;
    const double imuRate = declare_parameter<double>("imu_rate", 400.0);
    if (!std::isfinite(imuRate) || imuRate <= 0 || imuRate > 1000)
      throw std::runtime_error("Invalid IMU rate");
    imuPeriod = std::llround(1e9 / imuRate);
    if (!std::isfinite(hfov) || !std::isfinite(vfov) || hfov <= 0 ||
        vfov <= 0 || hfov >= M_PI || vfov >= M_PI)
      throw std::runtime_error("Invalid RGB FOV");
    auto qos = rclcpp::QoS(5).reliable().durability_volatile();
    cloud = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_raw", qos);
    image = create_publisher<sensor_msgs::msg::Image>("image", qos);
    info = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    imu = create_publisher<sensor_msgs::msg::Imu>(
        "imu", rclcpp::QoS(4000).reliable().durability_volatile());
    if (!transport.Subscribe(base + "/depth", &Adapter::OnCloud, this) ||
        !transport.Subscribe(base + "/image", &Adapter::OnImage, this) ||
        !transport.Subscribe(base + "/imu", &Adapter::OnImu, this))
      throw std::runtime_error("Gazebo subscription failed");
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Adapter>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
