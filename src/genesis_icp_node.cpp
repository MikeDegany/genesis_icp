// Copyright (c) 2026 Mike Degany <mike.degany@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "genesis_icp/laser_utils.hpp"
#include "genesis_icp/mapper_configure.hpp"
#include "genesis_icp/wire_scan_packet.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "karto_sdk/Karto.h"
#include "karto_sdk/Mapper.h"
#include "karto_sdk/Types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace genesis_icp
{

static karto::Pose2 transform_base_B_to_A(const karto::Pose2 & T_AB, const karto::Pose2 & p_B)
{
  const double c = std::cos(T_AB.GetHeading());
  const double s = std::sin(T_AB.GetHeading());
  const double x = c * p_B.GetX() - s * p_B.GetY() + T_AB.GetX();
  const double y = s * p_B.GetX() + c * p_B.GetY() + T_AB.GetY();
  return karto::Pose2(
    x, y,
    karto::math::NormalizeAngle(p_B.GetHeading() + T_AB.GetHeading()));
}

/** Grid of scan-B base seeds in match frame (A), centered on `center`, capped to `max_seeds`. */
static void build_match_seeds(
  const karto::Pose2 & center, bool multi_seed, double xy_extent_m, double xy_step_m,
  double yaw_extent_deg, double yaw_step_deg, size_t max_seeds, std::vector<karto::Pose2> & out)
{
  out.clear();
  if (!multi_seed || max_seeds < 1) {
    out.push_back(center);
    return;
  }

  auto half_steps = [](double extent, double step) -> int {
    if (step <= 0.0 || extent < 0.0) {
      return 0;
    }
    return static_cast<int>(std::floor((extent + 1e-9) / step));
  };

  int hx = half_steps(xy_extent_m, xy_step_m);
  int hy = half_steps(xy_extent_m, xy_step_m);
  const double yaw_ext_rad = karto::math::DegreesToRadians(yaw_extent_deg);
  const double yaw_step_rad = karto::math::DegreesToRadians(yaw_step_deg);
  int hyaw = (yaw_step_rad > 0.0) ? half_steps(yaw_ext_rad, yaw_step_rad) : 0;

  int nx = 2 * hx + 1;
  int ny = 2 * hy + 1;
  int nyaw = 2 * hyaw + 1;
  auto shrink_one = [&]() {
    if (nx >= ny && nx >= nyaw && nx > 1) {
      hx--;
      nx = 2 * hx + 1;
    } else if (ny >= nx && ny >= nyaw && ny > 1) {
      hy--;
      ny = 2 * hy + 1;
    } else if (nyaw > 1) {
      hyaw--;
      nyaw = 2 * hyaw + 1;
    }
  };
  while (static_cast<size_t>(nx) * static_cast<size_t>(ny) * static_cast<size_t>(nyaw) > max_seeds) {
    const int nx0 = nx;
    const int ny0 = ny;
    const int nyaw0 = nyaw;
    shrink_one();
    if (nx == nx0 && ny == ny0 && nyaw == nyaw0) {
      break;
    }
  }

  const double sx = xy_step_m > 0.0 ? xy_step_m : 0.0;
  const double sy = xy_step_m > 0.0 ? xy_step_m : 0.0;
  const double syaw = yaw_step_rad > 0.0 ? yaw_step_rad : 0.0;

  for (int ix = -hx; ix <= hx; ++ix) {
    for (int iy = -hy; iy <= hy; ++iy) {
      for (int iyaw = -hyaw; iyaw <= hyaw; ++iyaw) {
        const double ox = static_cast<double>(ix) * sx;
        const double oy = static_cast<double>(iy) * sy;
        const double oyaw = static_cast<double>(iyaw) * syaw;
        out.emplace_back(
          center.GetX() + ox, center.GetY() + oy,
          karto::math::NormalizeAngle(center.GetHeading() + oyaw));
      }
    }
  }
}

static tf2::Transform kartoPoseToTf(const karto::Pose2 & p)
{
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(p.GetX(), p.GetY(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.GetHeading());
  tf.setRotation(q);
  return tf;
}

static geometry_msgs::msg::TransformStamped laserMountFromPacket(
  const ScanPacket & pkt, const std::string & laser_frame_id)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = laser_frame_id;
  ts.transform.translation.x = pkt.laser_in_base_x;
  ts.transform.translation.y = pkt.laser_in_base_y;
  ts.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pkt.laser_in_base_yaw);
  ts.transform.rotation = tf2::toMsg(q);
  return ts;
}

static int tcp_listen_ipv4(int port)
{
  int s = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
  if (s < 0) {
    return -1;
  }
  int opt = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(s);
    return -1;
  }
  if (listen(s, 2) < 0) {
    close(s);
    return -1;
  }
  return s;
}

static void tcp_reader_thread(
  int listen_fd, std::mutex & mx, std::deque<ScanPacket> & buffer, size_t buffer_max,
  const rclcpp::Logger & logger, const std::string & label)
{
  while (rclcpp::ok()) {
    int c = accept(listen_fd, nullptr, nullptr);
    if (c < 0) {
      continue;
    }
    RCLCPP_INFO(logger, "%s: client connected", label.c_str());
    while (rclcpp::ok()) {
      auto opt = read_full_packet(c);
      if (!opt) {
        break;
      }
      std::lock_guard<std::mutex> lk(mx);
      buffer.push_back(*opt);
      while (buffer.size() > buffer_max) {
        buffer.pop_front();
      }
    }
    close(c);
    RCLCPP_WARN(logger, "%s: client disconnected", label.c_str());
  }
}

/** Pick scans with minimum |t_a - t_b| subject to |t_a - t_b| <= max_dt_sec (handles skewed bag stamps). */
static double shortest_angle_delta_rad(double from_yaw, double to_yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  double d = to_yaw - from_yaw;
  while (d > kPi) {
    d -= 2.0 * kPi;
  }
  while (d < -kPi) {
    d += 2.0 * kPi;
  }
  return d;
}

/** Exponential smoothing on planar transform (reduces publish jitter). alpha in (0,1]; higher = trust measurement more. */
static void smooth_tf2_transform(
  tf2::Transform & acc, bool & have_acc, const tf2::Transform & meas, double alpha)
{
  if (!have_acc || alpha >= 1.0) {
    acc = meas;
    have_acc = true;
    return;
  }
  if (alpha <= 0.0) {
    return;
  }
  const tf2::Vector3 t0 = acc.getOrigin();
  const tf2::Vector3 t1 = meas.getOrigin();
  acc.setOrigin(tf2::Vector3(
    (1.0 - alpha) * t0.x() + alpha * t1.x(), (1.0 - alpha) * t0.y() + alpha * t1.y(),
    (1.0 - alpha) * t0.z() + alpha * t1.z()));
  const double y0 = tf2::getYaw(acc.getRotation());
  const double y1 = tf2::getYaw(meas.getRotation());
  const double y = y0 + alpha * shortest_angle_delta_rad(y0, y1);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, y);
  acc.setRotation(q);
}

static bool select_best_scan_pair(
  const std::deque<ScanPacket> & buf_a, const std::deque<ScanPacket> & buf_b, double max_dt_sec,
  ScanPacket & out_a, ScanPacket & out_b)
{
  if (buf_a.empty() || buf_b.empty()) {
    return false;
  }
  const int64_t max_ns = static_cast<int64_t>(max_dt_sec * 1e9);
  int64_t best_abs = (std::numeric_limits<int64_t>::max)();
  bool found = false;
  for (const auto & pa : buf_a) {
    const int64_t ta = static_cast<int64_t>(pa.stamp_nsec);
    for (const auto & pb : buf_b) {
      const int64_t tb = static_cast<int64_t>(pb.stamp_nsec);
      const int64_t d = std::llabs(ta - tb);
      if (d <= max_ns && d < best_abs) {
        best_abs = d;
        out_a = pa;
        out_b = pb;
        found = true;
      }
    }
  }
  return found;
}

class GenesisIcpNode : public rclcpp::Node
{
public:
  GenesisIcpNode()
  : Node("genesis_icp")
  {
    use_socket_bridge_ = declare_parameter<bool>("use_socket_bridge", true);
    fusion_listen_port_robot_a_ = declare_parameter<int>("fusion_listen_port_robot_a", 4403);
    fusion_listen_port_robot_b_ = declare_parameter<int>("fusion_listen_port_robot_b", 4405);

    scan_topic_robot_a_ = declare_parameter<std::string>("scan_topic_robot_a", "JK3/sensors/lidar2d_0/scan");
    scan_topic_robot_b_ = declare_parameter<std::string>("scan_topic_robot_b", "JK5/sensors/lidar2d_0/scan");
    odom_frame_robot_a_ = declare_parameter<std::string>("odom_frame_robot_a", "JK3/odom");
    odom_frame_robot_b_ = declare_parameter<std::string>("odom_frame_robot_b", "JK5/odom");
    base_frame_robot_a_ = declare_parameter<std::string>("base_frame_robot_a", "JK3/base_link");
    base_frame_robot_b_ = declare_parameter<std::string>("base_frame_robot_b", "JK5/base_link");

    guess_dx_ = declare_parameter<double>("initial_guess_odom_b_in_a_x", 0.0);
    guess_dy_ = declare_parameter<double>("initial_guess_odom_b_in_a_y", 0.0);
    guess_dyaw_ = declare_parameter<double>("initial_guess_odom_b_in_a_yaw", 0.0);

    global_odom_frame_ = declare_parameter<std::string>("global_odom_frame", "global_odom");
    odom_child_robot_a_ = declare_parameter<std::string>("odom_child_frame_robot_a", "JK3/odom");
    odom_child_robot_b_ = declare_parameter<std::string>("odom_child_frame_robot_b", "JK5/odom");
    tf_static_topic_a_ = declare_parameter<std::string>("tf_static_topic_robot_a", "/JK3/tf_static");
    tf_static_topic_b_ = declare_parameter<std::string>("tf_static_topic_robot_b", "/JK5/tf_static");
    anchor_mode_ = declare_parameter<std::string>("anchor_mode", "robot_a");

    max_time_delta_sec_ = declare_parameter<double>("max_time_delta_sec", 10.0);
    scan_pair_buffer_size_ = declare_parameter<int>("scan_pair_buffer_size", 80);
    if (scan_pair_buffer_size_ < 4) {
      scan_pair_buffer_size_ = 4;
    }
    minimum_match_response_ = declare_parameter<double>("minimum_match_response", 0.05);
    max_laser_range_ = declare_parameter<double>("max_laser_range", 12.0);
    match_scan_do_penalize_ = declare_parameter<bool>("match_scan_do_penalize", false);
    relative_pose_smooth_alpha_ = declare_parameter<double>("relative_pose_smooth_alpha", 0.25);
    if (relative_pose_smooth_alpha_ < 0.0) {
      relative_pose_smooth_alpha_ = 0.0;
    } else if (relative_pose_smooth_alpha_ > 1.0) {
      relative_pose_smooth_alpha_ = 1.0;
    }

    use_odom_poses_for_match_ = declare_parameter<bool>("use_odom_poses_for_match", false);
    multi_seed_match_ = declare_parameter<bool>("multi_seed_match", false);
    multi_seed_xy_step_m_ = declare_parameter<double>("multi_seed_xy_step_m", 1.0);
    multi_seed_xy_extent_m_ = declare_parameter<double>("multi_seed_xy_extent_m", 1.0);
    multi_seed_yaw_step_deg_ = declare_parameter<double>("multi_seed_yaw_step_deg", 10.0);
    multi_seed_yaw_extent_deg_ = declare_parameter<double>("multi_seed_yaw_extent_deg", 20.0);
    multi_seed_max_seeds_ = declare_parameter<int>("multi_seed_max_seeds", 48);
    if (multi_seed_max_seeds_ < 1) {
      multi_seed_max_seeds_ = 1;
    }

    laser_id_robot_a_ = declare_parameter<std::string>("laser_id_robot_a", "genesis_robot_a_laser");
    laser_id_robot_b_ = declare_parameter<std::string>("laser_id_robot_b", "genesis_robot_b_laser");
    tf_buffer_cache_sec_ = declare_parameter<double>("tf_buffer_cache_sec", 60.0);

    mapper_ = std::make_unique<karto::Mapper>();
    mapper_params_applied_ = false;
    mapper_initialized_ = false;

    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();
    pub_tf_a_ = create_publisher<tf2_msgs::msg::TFMessage>(tf_static_topic_a_, qos);
    pub_tf_b_ = create_publisher<tf2_msgs::msg::TFMessage>(tf_static_topic_b_, qos);

    if (use_socket_bridge_) {
      int fa = tcp_listen_ipv4(fusion_listen_port_robot_a_);
      int fb = tcp_listen_ipv4(fusion_listen_port_robot_b_);
      if (fa < 0 || fb < 0) {
        RCLCPP_FATAL(get_logger(), "Failed to bind TCP listen ports %d / %d", fusion_listen_port_robot_a_,
          fusion_listen_port_robot_b_);
        throw std::runtime_error("tcp bind");
      }
      listen_fd_a_ = fa;
      listen_fd_b_ = fb;
      thread_a_ = std::thread(
        tcp_reader_thread, listen_fd_a_, std::ref(mutex_a_), std::ref(buffer_a_),
        static_cast<size_t>(scan_pair_buffer_size_), get_logger(), std::string("robot_a"));
      thread_b_ = std::thread(
        tcp_reader_thread, listen_fd_b_, std::ref(mutex_b_), std::ref(buffer_b_),
        static_cast<size_t>(scan_pair_buffer_size_), get_logger(), std::string("robot_b"));
      RCLCPP_INFO(
        get_logger(), "Socket bridge listening on ports %d (A) and %d (B)",
        fusion_listen_port_robot_a_, fusion_listen_port_robot_b_);
    } else {
      const double cache = tf_buffer_cache_sec_ > 0.0 ? tf_buffer_cache_sec_ : 60.0;
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(cache));
      rclcpp::QoS tf_dynamic_qos(rclcpp::KeepLast(200));
      tf_dynamic_qos.reliable();
      tf_dynamic_qos.durability(rclcpp::DurabilityPolicy::Volatile);
      rclcpp::QoS tf_static_qos(rclcpp::KeepLast(100));
      tf_static_qos.reliable();
      tf_static_qos.durability(rclcpp::DurabilityPolicy::Volatile);
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer_, this, true, tf_dynamic_qos, tf_static_qos);
      rclcpp::QoS scan_qos(rclcpp::KeepLast(50));
      scan_qos.reliable();
      scan_qos.durability(rclcpp::DurabilityPolicy::Volatile);
      sub_a_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_robot_a_, scan_qos,
        std::bind(&GenesisIcpNode::onScanA, this, std::placeholders::_1));
      sub_b_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_robot_b_, scan_qos,
        std::bind(&GenesisIcpNode::onScanB, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&GenesisIcpNode::onTimer, this));
  }

  ~GenesisIcpNode() override
  {
    if (listen_fd_a_ >= 0) {
      shutdown(listen_fd_a_, SHUT_RDWR);
      close(listen_fd_a_);
      listen_fd_a_ = -1;
    }
    if (listen_fd_b_ >= 0) {
      shutdown(listen_fd_b_, SHUT_RDWR);
      close(listen_fd_b_);
      listen_fd_b_ = -1;
    }
    if (thread_a_.joinable()) {
      thread_a_.join();
    }
    if (thread_b_.joinable()) {
      thread_b_.join();
    }
  }

private:
  bool get_base_in_odom(
    const std::string & base_frame, const std::string & odom_frame,
    const rclcpp::Time & /*t*/, double & x, double & y, double & yaw)
  {
    try {
      const auto odom_pose = tf_buffer_->lookupTransform(
        odom_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      x = odom_pose.transform.translation.x;
      y = odom_pose.transform.translation.y;
      yaw = tf2::getYaw(odom_pose.transform.rotation);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool fill_packet_from_ros(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan, const std::string & base_frame,
    const std::string & odom_frame, ScanPacket & out)
  {
    rclcpp::Time t(scan->header.stamp);
    double bx = 0, by = 0, byaw = 0;
    if (!get_base_in_odom(base_frame, odom_frame, t, bx, by, byaw)) {
      return false;
    }
    double lx = 0, ly = 0, lyaw = 0;
    try {
      const auto in_base = tf_buffer_->lookupTransform(
        base_frame, scan->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
      lx = in_base.transform.translation.x;
      ly = in_base.transform.translation.y;
      lyaw = tf2::getYaw(in_base.transform.rotation);
    } catch (const tf2::TransformException &) {
      return false;
    }
    out.stamp_nsec = static_cast<uint64_t>(t.nanoseconds());
    out.frame_id = scan->header.frame_id;
    out.ranges.assign(scan->ranges.begin(), scan->ranges.end());
    out.angle_min = scan->angle_min;
    out.angle_max = scan->angle_max;
    out.angle_increment = scan->angle_increment;
    out.range_min = scan->range_min;
    out.range_max = scan->range_max;
    out.base_x = bx;
    out.base_y = by;
    out.base_yaw = byaw;
    out.laser_in_base_x = lx;
    out.laser_in_base_y = ly;
    out.laser_in_base_yaw = lyaw;
    return true;
  }

  void onScanA(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    ScanPacket p;
    if (!fill_packet_from_ros(scan, base_frame_robot_a_, odom_frame_robot_a_, p)) {
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_a_);
    buffer_a_.push_back(std::move(p));
    while (buffer_a_.size() > static_cast<size_t>(scan_pair_buffer_size_)) {
      buffer_a_.pop_front();
    }
  }

  void onScanB(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    ScanPacket p;
    if (!fill_packet_from_ros(scan, base_frame_robot_b_, odom_frame_robot_b_, p)) {
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_b_);
    buffer_b_.push_back(std::move(p));
    while (buffer_b_.size() > static_cast<size_t>(scan_pair_buffer_size_)) {
      buffer_b_.pop_front();
    }
  }

  void ensure_mapper_ready()
  {
    if (!mapper_params_applied_) {
      configure_karto_mapper(shared_from_this(), mapper_.get());
      mapper_params_applied_ = true;
    }
    if (!mapper_initialized_) {
      mapper_->Initialize(static_cast<kt_double>(max_laser_range_));
      mapper_initialized_ = true;
    }
  }

  laser_utils::LaserMetadata & get_laser_meta(
    const std::string & key, const ScanPacket & pkt,
    const sensor_msgs::msg::LaserScan & scan_msg, laser_utils::LaserAssistant & asst)
  {
    if (lasers_.find(key) == lasers_.end()) {
      auto sm = std::make_shared<sensor_msgs::msg::LaserScan>(scan_msg);
      geometry_msgs::msg::TransformStamped mount = laserMountFromPacket(pkt, scan_msg.header.frame_id);
      mount.header.stamp = scan_msg.header.stamp;
      lasers_[key] = asst.toLaserMetadata(*sm, mount);
      dataset_.Add(lasers_[key].getLaser(), true);
    }
    return lasers_[key];
  }

  void onTimer()
  {
    ScanPacket a;
    ScanPacket b;
    {
      std::lock_guard<std::mutex> la(mutex_a_);
      std::lock_guard<std::mutex> lb(mutex_b_);
      if (!select_best_scan_pair(buffer_a_, buffer_b_, max_time_delta_sec_, a, b)) {
        return;
      }
    }

    ensure_mapper_ready();

    sensor_msgs::msg::LaserScan scan_a = to_laser_scan(a);
    scan_a.header.frame_id = laser_id_robot_a_;
    sensor_msgs::msg::LaserScan scan_b = to_laser_scan(b);
    scan_b.header.frame_id = laser_id_robot_b_;

    if (!tf_buffer_) {
      const double cache = tf_buffer_cache_sec_ > 0.0 ? tf_buffer_cache_sec_ : 60.0;
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(cache));
    }
    laser_utils::LaserAssistant asst(shared_from_this(), tf_buffer_.get(), "map");

    auto & meta_a = get_laser_meta("a", a, scan_a, asst);
    auto & meta_b = get_laser_meta("b", b, scan_b, asst);

    karto::LaserRangeFinder * laser_a = meta_a.getLaser();
    karto::LaserRangeFinder * laser_b = meta_b.getLaser();

    const std::vector<double> readings_a = laser_utils::scanToReadings(scan_a, meta_a.isInverted());
    const std::vector<double> readings_b = laser_utils::scanToReadings(scan_b, meta_b.isInverted());

    karto::Pose2 base_a;
    karto::Pose2 base_b_in_B;
    if (use_odom_poses_for_match_) {
      base_a = karto::Pose2(a.base_x, a.base_y, a.base_yaw);
      base_b_in_B = karto::Pose2(b.base_x, b.base_y, b.base_yaw);
    } else {
      base_a = karto::Pose2(0.0, 0.0, 0.0);
      base_b_in_B = karto::Pose2(0.0, 0.0, 0.0);
    }

    karto::LocalizedRangeScan * ra = new karto::LocalizedRangeScan(laser_a->GetName(), readings_a);
    ra->SetOdometricPose(base_a);
    ra->SetCorrectedPose(base_a);

    const karto::Pose2 T_guess(guess_dx_, guess_dy_, guess_dyaw_);
    const karto::Pose2 base_b_in_A_center = transform_base_B_to_A(T_guess, base_b_in_B);

    std::vector<karto::Pose2> seeds;
    build_match_seeds(
      base_b_in_A_center, multi_seed_match_, multi_seed_xy_extent_m_, multi_seed_xy_step_m_,
      multi_seed_yaw_extent_deg_, multi_seed_yaw_step_deg_,
      static_cast<size_t>(multi_seed_max_seeds_), seeds);

    karto::LocalizedRangeScan * rb = new karto::LocalizedRangeScan(laser_b->GetName(), readings_b);

    karto::LocalizedRangeScan * rb_sensor_only = new karto::LocalizedRangeScan(laser_b->GetName(), readings_b);
    rb_sensor_only->SetOdometricPose(base_b_in_B);
    rb_sensor_only->SetCorrectedPose(base_b_in_B);
    const karto::Pose2 sensor_b_in_B = rb_sensor_only->GetSensorPose();
    delete rb_sensor_only;

    karto::LocalizedRangeScanVector base_chain;
    base_chain.push_back(ra);

    kt_double best_response = -1.0;
    karto::Pose2 best_mean(0.0, 0.0, 0.0);
    for (const karto::Pose2 & seed : seeds) {
      rb->SetOdometricPose(seed);
      rb->SetCorrectedPose(seed);
      karto::Pose2 trial_mean;
      karto::Matrix3 trial_cov;
      const kt_double response = mapper_->GetSequentialScanMatcher()->MatchScan(
        rb, base_chain, trial_mean, trial_cov, static_cast<kt_bool>(match_scan_do_penalize_), true);
      if (response > best_response) {
        best_response = response;
        best_mean = trial_mean;
      }
    }
    const kt_double response = best_response;

    if (response < static_cast<kt_double>(minimum_match_response_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Scan match response %.4f below threshold %.4f", static_cast<double>(response),
        minimum_match_response_);
      delete ra;
      delete rb;
      return;
    }

    const tf2::Transform T_sensor_B_in_A = kartoPoseToTf(best_mean);
    const tf2::Transform T_sensor_B_in_B = kartoPoseToTf(sensor_b_in_B);
    // Each robot's odom is taken coincident with base at startup; packet base_* is ignored when
    // use_odom_poses_for_match is false. Then T_odomA_odomB is still B_odom -> A_odom from the
    // matched laser pose and B's laser–base extrinsic.
    const tf2::Transform T_odomA_odomB_raw = T_sensor_B_in_A * T_sensor_B_in_B.inverse();

    const double guess_base_sep_m =
      std::hypot(base_b_in_A_center.GetX() - base_a.GetX(), base_b_in_A_center.GetY() - base_a.GetY());

    tf2::Transform T_publish = T_odomA_odomB_raw;
    if (relative_pose_smooth_alpha_ > 0.0) {
      smooth_tf2_transform(
        T_rel_smoothed_, have_rel_pose_smooth_, T_odomA_odomB_raw, relative_pose_smooth_alpha_);
      T_publish = T_rel_smoothed_;
    } else {
      have_rel_pose_smooth_ = false;
    }

    const tf2::Vector3 t_b_in_a = T_publish.getOrigin();
    const double dist_m = std::hypot(t_b_in_a.x(), t_b_in_a.y());
    const double dyaw_rad = tf2::getYaw(T_publish.getRotation());
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const tf2::Vector3 raw_o = T_odomA_odomB_raw.getOrigin();
    const double raw_dist = std::hypot(raw_o.x(), raw_o.y());
    if (use_odom_poses_for_match_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Match ok (response=%.4f): odom_B in odom_A dist=%.3f m (raw=%.3f) dxy=(%.3f, %.3f) dyaw=%.2f deg | "
        "guess_base_sep=%.3f m (set initial_guess_* + widen correlation_search_space_dimension if wrong)",
        static_cast<double>(response), dist_m, raw_dist, t_b_in_a.x(), t_b_in_a.y(), dyaw_rad * kRadToDeg,
        guess_base_sep_m);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Match ok (response=%.4f): odom_B in odom_A dist=%.3f m (raw=%.3f) dxy=(%.3f, %.3f) dyaw=%.2f deg | "
        "odom_free seed_xy_yaw=(%.3f, %.3f, %.2f deg) seeds_tried=%zu "
        "(initial_guess_* + multi_seed_* / correlation_search_space_dimension tune overlap)",
        static_cast<double>(response), dist_m, raw_dist, t_b_in_a.x(), t_b_in_a.y(), dyaw_rad * kRadToDeg,
        base_b_in_A_center.GetX(), base_b_in_A_center.GetY(),
        base_b_in_A_center.GetHeading() * kRadToDeg, seeds.size());
    }

    publish_static_transforms(T_publish);

    delete ra;
    delete rb;
  }

  void publish_static_transforms(const tf2::Transform & T_odomA_odomB)
  {
    const tf2::Transform I;
    tf2::Transform T_global_odomA;
    tf2::Transform T_global_odomB;

    if (anchor_mode_ == "robot_a") {
      T_global_odomA = I;
      T_global_odomB = T_odomA_odomB.inverse();
    } else if (anchor_mode_ == "robot_b") {
      T_global_odomA = T_odomA_odomB;
      T_global_odomB = I;
    } else if (anchor_mode_ == "midpoint") {
      const tf2::Vector3 t = T_odomA_odomB.getOrigin();
      const tf2::Vector3 t_mid = t * 0.5;
      tf2::Transform shift;
      shift.setIdentity();
      shift.setOrigin(-t_mid);
      T_global_odomA = shift;
      tf2::Transform T_B_in_A = T_odomA_odomB;
      T_global_odomB = shift * T_B_in_A;
    } else {
      RCLCPP_WARN(get_logger(), "Unknown anchor_mode '%s', using robot_a", anchor_mode_.c_str());
      T_global_odomA = I;
      T_global_odomB = T_odomA_odomB.inverse();
    }

    geometry_msgs::msg::TransformStamped ta;
    ta.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    ta.header.frame_id = global_odom_frame_;
    ta.child_frame_id = odom_child_robot_a_;
    tf2::toMsg(T_global_odomA, ta.transform);

    geometry_msgs::msg::TransformStamped tb;
    tb.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    tb.header.frame_id = global_odom_frame_;
    tb.child_frame_id = odom_child_robot_b_;
    tf2::toMsg(T_global_odomB, tb.transform);

    tf2_msgs::msg::TFMessage msg_a;
    msg_a.transforms.push_back(ta);
    tf2_msgs::msg::TFMessage msg_b;
    msg_b.transforms.push_back(tb);
    pub_tf_a_->publish(msg_a);
    pub_tf_b_->publish(msg_b);
  }

  bool use_socket_bridge_{true};
  int fusion_listen_port_robot_a_{4403};
  int fusion_listen_port_robot_b_{4405};
  std::string scan_topic_robot_a_;
  std::string scan_topic_robot_b_;
  std::string odom_frame_robot_a_;
  std::string odom_frame_robot_b_;
  std::string base_frame_robot_a_;
  std::string base_frame_robot_b_;
  double guess_dx_{0}, guess_dy_{0}, guess_dyaw_{0};
  std::string global_odom_frame_;
  std::string odom_child_robot_a_;
  std::string odom_child_robot_b_;
  std::string tf_static_topic_a_;
  std::string tf_static_topic_b_;
  std::string anchor_mode_;
  double max_time_delta_sec_{10.0};
  int scan_pair_buffer_size_{80};
  double minimum_match_response_{0.05};
  double max_laser_range_{12.0};
  bool match_scan_do_penalize_{false};
  double relative_pose_smooth_alpha_{0.25};
  bool use_odom_poses_for_match_{false};
  bool multi_seed_match_{false};
  double multi_seed_xy_step_m_{1.0};
  double multi_seed_xy_extent_m_{1.0};
  double multi_seed_yaw_step_deg_{10.0};
  double multi_seed_yaw_extent_deg_{20.0};
  int multi_seed_max_seeds_{48};
  bool have_rel_pose_smooth_{false};
  tf2::Transform T_rel_smoothed_;
  std::string laser_id_robot_a_;
  std::string laser_id_robot_b_;
  double tf_buffer_cache_sec_{60.0};

  std::unique_ptr<karto::Mapper> mapper_;
  bool mapper_params_applied_{false};
  bool mapper_initialized_{false};

  std::mutex mutex_a_, mutex_b_;
  std::deque<ScanPacket> buffer_a_, buffer_b_;
  int listen_fd_a_{-1};
  int listen_fd_b_{-1};
  std::thread thread_a_;
  std::thread thread_b_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_a_, sub_b_;

  std::unordered_map<std::string, laser_utils::LaserMetadata> lasers_;
  karto::Dataset dataset_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_a_, pub_tf_b_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace genesis_icp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<genesis_icp::GenesisIcpNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
