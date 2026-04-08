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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "genesis_icp/wire_scan_packet.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <arpa/inet.h>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std::chrono_literals;

/** TF2 frame IDs must not have a leading slash (ROS 2); scans sometimes still use one. */
static std::string strip_tf_leading_slash(std::string id)
{
  while (!id.empty() && id.front() == '/') {
    id.erase(0, 1);
  }
  return id;
}

static int connect_tcp(const std::string & host, int port)
{
  struct addrinfo hints;
  struct addrinfo * res = nullptr;
  memset(&hints, 0, sizeof(hints));
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_family = AF_UNSPEC;
  std::string port_str = std::to_string(port);
  if (getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0 || !res) {
    return -1;
  }
  int fd = -1;
  for (struct addrinfo * p = res; p; p = p->ai_next) {
    fd = static_cast<int>(socket(p->ai_family, p->ai_socktype, p->ai_protocol));
    if (fd < 0) {
      continue;
    }
    if (connect(fd, p->ai_addr, p->ai_addrlen) == 0) {
      break;
    }
    close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  return fd;
}

static bool get_base_in_odom(
  tf2_ros::Buffer & tf, const std::string & base_frame, const std::string & odom_frame,
  const rclcpp::Time & /*t*/, double & x, double & y, double & yaw, double timeout_sec)
{
  try {
    const auto odom_pose = tf.lookupTransform(
      strip_tf_leading_slash(odom_frame), strip_tf_leading_slash(base_frame), tf2::TimePointZero,
      tf2::durationFromSec(timeout_sec));
    x = odom_pose.transform.translation.x;
    y = odom_pose.transform.translation.y;
    yaw = tf2::getYaw(odom_pose.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

static bool get_laser_in_base_direct(
  tf2_ros::Buffer & tf, const std::string & base_frame, const std::string & laser_frame,
  double & x, double & y, double & yaw, double timeout_sec)
{
  try {
    const auto in_base = tf.lookupTransform(
      strip_tf_leading_slash(base_frame), strip_tf_leading_slash(laser_frame), tf2::TimePointZero,
      tf2::durationFromSec(timeout_sec));
    x = in_base.transform.translation.x;
    y = in_base.transform.translation.y;
    yaw = tf2::getYaw(in_base.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

/** Laser pose in base via odom (fallback if direct base→laser fails). */
static bool get_laser_in_base_via_odom(
  tf2_ros::Buffer & tf, const std::string & odom_frame, const std::string & base_frame,
  const std::string & laser_frame, double & x, double & y, double & yaw, double timeout_sec)
{
  try {
    const std::string odom = strip_tf_leading_slash(odom_frame);
    const std::string base = strip_tf_leading_slash(base_frame);
    const std::string laser = strip_tf_leading_slash(laser_frame);
    const auto t_odom_base = tf.lookupTransform(
      odom, base, tf2::TimePointZero, tf2::durationFromSec(timeout_sec));
    const auto t_odom_laser = tf.lookupTransform(
      odom, laser, tf2::TimePointZero, tf2::durationFromSec(timeout_sec));
    tf2::Transform T_ob;
    tf2::Transform T_ol;
    tf2::fromMsg(t_odom_base.transform, T_ob);
    tf2::fromMsg(t_odom_laser.transform, T_ol);
    const tf2::Transform T_bl = T_ob.inverse() * T_ol;
    x = T_bl.getOrigin().x();
    y = T_bl.getOrigin().y();
    yaw = tf2::getYaw(T_bl.getRotation());
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

static bool get_laser_in_base(
  tf2_ros::Buffer & tf, const std::string & odom_frame, const std::string & base_frame,
  const std::string & laser_frame_raw, double & x, double & y, double & yaw, double timeout_sec)
{
  const std::string laser = strip_tf_leading_slash(laser_frame_raw);
  if (laser.empty()) {
    return false;
  }
  if (get_laser_in_base_direct(tf, base_frame, laser, x, y, yaw, timeout_sec)) {
    return true;
  }
  return get_laser_in_base_via_odom(tf, odom_frame, base_frame, laser, x, y, yaw, timeout_sec);
}

class GenesisIcpSocketRelay : public rclcpp::Node
{
public:
  GenesisIcpSocketRelay()
  : Node("genesis_icp_socket_relay")
  {
    tcp_host_ = declare_parameter<std::string>("tcp_host", "127.0.0.1");
    tcp_port_ = declare_parameter<int>("tcp_port", 4403);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "scan");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const double cache_sec = declare_parameter<double>("tf_buffer_cache_sec", 60.0);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 1.0);
    if (tf_lookup_timeout_sec_ <= 0.0) {
      tf_lookup_timeout_sec_ = 1.0;
    }
    const bool scan_best_effort = declare_parameter<bool>("scan_use_best_effort_qos", false);
    const bool tf_static_transient_local =
      declare_parameter<bool>("tf_static_use_transient_local", false);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
      get_clock(), tf2::durationFromSec(cache_sec > 0.0 ? cache_sec : 60.0));
    // rosbag2 play publishes /tf as reliable + volatile (not recorded durability).
    rclcpp::QoS tf_dynamic_qos(rclcpp::KeepLast(200));
    tf_dynamic_qos.reliable();
    tf_dynamic_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    rclcpp::QoS tf_static_qos(rclcpp::KeepLast(100));
    tf_static_qos.reliable();
    if (tf_static_transient_local) {
      tf_static_qos.transient_local();
    } else {
      tf_static_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    }
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, true, tf_dynamic_qos, tf_static_qos);

    rclcpp::QoS scan_qos =
      scan_best_effort ? rclcpp::SensorDataQoS() : rclcpp::QoS(rclcpp::KeepLast(50));
    if (!scan_best_effort) {
      scan_qos.reliable();
      scan_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    }
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos,
      std::bind(&GenesisIcpSocketRelay::onScan, this, std::placeholders::_1));

    reconnect_timer_ = create_wall_timer(1s, [this]() { tryConnect(); });
    tryConnect();
  }

private:
  void tryConnect()
  {
    if (sock_fd_ >= 0) {
      return;
    }
    int fd = connect_tcp(tcp_host_, tcp_port_);
    if (fd >= 0) {
      sock_fd_ = fd;
      RCLCPP_INFO(get_logger(), "Connected to %s:%d", tcp_host_.c_str(), tcp_port_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not connect to fusion at %s:%d", tcp_host_.c_str(), tcp_port_);
    }
  }

  void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    if (sock_fd_ < 0) {
      tryConnect();
      if (sock_fd_ < 0) {
        return;
      }
    }

    rclcpp::Time t(scan->header.stamp);
    double bx = 0, by = 0, byaw = 0;
    if (!get_base_in_odom(
        *tf_buffer_, base_frame_, odom_frame_, t, bx, by, byaw, tf_lookup_timeout_sec_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No tf base-in-odom");
      return;
    }
    double lx = 0, ly = 0, lyaw = 0;
    if (!get_laser_in_base(
        *tf_buffer_, odom_frame_, base_frame_, scan->header.frame_id, lx, ly, lyaw,
        tf_lookup_timeout_sec_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No tf laser-in-base (frame_id='%s', base='%s', odom='%s'); "
        "check URDF/static TF and try scan_use_best_effort_qos:=true for live drivers",
        scan->header.frame_id.c_str(), base_frame_.c_str(), odom_frame_.c_str());
      return;
    }

    genesis_icp::ScanPacket p;
    p.stamp_nsec = static_cast<uint64_t>(t.nanoseconds());
    p.frame_id = strip_tf_leading_slash(scan->header.frame_id);
    p.ranges.assign(scan->ranges.begin(), scan->ranges.end());
    p.angle_min = scan->angle_min;
    p.angle_max = scan->angle_max;
    p.angle_increment = scan->angle_increment;
    p.range_min = scan->range_min;
    p.range_max = scan->range_max;
    p.base_x = bx;
    p.base_y = by;
    p.base_yaw = byaw;
    p.laser_in_base_x = lx;
    p.laser_in_base_y = ly;
    p.laser_in_base_yaw = lyaw;

    if (!genesis_icp::write_full_packet(sock_fd_, p)) {
      RCLCPP_WARN(get_logger(), "Send failed; reconnecting");
      close(sock_fd_);
      sock_fd_ = -1;
    }
  }

  std::string tcp_host_;
  int tcp_port_{4403};
  std::string scan_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double tf_lookup_timeout_sec_{1.0};
  int sock_fd_{-1};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenesisIcpSocketRelay>());
  rclcpp::shutdown();
  return 0;
}
