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

#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "genesis_icp/scan_fusion_core.hpp"
#include "genesis_icp/wire_scan_packet.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

class GenesisIcpNode : public rclcpp::Node
{
public:
  GenesisIcpNode()
  : Node("genesis_icp")
  {
    declare_scan_fusion_parameters(*this);

    use_socket_bridge_ = declare_parameter<bool>("use_socket_bridge", true);
    fusion_listen_port_robot_a_ = declare_parameter<int>("fusion_listen_port_robot_a", 4403);
    fusion_listen_port_robot_b_ = declare_parameter<int>("fusion_listen_port_robot_b", 4405);

    scan_topic_robot_a_ = declare_parameter<std::string>("scan_topic_robot_a", "JK3/sensors/lidar2d_0/scan");
    scan_topic_robot_b_ = declare_parameter<std::string>("scan_topic_robot_b", "JK5/sensors/lidar2d_0/scan");
    odom_frame_robot_a_ = declare_parameter<std::string>("odom_frame_robot_a", "JK3/odom");
    odom_frame_robot_b_ = declare_parameter<std::string>("odom_frame_robot_b", "JK5/odom");
    base_frame_robot_a_ = declare_parameter<std::string>("base_frame_robot_a", "JK3/base_link");
    base_frame_robot_b_ = declare_parameter<std::string>("base_frame_robot_b", "JK5/base_link");

    tf_static_topic_a_ = declare_parameter<std::string>("tf_static_topic_robot_a", "/JK3/tf_static");
    tf_static_topic_b_ = declare_parameter<std::string>("tf_static_topic_robot_b", "/JK5/tf_static");

    scan_pair_buffer_size_ = declare_parameter<int>("scan_pair_buffer_size", 80);
    if (scan_pair_buffer_size_ < 4) {
      scan_pair_buffer_size_ = 4;
    }

    if (!has_parameter("tf_buffer_cache_sec")) {
      declare_parameter<double>("tf_buffer_cache_sec", 60.0);
    }
    tf_buffer_cache_sec_ = get_parameter("tf_buffer_cache_sec").as_double();

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

  void onTimer()
  {
    if (!fusion_core_) {
      fusion_core_ = std::make_unique<ScanFusionCore>(shared_from_this(), pub_tf_a_, pub_tf_b_);
    }
    fusion_core_->process_best_pair(mutex_a_, mutex_b_, buffer_a_, buffer_b_);
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
  std::string tf_static_topic_a_;
  std::string tf_static_topic_b_;
  int scan_pair_buffer_size_{80};
  double tf_buffer_cache_sec_{60.0};

  std::mutex mutex_a_, mutex_b_;
  std::deque<ScanPacket> buffer_a_, buffer_b_;
  int listen_fd_a_{-1};
  int listen_fd_b_{-1};
  std::thread thread_a_;
  std::thread thread_b_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_a_, sub_b_;

  std::unique_ptr<ScanFusionCore> fusion_core_;

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
