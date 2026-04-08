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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <optional>
#include <unordered_map>
#include <vector>

#include "genesis_icp/scan_fusion_core.hpp"
#include "genesis_icp/wire_scan_packet.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace genesis_icp
{

static std::string strip_leading_slash(std::string id)
{
  while (!id.empty() && id.front() == '/') {
    id.erase(0, 1);
  }
  return id;
}

static std::string norm_topic(std::string t)
{
  while (!t.empty() && t.front() == '/') {
    t.erase(0, 1);
  }
  return t;
}

static bool topic_matches(const std::string & recorded, const std::string & configured)
{
  return norm_topic(recorded) == norm_topic(configured);
}

static void apply_tf_message(
  tf2_ros::Buffer & buf, const tf2_msgs::msg::TFMessage & msg, bool is_static)
{
  for (geometry_msgs::msg::TransformStamped ts : msg.transforms) {
    ts.header.frame_id = strip_leading_slash(ts.header.frame_id);
    ts.child_frame_id = strip_leading_slash(ts.child_frame_id);
    try {
      buf.setTransform(ts, "bag", is_static);
    } catch (const tf2::TransformException &) {
    }
  }
}

static bool fill_scan_packet(
  tf2_ros::Buffer & tf, const sensor_msgs::msg::LaserScan & scan,
  const std::string & odom_frame, const std::string & base_frame, ScanPacket & out)
{
  const std::string odom = strip_leading_slash(odom_frame);
  const std::string base = strip_leading_slash(base_frame);
  const std::string laser = strip_leading_slash(scan.header.frame_id);
  if (laser.empty()) {
    return false;
  }

  // Single-threaded bag replay: timeout must be zero (no TransformListener thread).
  const tf2::Duration no_wait = tf2::durationFromSec(0.0);
  geometry_msgs::msg::TransformStamped odom_pose;
  try {
    odom_pose = tf.lookupTransform(
      odom, base, tf2_ros::fromMsg(scan.header.stamp), no_wait);
  } catch (const tf2::TransformException &) {
    try {
      odom_pose = tf.lookupTransform(odom, base, tf2::TimePointZero, no_wait);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  geometry_msgs::msg::TransformStamped in_base;
  try {
    in_base = tf.lookupTransform(
      base, laser, tf2_ros::fromMsg(scan.header.stamp), no_wait);
  } catch (const tf2::TransformException &) {
    try {
      in_base = tf.lookupTransform(base, laser, tf2::TimePointZero, no_wait);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  rclcpp::Time t(scan.header.stamp);
  out.stamp_nsec = static_cast<uint64_t>(t.nanoseconds());
  out.frame_id = scan.header.frame_id;
  out.ranges.assign(scan.ranges.begin(), scan.ranges.end());
  out.angle_min = scan.angle_min;
  out.angle_max = scan.angle_max;
  out.angle_increment = scan.angle_increment;
  out.range_min = scan.range_min;
  out.range_max = scan.range_max;
  out.base_x = odom_pose.transform.translation.x;
  out.base_y = odom_pose.transform.translation.y;
  out.base_yaw = tf2::getYaw(odom_pose.transform.rotation);
  out.laser_in_base_x = in_base.transform.translation.x;
  out.laser_in_base_y = in_base.transform.translation.y;
  out.laser_in_base_yaw = tf2::getYaw(in_base.transform.rotation);
  return true;
}

static std::string lookup_topic_type(
  const std::unordered_map<std::string, std::string> & types,
  const std::string & topic_from_bag)
{
  auto it = types.find(topic_from_bag);
  if (it != types.end()) {
    return it->second;
  }
  for (const auto & kv : types) {
    if (topic_matches(kv.first, topic_from_bag)) {
      return kv.second;
    }
  }
  return {};
}

static int topic_priority(
  const std::string & topic, const std::string & scan_topic,
  const std::string & tf_topic, const std::string & tf_static_topic)
{
  if (topic_matches(topic, tf_static_topic)) {
    return 3;
  }
  if (topic_matches(topic, tf_topic)) {
    return 2;
  }
  if (topic_matches(topic, scan_topic)) {
    return 1;
  }
  return 0;
}

static int64_t builtin_time_to_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

static bool topic_is_relevant(
  const std::string & topic, const std::string & scan_topic,
  const std::string & tf_topic, const std::string & tf_static_topic)
{
  return topic_priority(topic, scan_topic, tf_topic, tf_static_topic) > 0;
}

struct OfflineQueuedMsg
{
  int64_t sort_ns{0};
  int priority{0};
  int robot{0};
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag;
};

static std::vector<OfflineQueuedMsg> load_filtered_queue(
  rosbag2_cpp::Reader & reader, const std::unordered_map<std::string, std::string> & types,
  int robot, const std::string & scan_topic, const std::string & tf_topic,
  const std::string & tf_static_topic)
{
  std::vector<OfflineQueuedMsg> out;
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> ser_scan;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> ser_tf;

  while (reader.has_next()) {
    auto bag = reader.read_next();
    if (!topic_is_relevant(bag->topic_name, scan_topic, tf_topic, tf_static_topic)) {
      continue;
    }
    const std::string type = lookup_topic_type(types, bag->topic_name);
    if (type.empty()) {
      continue;
    }
    const int pri = topic_priority(bag->topic_name, scan_topic, tf_topic, tf_static_topic);
    int64_t sort_ns = static_cast<int64_t>(bag->time_stamp);
    rclcpp::SerializedMessage extracted(*bag->serialized_data);

    if (type == "sensor_msgs/msg/LaserScan") {
      sensor_msgs::msg::LaserScan scan;
      ser_scan.deserialize_message(&extracted, &scan);
      sort_ns = builtin_time_to_ns(scan.header.stamp);
    } else if (type == "tf2_msgs/msg/TFMessage") {
      tf2_msgs::msg::TFMessage tfm;
      ser_tf.deserialize_message(&extracted, &tfm);
      sort_ns = 0;
      for (const auto & ts : tfm.transforms) {
        const int64_t tns = builtin_time_to_ns(ts.header.stamp);
        sort_ns = (sort_ns == 0) ? tns : std::max(sort_ns, tns);
      }
      if (sort_ns == 0) {
        sort_ns = static_cast<int64_t>(bag->time_stamp);
      }
    }

    out.push_back(OfflineQueuedMsg{sort_ns, pri, robot, std::move(bag)});
  }
  return out;
}

class GenesisIcpOfflineNode : public rclcpp::Node
{
public:
  GenesisIcpOfflineNode()
  : Node("genesis_icp")
  {
    declare_scan_fusion_parameters(*this);

    robot_a_bag_ = declare_parameter<std::string>("robot_a_bag", "");
    robot_b_bag_ = declare_parameter<std::string>("robot_b_bag", "");
    robot_a_scan_topic_ = declare_parameter<std::string>("robot_a_scan_topic", "/JK3/sensors/lidar2d_0/scan");
    robot_b_scan_topic_ = declare_parameter<std::string>("robot_b_scan_topic", "/JK5/sensors/lidar2d_0/scan");
    robot_a_tf_topic_ = declare_parameter<std::string>("robot_a_tf_topic", "/JK3/tf");
    robot_b_tf_topic_ = declare_parameter<std::string>("robot_b_tf_topic", "/JK5/tf");
    robot_a_tf_static_topic_ = declare_parameter<std::string>("robot_a_tf_static_topic", "/JK3/tf_static");
    robot_b_tf_static_topic_ = declare_parameter<std::string>("robot_b_tf_static_topic", "/JK5/tf_static");

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

    playback_realtime_factor_ = declare_parameter<double>("playback_realtime_factor", 0.0);
    max_bag_messages_ = declare_parameter<int>("max_bag_messages", 0);

    const double cache = 600.0;
    tf_a_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(cache));
    tf_b_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(cache));
    tf_a_->setUsingDedicatedThread(true);
    tf_b_->setUsingDedicatedThread(true);

    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();
    pub_tf_a_ = create_publisher<tf2_msgs::msg::TFMessage>(tf_static_topic_a_, qos);
    pub_tf_b_ = create_publisher<tf2_msgs::msg::TFMessage>(tf_static_topic_b_, qos);
  }

  void run()
  {
    if (robot_a_bag_.empty() || robot_b_bag_.empty()) {
      RCLCPP_FATAL(get_logger(), "robot_a_bag and robot_b_bag must be non-empty paths to rosbag2");
      return;
    }

    fusion_core_ = std::make_unique<ScanFusionCore>(shared_from_this(), pub_tf_a_, pub_tf_b_);

    rosbag2_storage::StorageOptions sa;
    sa.uri = robot_a_bag_;
    sa.storage_id = "";
    rosbag2_storage::StorageOptions sb;
    sb.uri = robot_b_bag_;
    sb.storage_id = "";

    rosbag2_cpp::Reader reader_a;
    rosbag2_cpp::Reader reader_b;
    reader_a.open(sa);
    reader_b.open(sb);

    std::unordered_map<std::string, std::string> types_a;
    for (const auto & meta : reader_a.get_all_topics_and_types()) {
      types_a[meta.name] = meta.type;
    }
    std::unordered_map<std::string, std::string> types_b;
    for (const auto & meta : reader_b.get_all_topics_and_types()) {
      types_b[meta.name] = meta.type;
    }

    RCLCPP_INFO(
      get_logger(), "Offline fusion: bag A '%s', bag B '%s'", robot_a_bag_.c_str(),
      robot_b_bag_.c_str());

    std::vector<OfflineQueuedMsg> qa = load_filtered_queue(
      reader_a, types_a, 0, robot_a_scan_topic_, robot_a_tf_topic_, robot_a_tf_static_topic_);
    std::vector<OfflineQueuedMsg> qb = load_filtered_queue(
      reader_b, types_b, 1, robot_b_scan_topic_, robot_b_tf_topic_, robot_b_tf_static_topic_);
    std::vector<OfflineQueuedMsg> merged;
    merged.reserve(qa.size() + qb.size());
    merged.insert(merged.end(), qa.begin(), qa.end());
    merged.insert(merged.end(), qb.begin(), qb.end());
    std::sort(
      merged.begin(), merged.end(),
      [](const OfflineQueuedMsg & a, const OfflineQueuedMsg & b) {
        if (a.sort_ns != b.sort_ns) {
          return a.sort_ns < b.sort_ns;
        }
        if (a.priority != b.priority) {
          return a.priority > b.priority;
        }
        return a.robot < b.robot;
      });

    RCLCPP_INFO(
      get_logger(), "Merged timeline: %zu messages (A=%zu, B=%zu)", merged.size(), qa.size(), qb.size());

    std::optional<int64_t> prev_sort_ns;
    int msg_count = 0;

    for (const OfflineQueuedMsg & ev : merged) {
      if (!rclcpp::ok()) {
        break;
      }
      if (max_bag_messages_ > 0 && msg_count >= max_bag_messages_) {
        break;
      }

      if (playback_realtime_factor_ > 0.0 && prev_sort_ns) {
        const int64_t delta = ev.sort_ns - *prev_sort_ns;
        if (delta > 0) {
          const auto sleep_ns =
            static_cast<int64_t>(static_cast<double>(delta) / playback_realtime_factor_);
          std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        }
      }
      prev_sort_ns = ev.sort_ns;

      if (ev.robot == 0) {
        dispatch_message(ev.bag, types_a, *tf_a_, true);
      } else {
        dispatch_message(ev.bag, types_b, *tf_b_, false);
      }

      ++msg_count;
      if ((msg_count & 0x3FF) == 0) {
        rclcpp::spin_some(shared_from_this());
      }
    }

    RCLCPP_INFO(get_logger(), "Finished reading bags (%d messages).", msg_count);
    rclcpp::spin_some(shared_from_this());
  }

private:
  void dispatch_message(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_msg,
    const std::unordered_map<std::string, std::string> & types,
    tf2_ros::Buffer & tf_buf, bool is_robot_a)
  {
    const std::string type = lookup_topic_type(types, bag_msg->topic_name);
    if (type.empty()) {
      return;
    }
    const std::string & scan_topic = is_robot_a ? robot_a_scan_topic_ : robot_b_scan_topic_;
    const std::string & tf_topic = is_robot_a ? robot_a_tf_topic_ : robot_b_tf_topic_;
    const std::string & tf_static_topic = is_robot_a ? robot_a_tf_static_topic_ : robot_b_tf_static_topic_;
    const std::string & odom = is_robot_a ? odom_frame_robot_a_ : odom_frame_robot_b_;
    const std::string & base = is_robot_a ? base_frame_robot_a_ : base_frame_robot_b_;

    rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);

    if (type == "tf2_msgs/msg/TFMessage") {
      if (!topic_matches(bag_msg->topic_name, tf_topic) &&
        !topic_matches(bag_msg->topic_name, tf_static_topic))
      {
        return;
      }
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> ser;
      tf2_msgs::msg::TFMessage tfm;
      ser.deserialize_message(&extracted, &tfm);
      const bool is_static = topic_matches(bag_msg->topic_name, tf_static_topic);
      apply_tf_message(tf_buf, tfm, is_static);
      return;
    }

    if (type == "sensor_msgs/msg/LaserScan") {
      if (!topic_matches(bag_msg->topic_name, scan_topic)) {
        return;
      }
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> ser;
      sensor_msgs::msg::LaserScan scan;
      ser.deserialize_message(&extracted, &scan);
      ScanPacket pkt;
      if (!fill_scan_packet(tf_buf, scan, odom, base, pkt)) {
        return;
      }
      if (is_robot_a) {
        std::lock_guard<std::mutex> lk(mutex_a_);
        buffer_a_.push_back(std::move(pkt));
        while (buffer_a_.size() > static_cast<size_t>(scan_pair_buffer_size_)) {
          buffer_a_.pop_front();
        }
      } else {
        std::lock_guard<std::mutex> lk(mutex_b_);
        buffer_b_.push_back(std::move(pkt));
        while (buffer_b_.size() > static_cast<size_t>(scan_pair_buffer_size_)) {
          buffer_b_.pop_front();
        }
      }
      fusion_core_->process_best_pair(mutex_a_, mutex_b_, buffer_a_, buffer_b_);
    }
  }

  std::string robot_a_bag_;
  std::string robot_b_bag_;
  std::string robot_a_scan_topic_;
  std::string robot_b_scan_topic_;
  std::string robot_a_tf_topic_;
  std::string robot_b_tf_topic_;
  std::string robot_a_tf_static_topic_;
  std::string robot_b_tf_static_topic_;
  std::string odom_frame_robot_a_;
  std::string odom_frame_robot_b_;
  std::string base_frame_robot_a_;
  std::string base_frame_robot_b_;
  std::string tf_static_topic_a_;
  std::string tf_static_topic_b_;
  int scan_pair_buffer_size_{80};
  double playback_realtime_factor_{0.0};
  int max_bag_messages_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_a_;
  std::unique_ptr<tf2_ros::Buffer> tf_b_;

  std::mutex mutex_a_, mutex_b_;
  std::deque<ScanPacket> buffer_a_, buffer_b_;

  std::unique_ptr<ScanFusionCore> fusion_core_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_a_, pub_tf_b_;
};

}  // namespace genesis_icp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<genesis_icp::GenesisIcpOfflineNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
