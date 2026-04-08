/*
 * Copyright (c) 2026 Mike Degany <mike.degany@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef GENESIS_ICP__SCAN_FUSION_CORE_HPP_
#define GENESIS_ICP__SCAN_FUSION_CORE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "genesis_icp/laser_utils.hpp"
#include "genesis_icp/wire_scan_packet.hpp"
#include "karto_sdk/Mapper.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace genesis_icp
{

/** Declare parameters consumed by ScanFusionCore (call before constructing ScanFusionCore). */
void declare_scan_fusion_parameters(rclcpp::Node & node);

/** Pick scans with minimum |t_a - t_b| subject to |t_a - t_b| <= max_dt_sec. */
bool select_best_scan_pair(
  const std::deque<ScanPacket> & buf_a, const std::deque<ScanPacket> & buf_b, double max_dt_sec,
  ScanPacket & out_a, ScanPacket & out_b);

/**
 * Karto scan matching + static TF publish shared by genesis_icp_node (TCP) and offline bag playback.
 * Parameters are read from `node` on construction (same names as genesis_icp.yaml).
 */
class ScanFusionCore
{
public:
  ScanFusionCore(
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_a,
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_b);

  /**
   * Under mutex_a/mutex_b, selects the best scan pair; then runs matching outside the locks.
   * Returns true if a match was published (same criteria as legacy onTimer).
   */
  bool process_best_pair(
    std::mutex & mutex_a, std::mutex & mutex_b,
    std::deque<ScanPacket> & buffer_a, std::deque<ScanPacket> & buffer_b);

  /** Process a specific pair (offline: caller already synchronized buffers). */
  bool process_pair(const ScanPacket & a, const ScanPacket & b);

private:
  void ensure_mapper_ready();
  laser_utils::LaserMetadata & get_laser_meta(
    const std::string & key, const ScanPacket & pkt,
    const sensor_msgs::msg::LaserScan & scan_msg, laser_utils::LaserAssistant & asst);
  void publish_static_transforms(const tf2::Transform & T_odomA_odomB);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_a_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_b_;

  double guess_dx_{0}, guess_dy_{0}, guess_dyaw_{0};
  std::string global_odom_frame_;
  std::string odom_child_robot_a_;
  std::string odom_child_robot_b_;
  std::string anchor_mode_;
  double max_time_delta_sec_{10.0};
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

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unordered_map<std::string, laser_utils::LaserMetadata> lasers_;
  karto::Dataset dataset_;
};

}  // namespace genesis_icp

#endif  // GENESIS_ICP__SCAN_FUSION_CORE_HPP_
