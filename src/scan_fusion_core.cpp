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

#include "genesis_icp/scan_fusion_core.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "genesis_icp/mapper_configure.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "karto_sdk/Karto.h"
#include "karto_sdk/Types.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace genesis_icp
{

namespace
{

karto::Pose2 transform_base_B_to_A(const karto::Pose2 & T_AB, const karto::Pose2 & p_B)
{
  const double c = std::cos(T_AB.GetHeading());
  const double s = std::sin(T_AB.GetHeading());
  const double x = c * p_B.GetX() - s * p_B.GetY() + T_AB.GetX();
  const double y = s * p_B.GetX() + c * p_B.GetY() + T_AB.GetY();
  return karto::Pose2(
    x, y,
    karto::math::NormalizeAngle(p_B.GetHeading() + T_AB.GetHeading()));
}

void build_match_seeds(
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

tf2::Transform kartoPoseToTf(const karto::Pose2 & p)
{
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(p.GetX(), p.GetY(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.GetHeading());
  tf.setRotation(q);
  return tf;
}

geometry_msgs::msg::TransformStamped laserMountFromPacket(
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

double shortest_angle_delta_rad(double from_yaw, double to_yaw)
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

void smooth_tf2_transform(
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

bool transform_msg_finite(const geometry_msgs::msg::Transform & tr)
{
  const auto & t = tr.translation;
  const auto & r = tr.rotation;
  if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z)) {
    return false;
  }
  if (!std::isfinite(r.x) || !std::isfinite(r.y) || !std::isfinite(r.z) || !std::isfinite(r.w)) {
    return false;
  }
  const double n2 = r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w;
  return std::isfinite(n2) && n2 > 1e-12;
}

}  // namespace

void declare_scan_fusion_parameters(rclcpp::Node & node)
{
  node.declare_parameter<double>("initial_guess_odom_b_in_a_x", 0.0);
  node.declare_parameter<double>("initial_guess_odom_b_in_a_y", 0.0);
  node.declare_parameter<double>("initial_guess_odom_b_in_a_yaw", 0.0);

  node.declare_parameter<std::string>("global_odom_frame", "global_odom");
  node.declare_parameter<std::string>("odom_child_frame_robot_a", "JK3/odom");
  node.declare_parameter<std::string>("odom_child_frame_robot_b", "JK5/odom");
  node.declare_parameter<std::string>("anchor_mode", "robot_a");

  node.declare_parameter<double>("max_time_delta_sec", 10.0);
  node.declare_parameter<double>("minimum_match_response", 0.05);
  node.declare_parameter<double>("max_laser_range", 12.0);
  node.declare_parameter<bool>("match_scan_do_penalize", false);
  node.declare_parameter<double>("relative_pose_smooth_alpha", 0.25);

  node.declare_parameter<bool>("use_odom_poses_for_match", false);
  node.declare_parameter<bool>("multi_seed_match", false);
  node.declare_parameter<double>("multi_seed_xy_step_m", 1.0);
  node.declare_parameter<double>("multi_seed_xy_extent_m", 1.0);
  node.declare_parameter<double>("multi_seed_yaw_step_deg", 10.0);
  node.declare_parameter<double>("multi_seed_yaw_extent_deg", 20.0);
  node.declare_parameter<int>("multi_seed_max_seeds", 48);

  node.declare_parameter<std::string>("laser_id_robot_a", "genesis_robot_a_laser");
  node.declare_parameter<std::string>("laser_id_robot_b", "genesis_robot_b_laser");

  node.declare_parameter<int>("tf_static_min_consecutive_matches", 3);
  node.declare_parameter<bool>("tf_static_publish_only_once", false);
  // tf_buffer_cache_sec: declared by genesis_icp_node / genesis_icp_offline_node (not here) to avoid
  // duplicate declare with the node's subscription-path TF buffer.
}

bool select_best_scan_pair(
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

ScanFusionCore::ScanFusionCore(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_a,
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_b)
: node_(std::move(node)),
  pub_tf_a_(std::move(pub_tf_a)),
  pub_tf_b_(std::move(pub_tf_b))
{
  guess_dx_ = node_->get_parameter("initial_guess_odom_b_in_a_x").as_double();
  guess_dy_ = node_->get_parameter("initial_guess_odom_b_in_a_y").as_double();
  guess_dyaw_ = node_->get_parameter("initial_guess_odom_b_in_a_yaw").as_double();

  global_odom_frame_ = node_->get_parameter("global_odom_frame").as_string();
  odom_child_robot_a_ = node_->get_parameter("odom_child_frame_robot_a").as_string();
  odom_child_robot_b_ = node_->get_parameter("odom_child_frame_robot_b").as_string();
  anchor_mode_ = node_->get_parameter("anchor_mode").as_string();

  max_time_delta_sec_ = node_->get_parameter("max_time_delta_sec").as_double();
  minimum_match_response_ = node_->get_parameter("minimum_match_response").as_double();
  max_laser_range_ = node_->get_parameter("max_laser_range").as_double();
  match_scan_do_penalize_ = node_->get_parameter("match_scan_do_penalize").as_bool();
  relative_pose_smooth_alpha_ = node_->get_parameter("relative_pose_smooth_alpha").as_double();
  if (relative_pose_smooth_alpha_ < 0.0) {
    relative_pose_smooth_alpha_ = 0.0;
  } else if (relative_pose_smooth_alpha_ > 1.0) {
    relative_pose_smooth_alpha_ = 1.0;
  }

  use_odom_poses_for_match_ = node_->get_parameter("use_odom_poses_for_match").as_bool();
  multi_seed_match_ = node_->get_parameter("multi_seed_match").as_bool();
  multi_seed_xy_step_m_ = node_->get_parameter("multi_seed_xy_step_m").as_double();
  multi_seed_xy_extent_m_ = node_->get_parameter("multi_seed_xy_extent_m").as_double();
  multi_seed_yaw_step_deg_ = node_->get_parameter("multi_seed_yaw_step_deg").as_double();
  multi_seed_yaw_extent_deg_ = node_->get_parameter("multi_seed_yaw_extent_deg").as_double();
  multi_seed_max_seeds_ = node_->get_parameter("multi_seed_max_seeds").as_int();
  if (multi_seed_max_seeds_ < 1) {
    multi_seed_max_seeds_ = 1;
  }

  laser_id_robot_a_ = node_->get_parameter("laser_id_robot_a").as_string();
  laser_id_robot_b_ = node_->get_parameter("laser_id_robot_b").as_string();

  tf_static_min_consecutive_matches_ = node_->get_parameter("tf_static_min_consecutive_matches").as_int();
  if (tf_static_min_consecutive_matches_ < 1) {
    tf_static_min_consecutive_matches_ = 1;
  }
  tf_static_publish_only_once_ = node_->get_parameter("tf_static_publish_only_once").as_bool();

  if (node_->has_parameter("tf_buffer_cache_sec")) {
    tf_buffer_cache_sec_ = node_->get_parameter("tf_buffer_cache_sec").as_double();
  } else {
    tf_buffer_cache_sec_ = 60.0;
  }

  mapper_ = std::make_unique<karto::Mapper>();
}

bool ScanFusionCore::process_best_pair(
  std::mutex & mutex_a, std::mutex & mutex_b,
  std::deque<ScanPacket> & buffer_a, std::deque<ScanPacket> & buffer_b)
{
  ScanPacket a;
  ScanPacket b;
  {
    std::lock_guard<std::mutex> la(mutex_a);
    std::lock_guard<std::mutex> lb(mutex_b);
    if (!select_best_scan_pair(buffer_a, buffer_b, max_time_delta_sec_, a, b)) {
      return false;
    }
  }
  return process_pair(a, b);
}

void ScanFusionCore::ensure_mapper_ready()
{
  if (!mapper_params_applied_) {
    configure_karto_mapper(node_, mapper_.get());
    mapper_params_applied_ = true;
  }
  if (!mapper_initialized_) {
    mapper_->Initialize(static_cast<kt_double>(max_laser_range_));
    mapper_initialized_ = true;
  }
}

laser_utils::LaserMetadata & ScanFusionCore::get_laser_meta(
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

bool ScanFusionCore::process_pair(const ScanPacket & a, const ScanPacket & b)
{
  ensure_mapper_ready();

  sensor_msgs::msg::LaserScan scan_a = to_laser_scan(a);
  scan_a.header.frame_id = laser_id_robot_a_;
  sensor_msgs::msg::LaserScan scan_b = to_laser_scan(b);
  scan_b.header.frame_id = laser_id_robot_b_;

  if (!tf_buffer_) {
    const double cache = tf_buffer_cache_sec_ > 0.0 ? tf_buffer_cache_sec_ : 60.0;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock(), tf2::durationFromSec(cache));
    // Same-thread setTransform + lookup (socket/offline); avoids tf2 "no dedicated thread" errors on transform().
    tf_buffer_->setUsingDedicatedThread(true);
  }
  laser_utils::LaserAssistant asst(node_, tf_buffer_.get(), "map");

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
      node_->get_logger(), *node_->get_clock(), 3000,
      "Scan match response %.4f below threshold %.4f", static_cast<double>(response),
      minimum_match_response_);
    if (!have_published_tf_static_once_) {
      consecutive_good_matches_ = 0;
    }
    delete ra;
    delete rb;
    return false;
  }

  if (!have_published_tf_static_once_) {
    ++consecutive_good_matches_;
    if (consecutive_good_matches_ < tf_static_min_consecutive_matches_) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Match above threshold (response=%.4f), solidifying %d / %d consecutive matches before "
        "publishing tf_static",
        static_cast<double>(response), consecutive_good_matches_, tf_static_min_consecutive_matches_);
      delete ra;
      delete rb;
      return true;
    }
  }

  if (tf_static_publish_only_once_ && have_published_tf_static_once_) {
    delete ra;
    delete rb;
    return true;
  }

  const tf2::Transform T_sensor_B_in_A = kartoPoseToTf(best_mean);
  const tf2::Transform T_sensor_B_in_B = kartoPoseToTf(sensor_b_in_B);
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
      node_->get_logger(), *node_->get_clock(), 2000,
      "Match ok (response=%.4f): odom_B in odom_A dist=%.3f m (raw=%.3f) dxy=(%.3f, %.3f) dyaw=%.2f deg | "
      "guess_base_sep=%.3f m (set initial_guess_* + widen correlation_search_space_dimension if wrong)",
      static_cast<double>(response), dist_m, raw_dist, t_b_in_a.x(), t_b_in_a.y(), dyaw_rad * kRadToDeg,
      guess_base_sep_m);
  } else {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Match ok (response=%.4f): odom_B in odom_A dist=%.3f m (raw=%.3f) dxy=(%.3f, %.3f) dyaw=%.2f deg | "
      "odom_free seed_xy_yaw=(%.3f, %.3f, %.2f deg) seeds_tried=%zu "
      "(initial_guess_* + multi_seed_* / correlation_search_space_dimension tune overlap)",
      static_cast<double>(response), dist_m, raw_dist, t_b_in_a.x(), t_b_in_a.y(), dyaw_rad * kRadToDeg,
      base_b_in_A_center.GetX(), base_b_in_A_center.GetY(),
      base_b_in_A_center.GetHeading() * kRadToDeg, seeds.size());
  }

  publish_static_transforms(T_publish);
  have_published_tf_static_once_ = true;

  delete ra;
  delete rb;
  return true;
}

void ScanFusionCore::publish_static_transforms(const tf2::Transform & T_odomA_odomB)
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
    RCLCPP_WARN(node_->get_logger(), "Unknown anchor_mode '%s', using robot_a", anchor_mode_.c_str());
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

  if (!transform_msg_finite(ta.transform) || !transform_msg_finite(tb.transform)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Skipping fusion tf_static publish: non-finite transform (check match / smoothing)");
    return;
  }

  tf2_msgs::msg::TFMessage msg_a;
  msg_a.transforms.push_back(ta);
  tf2_msgs::msg::TFMessage msg_b;
  msg_b.transforms.push_back(tb);
  pub_tf_a_->publish(msg_a);
  pub_tf_b_->publish(msg_b);
}

}  // namespace genesis_icp
