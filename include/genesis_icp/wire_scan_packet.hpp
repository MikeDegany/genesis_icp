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

#ifndef GENESIS_ICP__WIRE_SCAN_PACKET_HPP_
#define GENESIS_ICP__WIRE_SCAN_PACKET_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include "sensor_msgs/msg/laser_scan.hpp"

namespace genesis_icp
{

static constexpr uint32_t kWireMagic = 0x50434947;  // 'GICP' LE

struct ScanPacket
{
  uint64_t stamp_nsec{0};
  std::string frame_id;
  std::vector<float> ranges;
  float angle_min{0};
  float angle_max{0};
  float angle_increment{0};
  float range_min{0};
  float range_max{0};
  /** Base frame pose in this robot's odom (same convention as slam_toolbox getOdomPose). */
  double base_x{0};
  double base_y{0};
  double base_yaw{0};
  /** Laser frame expressed in base_frame (for Karto LaserRangeFinder offset). */
  double laser_in_base_x{0};
  double laser_in_base_y{0};
  double laser_in_base_yaw{0};
};

bool write_full_packet(int fd, const ScanPacket & p);
std::optional<ScanPacket> read_full_packet(int fd);

sensor_msgs::msg::LaserScan to_laser_scan(const ScanPacket & p);

}  // namespace genesis_icp

#endif  // GENESIS_ICP__WIRE_SCAN_PACKET_HPP_
