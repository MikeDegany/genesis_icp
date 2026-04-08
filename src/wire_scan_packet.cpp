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

#include "genesis_icp/wire_scan_packet.hpp"

#include <arpa/inet.h>
#include <endian.h>
#include <cstring>
#include <unistd.h>
#include <vector>

namespace genesis_icp
{

static bool write_all(int fd, const void * buf, size_t len)
{
  const char * p = static_cast<const char *>(buf);
  size_t left = len;
  while (left > 0) {
    ssize_t n = ::write(fd, p, left);
    if (n <= 0) {
      return false;
    }
    p += n;
    left -= static_cast<size_t>(n);
  }
  return true;
}

static bool read_all(int fd, void * buf, size_t len)
{
  char * p = static_cast<char *>(buf);
  size_t left = len;
  while (left > 0) {
    ssize_t n = ::read(fd, p, left);
    if (n <= 0) {
      return false;
    }
    p += n;
    left -= static_cast<size_t>(n);
  }
  return true;
}

bool write_full_packet(int fd, const ScanPacket & p)
{
  const uint32_t magic = htonl(kWireMagic);
  const uint32_t version = htonl(2);
  const uint64_t stamp_be = htobe64(p.stamp_nsec);
  if (p.frame_id.size() > 65535) {
    return false;
  }
  const uint16_t id_len = htons(static_cast<uint16_t>(p.frame_id.size()));
  const uint32_t n_ranges = htonl(static_cast<uint32_t>(p.ranges.size()));

  if (!write_all(fd, &magic, sizeof(magic))) {
    return false;
  }
  if (!write_all(fd, &version, sizeof(version))) {
    return false;
  }
  if (!write_all(fd, &stamp_be, sizeof(stamp_be))) {
    return false;
  }
  if (!write_all(fd, &id_len, sizeof(id_len))) {
    return false;
  }
  if (!p.frame_id.empty() && !write_all(fd, p.frame_id.data(), p.frame_id.size())) {
    return false;
  }
  if (!write_all(fd, &n_ranges, sizeof(n_ranges))) {
    return false;
  }
  for (float r : p.ranges) {
    uint32_t bits = 0;
    std::memcpy(&bits, &r, sizeof(bits));
    bits = htonl(bits);
    if (!write_all(fd, &bits, sizeof(bits))) {
      return false;
    }
  }
  float angles[5] = {p.angle_min, p.angle_max, p.angle_increment, p.range_min, p.range_max};
  for (int i = 0; i < 5; ++i) {
    uint32_t bits = 0;
    std::memcpy(&bits, &angles[i], sizeof(bits));
    bits = htonl(bits);
    if (!write_all(fd, &bits, sizeof(bits))) {
      return false;
    }
  }
  double pose[3] = {p.base_x, p.base_y, p.base_yaw};
  for (int i = 0; i < 3; ++i) {
    uint64_t bits = 0;
    std::memcpy(&bits, &pose[i], sizeof(bits));
    bits = htobe64(bits);
    if (!write_all(fd, &bits, sizeof(bits))) {
      return false;
    }
  }
  double laser_base[3] = {p.laser_in_base_x, p.laser_in_base_y, p.laser_in_base_yaw};
  for (int i = 0; i < 3; ++i) {
    uint64_t bits = 0;
    std::memcpy(&bits, &laser_base[i], sizeof(bits));
    bits = htobe64(bits);
    if (!write_all(fd, &bits, sizeof(bits))) {
      return false;
    }
  }
  return true;
}

std::optional<ScanPacket> read_full_packet(int fd)
{
  uint32_t magic_net = 0, version_net = 0;
  uint64_t stamp_be = 0;
  uint16_t id_len_net = 0;
  uint32_t n_ranges_net = 0;

  if (!read_all(fd, &magic_net, sizeof(magic_net))) {
    return std::nullopt;
  }
  if (ntohl(magic_net) != kWireMagic) {
    return std::nullopt;
  }
  if (!read_all(fd, &version_net, sizeof(version_net))) {
    return std::nullopt;
  }
  const uint32_t ver = ntohl(version_net);
  if (ver != 1 && ver != 2) {
    return std::nullopt;
  }
  if (!read_all(fd, &stamp_be, sizeof(stamp_be))) {
    return std::nullopt;
  }
  if (!read_all(fd, &id_len_net, sizeof(id_len_net))) {
    return std::nullopt;
  }

  ScanPacket p;
  p.stamp_nsec = be64toh(stamp_be);
  uint16_t id_len = ntohs(id_len_net);
  p.frame_id.resize(id_len);
  if (id_len > 0 && !read_all(fd, &p.frame_id[0], id_len)) {
    return std::nullopt;
  }
  if (!read_all(fd, &n_ranges_net, sizeof(n_ranges_net))) {
    return std::nullopt;
  }
  uint32_t n_ranges = ntohl(n_ranges_net);
  if (n_ranges > 100000) {
    return std::nullopt;
  }
  p.ranges.resize(n_ranges);
  for (uint32_t i = 0; i < n_ranges; ++i) {
    uint32_t bits = 0;
    if (!read_all(fd, &bits, sizeof(bits))) {
      return std::nullopt;
    }
    bits = ntohl(bits);
    float r = 0;
    std::memcpy(&r, &bits, sizeof(r));
    p.ranges[i] = r;
  }
  float angles[5];
  for (int i = 0; i < 5; ++i) {
    uint32_t bits = 0;
    if (!read_all(fd, &bits, sizeof(bits))) {
      return std::nullopt;
    }
    bits = ntohl(bits);
    std::memcpy(&angles[i], &bits, sizeof(float));
  }
  p.angle_min = angles[0];
  p.angle_max = angles[1];
  p.angle_increment = angles[2];
  p.range_min = angles[3];
  p.range_max = angles[4];

  double pose[3];
  for (int i = 0; i < 3; ++i) {
    uint64_t bits = 0;
    if (!read_all(fd, &bits, sizeof(bits))) {
      return std::nullopt;
    }
    bits = be64toh(bits);
    std::memcpy(&pose[i], &bits, sizeof(double));
  }
  p.base_x = pose[0];
  p.base_y = pose[1];
  p.base_yaw = pose[2];
  if (ver >= 2) {
    double lb[3];
    for (int i = 0; i < 3; ++i) {
      uint64_t bits = 0;
      if (!read_all(fd, &bits, sizeof(bits))) {
        return std::nullopt;
      }
      bits = be64toh(bits);
      std::memcpy(&lb[i], &bits, sizeof(double));
    }
    p.laser_in_base_x = lb[0];
    p.laser_in_base_y = lb[1];
    p.laser_in_base_yaw = lb[2];
  }
  return p;
}

sensor_msgs::msg::LaserScan to_laser_scan(const ScanPacket & p)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = p.angle_min;
  s.angle_max = p.angle_max;
  s.angle_increment = p.angle_increment;
  s.range_min = p.range_min;
  s.range_max = p.range_max;
  s.ranges = p.ranges;
  s.header.frame_id = p.frame_id;
  uint64_t sec = p.stamp_nsec / 1000000000ULL;
  uint64_t nsec = p.stamp_nsec % 1000000000ULL;
  s.header.stamp.sec = static_cast<int32_t>(sec);
  s.header.stamp.nanosec = static_cast<uint32_t>(nsec);
  return s;
}

}  // namespace genesis_icp
