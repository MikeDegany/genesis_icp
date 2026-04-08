# Copyright (c) 2026 Mike Degany <mike.degany@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.5)
project(genesis_icp)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)
find_package(TBB REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/lib/karto_sdk/cmake")
add_subdirectory(lib/karto_sdk)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(DEPS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
  std_msgs
)

add_library(genesis_icp_common STATIC
  src/laser_utils.cpp
  src/wire_scan_packet.cpp
  src/mapper_configure.cpp
)
target_include_directories(genesis_icp_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(genesis_icp_common ${DEPS})
target_link_libraries(genesis_icp_common karto_genesis_icp ${Boost_LIBRARIES} TBB::tbb)

add_library(genesis_icp_scan_fusion STATIC src/scan_fusion_core.cpp)
target_include_directories(genesis_icp_scan_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(genesis_icp_scan_fusion ${DEPS})
target_link_libraries(genesis_icp_scan_fusion genesis_icp_common)

add_executable(genesis_icp_socket_relay src/socket_relay_main.cpp)
ament_target_dependencies(genesis_icp_socket_relay ${DEPS})
target_link_libraries(genesis_icp_socket_relay genesis_icp_common)

add_executable(genesis_icp_node src/genesis_icp_node.cpp)
ament_target_dependencies(genesis_icp_node ${DEPS})
target_link_libraries(genesis_icp_node genesis_icp_scan_fusion genesis_icp_common)

add_executable(genesis_icp_offline_node src/genesis_icp_offline_node.cpp)
ament_target_dependencies(genesis_icp_offline_node
  ${DEPS}
  rosbag2_cpp
  rosbag2_storage
)
target_link_libraries(genesis_icp_offline_node genesis_icp_scan_fusion genesis_icp_common)

install(TARGETS genesis_icp_socket_relay genesis_icp_node genesis_icp_offline_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
