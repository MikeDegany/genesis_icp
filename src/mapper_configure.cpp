/*
 * Derived from slam_toolbox slam_mapper.cpp (configure) for genesis_icp.
 * Copyright as in slam_toolbox.
 */
#include "genesis_icp/mapper_configure.hpp"

namespace genesis_icp
{

using karto::math::Square;

void configure_karto_mapper(rclcpp::Node::SharedPtr node, karto::Mapper * mapper)
{
  bool use_scan_matching = true;
  if (!node->has_parameter("use_scan_matching")) {
    node->declare_parameter("use_scan_matching", use_scan_matching);
  }
  node->get_parameter("use_scan_matching", use_scan_matching);
  mapper->setParamUseScanMatching(use_scan_matching);

  bool use_scan_barycenter = true;
  if (!node->has_parameter("use_scan_barycenter")) {
    node->declare_parameter("use_scan_barycenter", use_scan_barycenter);
  }
  node->get_parameter("use_scan_barycenter", use_scan_barycenter);
  mapper->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_travel_distance = 0.5; 
  if (!node->has_parameter("minimum_travel_distance")) {
    node->declare_parameter("minimum_travel_distance", minimum_travel_distance);
  }
  node->get_parameter("minimum_travel_distance", minimum_travel_distance);
  mapper->setParamMinimumTravelDistance(minimum_travel_distance);

  double minimum_travel_heading = 0.5;
  if (!node->has_parameter("minimum_travel_heading")) {
    node->declare_parameter("minimum_travel_heading", minimum_travel_heading);
  }
  node->get_parameter("minimum_travel_heading", minimum_travel_heading);
  mapper->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size = 10;
  if (!node->has_parameter("scan_buffer_size")) {
    node->declare_parameter("scan_buffer_size", scan_buffer_size);
  }
  node->get_parameter("scan_buffer_size", scan_buffer_size);
  if (scan_buffer_size <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set scan_buffer_size to be a value smaller than zero,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_size = 10;
  }
  mapper->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance = 10;
  if (!node->has_parameter("scan_buffer_maximum_scan_distance")) {
    node->declare_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance);
  }
  node->get_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance);
  if (Square(scan_buffer_maximum_scan_distance) <= 1e-06) {
    RCLCPP_WARN(node->get_logger(),
      "You've set scan_buffer_maximum_scan_distance to be a value whose square is smaller than 1e-06,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_maximum_scan_distance = 10;
  }
  mapper->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine = 0.1;
  if (!node->has_parameter("link_match_minimum_response_fine")) {
    node->declare_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine);
  }
  node->get_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine);
  mapper->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance = 1.5;
  if (!node->has_parameter("link_scan_maximum_distance")) {
    node->declare_parameter("link_scan_maximum_distance", link_scan_maximum_distance);
  }
  node->get_parameter("link_scan_maximum_distance", link_scan_maximum_distance);
  mapper->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance = 3.0;
  if (!node->has_parameter("loop_search_maximum_distance")) {
    node->declare_parameter("loop_search_maximum_distance", loop_search_maximum_distance);
  }
  node->get_parameter("loop_search_maximum_distance", loop_search_maximum_distance);
  mapper->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing = true;
  if (!node->has_parameter("do_loop_closing")) {
    node->declare_parameter("do_loop_closing", do_loop_closing);
  }
  node->get_parameter("do_loop_closing", do_loop_closing);
  mapper->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size = 10;
  if (!node->has_parameter("loop_match_minimum_chain_size")) {
    node->declare_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size);
  }
  node->get_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size);
  mapper->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse = 3.0;
  if (!node->has_parameter("loop_match_maximum_variance_coarse")) {
    node->declare_parameter(
      "loop_match_maximum_variance_coarse",
      loop_match_maximum_variance_coarse);
  }
  node->get_parameter("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse);
  mapper->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse = 0.35;
  if (!node->has_parameter("loop_match_minimum_response_coarse")) {
    node->declare_parameter(
      "loop_match_minimum_response_coarse",
      loop_match_minimum_response_coarse);
  }
  node->get_parameter("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse);
  mapper->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine = 0.45;
  if (!node->has_parameter("loop_match_minimum_response_fine")) {
    node->declare_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine);
  }
  node->get_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine);
  mapper->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters
  double correlation_search_space_dimension = 3.0;
  if (!node->has_parameter("correlation_search_space_dimension")) {
    node->declare_parameter(
      "correlation_search_space_dimension",
      correlation_search_space_dimension);
  }
  node->get_parameter("correlation_search_space_dimension", correlation_search_space_dimension);
  if (correlation_search_space_dimension <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 3.0.");
    correlation_search_space_dimension = 3.0;
  }
  mapper->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution = 0.05;
  if (!node->has_parameter("correlation_search_space_resolution")) {
    node->declare_parameter(
      "correlation_search_space_resolution",
      correlation_search_space_resolution);
  }
  node->get_parameter("correlation_search_space_resolution", correlation_search_space_resolution);
  if (correlation_search_space_resolution <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.05.");
    correlation_search_space_resolution = 0.05;
  }
  mapper->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation = 0.1;
  if (!node->has_parameter("correlation_search_space_smear_deviation")) {
    node->declare_parameter(
      "correlation_search_space_smear_deviation",
      correlation_search_space_smear_deviation);
  }
  node->get_parameter(
    "correlation_search_space_smear_deviation",
    correlation_search_space_smear_deviation);
  if (correlation_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.1.");
    correlation_search_space_smear_deviation = 0.1;
  }
  mapper->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension = 8.0;
  if (!node->has_parameter("loop_search_space_dimension")) {
    node->declare_parameter("loop_search_space_dimension", loop_search_space_dimension);
  }
  node->get_parameter("loop_search_space_dimension", loop_search_space_dimension);
  if (loop_search_space_dimension <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 8.0.");
    loop_search_space_dimension = 8.0;
  }
  mapper->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution = 0.05;
  if (!node->has_parameter("loop_search_space_resolution")) {
    node->declare_parameter("loop_search_space_resolution", loop_search_space_resolution);
  }
  node->get_parameter("loop_search_space_resolution", loop_search_space_resolution);
  if (loop_search_space_resolution <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.05.");
    loop_search_space_resolution = 0.05;
  }
  mapper->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation = 0.03;
  if (!node->has_parameter("loop_search_space_smear_deviation")) {
    node->declare_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation);
  }
  node->get_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation);
  if (loop_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.03.");
    loop_search_space_smear_deviation = 0.03;
  }
  mapper->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters
  double distance_variance_penalty = 0.5;
  if (!node->has_parameter("distance_variance_penalty")) {
    node->declare_parameter("distance_variance_penalty", distance_variance_penalty);
  }
  node->get_parameter("distance_variance_penalty", distance_variance_penalty);
  mapper->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty = 1.0;
  if (!node->has_parameter("angle_variance_penalty")) {
    node->declare_parameter("angle_variance_penalty", angle_variance_penalty);
  }
  node->get_parameter("angle_variance_penalty", angle_variance_penalty);
  mapper->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset = 0.00349;
  if (!node->has_parameter("fine_search_angle_offset")) {
    node->declare_parameter("fine_search_angle_offset", fine_search_angle_offset);
  }
  node->get_parameter("fine_search_angle_offset", fine_search_angle_offset);
  mapper->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset = 0.349;
  if (!node->has_parameter("coarse_search_angle_offset")) {
    node->declare_parameter("coarse_search_angle_offset", coarse_search_angle_offset);
  }
  node->get_parameter("coarse_search_angle_offset", coarse_search_angle_offset);
  mapper->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution = 0.0349;
  if (!node->has_parameter("coarse_angle_resolution")) {
    node->declare_parameter("coarse_angle_resolution", coarse_angle_resolution);
  }
  node->get_parameter("coarse_angle_resolution", coarse_angle_resolution);
  mapper->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty = 0.9;
  if (!node->has_parameter("minimum_angle_penalty")) {
    node->declare_parameter("minimum_angle_penalty", minimum_angle_penalty);
  }
  node->get_parameter("minimum_angle_penalty", minimum_angle_penalty);
  mapper->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty = 0.05;
  if (!node->has_parameter("minimum_distance_penalty")) {
    node->declare_parameter("minimum_distance_penalty", minimum_distance_penalty);
  }
  node->get_parameter("minimum_distance_penalty", minimum_distance_penalty);
  mapper->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion = true;
  if (!node->has_parameter("use_response_expansion")) {
    node->declare_parameter("use_response_expansion", use_response_expansion);
  }
  node->get_parameter("use_response_expansion", use_response_expansion);
  mapper->setParamUseResponseExpansion(use_response_expansion);


  int min_pass_through = 2;
  if (!node->has_parameter("min_pass_through")) {
    node->declare_parameter("min_pass_through", min_pass_through);
  }
  node->get_parameter("min_pass_through", min_pass_through);
  mapper->setParamMinPassThrough(min_pass_through);

  double occupancy_threshold = 0.1;
  if (!node->has_parameter("occupancy_threshold")) {
    node->declare_parameter("occupancy_threshold", occupancy_threshold);
  }
  node->get_parameter("occupancy_threshold", occupancy_threshold);
  mapper->setParamOccupancyThreshold(occupancy_threshold);
}

}  // namespace genesis_icp
