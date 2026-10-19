#include "mid360_mapping/mapping_node.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mid360_mapping
{
MappingNode::MappingNode()
: Node("mapping_node")
{
  const auto source_map_topic =
    declare_parameter<std::string>("fast_lio_map_topic", "/Laser_map");
  const auto source_registered_topic =
    declare_parameter<std::string>("fast_lio_registered_topic", "/cloud_registered");
  const auto source_odom_topic =
    declare_parameter<std::string>("fast_lio_odom_topic", "/Odometry");
  const auto source_path_topic =
    declare_parameter<std::string>("fast_lio_path_topic", "/path");
  map_output_path_ =
    declare_parameter<std::string>("map_output_path", "maps/fastlio_map_from_bag.pcd");

  if (source_map_topic.empty() || source_registered_topic.empty() ||
      source_odom_topic.empty() || source_path_topic.empty() ||
      map_output_path_.empty()) {
    throw std::invalid_argument("Mapping bridge topics and output path must not be empty");
  }

  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/mapping/map", rclcpp::QoS(1).reliable().transient_local());
  registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/mapping/cloud_registered", rclcpp::SensorDataQoS());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mapping/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/mapping/path", rclcpp::QoS(1).reliable().transient_local());

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    source_map_topic, rclcpp::QoS(1).reliable(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      latest_map_ = msg;
      map_pub_->publish(*msg);
    });
  registered_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    source_registered_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      registered_pub_->publish(*msg);
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    source_odom_topic, 10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      odom_pub_->publish(*msg);
    });
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    source_path_topic, 10,
    [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
      path_pub_->publish(*msg);
    });

  save_service_ = create_service<std_srvs::srv::Trigger>(
    "/mapping/save_map",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      saveMap(request, std::move(response));
    });

  RCLCPP_INFO(
    get_logger(),
    "FAST-LIO bridge ready: map=%s odom=%s path=%s; PCD=%s",
    source_map_topic.c_str(), source_odom_topic.c_str(),
    source_path_topic.c_str(), map_output_path_.c_str());
}

void MappingNode::saveMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!latest_map_) {
    response->success = false;
    response->message = "No FAST-LIO map has arrived yet.";
    return;
  }

  try {
    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    pcl::fromROSMsg(*latest_map_, cloud);
    if (cloud.empty()) {
      response->success = false;
      response->message = "The latest FAST-LIO map is empty.";
      return;
    }

    const std::filesystem::path output_path(map_output_path_);
    if (!output_path.parent_path().empty()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    if (pcl::io::savePCDFileBinary(output_path.string(), cloud) != 0) {
      throw std::runtime_error("PCL could not write " + output_path.string());
    }

    response->success = true;
    response->message =
      "Saved " + std::to_string(cloud.size()) + " points to " +
      output_path.string();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } catch (const std::exception & error) {
    response->success = false;
    response->message = error.what();
    RCLCPP_ERROR(get_logger(), "Map save failed: %s", error.what());
  }
}
}  // namespace mid360_mapping
