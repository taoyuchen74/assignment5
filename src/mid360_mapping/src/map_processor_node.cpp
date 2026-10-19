#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class MapProcessorNode : public rclcpp::Node
{
public:
  MapProcessorNode()
  : Node("map_processor_node")
  {
    const auto input =
      declare_parameter<std::string>("input_topic", "/mapping/map");

    map_frame_ =
      declare_parameter<std::string>("map_frame", "map");

    output_directory_ =
      declare_parameter<std::string>("output_directory", "maps");

    voxel_size_ =
      declare_parameter<double>("voxel_size", 0.10);

    grid_resolution_ =
      declare_parameter<double>("grid_resolution", 0.10);

    obstacle_height_min_ =
      declare_parameter<double>("obstacle_height_min", 0.15);

    obstacle_height_max_ =
      declare_parameter<double>("obstacle_height_max", 1.50);

    outlier_mean_k_ =
      declare_parameter<int>("outlier_mean_k", 20);
    outlier_stddev_mul_ =
      declare_parameter<double>("outlier_stddev_mul", 1.0);

    if (output_directory_.empty()) {
      output_directory_ = "maps";
    }

    if (
      input.empty() || map_frame_.empty() ||
      !std::isfinite(voxel_size_) || voxel_size_ <= 0.0 ||
      !std::isfinite(grid_resolution_) || grid_resolution_ <= 0.0 ||
      !std::isfinite(obstacle_height_min_) ||
      !std::isfinite(obstacle_height_max_) ||
      obstacle_height_max_ <= obstacle_height_min_ ||
      outlier_mean_k_ < 2 ||
      !std::isfinite(outlier_stddev_mul_) || outlier_stddev_mul_ <= 0.0)
    {
      throw std::invalid_argument("Invalid map-processing startup parameters");
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        try {
          processCloud(*msg);
        } catch (const std::exception & error) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Map processing failed: %s", error.what());
        }
      });

    cleaned_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "mapping/map_filtered",
        rclcpp::QoS(1).reliable().transient_local());

    grid_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(
        "mapping/map_2d",
        rclcpp::QoS(1).reliable().transient_local());

    save_service_ = create_service<std_srvs::srv::Trigger>(
      "mapping/save_artifacts",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        saveArtifacts(*response);
      });

    RCLCPP_INFO(
      get_logger(),
      "Map processor started. input=%s voxel=%.3f grid=%.3f",
      input.c_str(), voxel_size_, grid_resolution_);
  }

private:

  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  void processCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    ++received_;

    Cloud::Ptr input_cloud(new Cloud);
    pcl::fromROSMsg(msg, *input_cloud);

    if (input_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received an empty point cloud");
      return;
    }

    // Remove invalid XYZ coordinates before filtering.
    std::vector<int> valid_indices;
    Cloud::Ptr valid_cloud(new Cloud);
    pcl::removeNaNFromPointCloud(
      *input_cloud, *valid_cloud, valid_indices);

    if (valid_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No finite points remain in the input cloud");
      return;
    }

    // Reduce the point count first; this makes statistical filtering cheaper.
    Cloud::Ptr downsampled(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(valid_cloud);
    voxel_filter.setLeafSize(
      static_cast<float>(voxel_size_),
      static_cast<float>(voxel_size_),
      static_cast<float>(voxel_size_));
    voxel_filter.filter(*downsampled);

    if (downsampled->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "The voxel filter produced an empty cloud");
      return;
    }

    // Remove sparse points that are far from their local neighbors.
    Cloud::Ptr cleaned(new Cloud);
    if (downsampled->size() > static_cast<std::size_t>(outlier_mean_k_)) {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
      outlier_filter.setInputCloud(downsampled);
      outlier_filter.setMeanK(outlier_mean_k_);
      outlier_filter.setStddevMulThresh(outlier_stddev_mul_);
      outlier_filter.filter(*cleaned);
    } else {
      *cleaned = *downsampled;
    }

    if (cleaned->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Outlier filtering produced an empty cloud");
      return;
    }

    latest_cloud_ = cleaned;

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cleaned, output_msg);
    output_msg.header = msg.header;
    if (output_msg.header.frame_id.empty()) {
      output_msg.header.frame_id = map_frame_;
    }

    cleaned_pub_->publish(output_msg);
    buildOccupancyGrid(*cleaned, output_msg.header);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Processed snapshot %llu: input=%zu finite=%zu voxel=%zu cleaned=%zu",
      static_cast<unsigned long long>(received_),
      input_cloud->size(),
      valid_cloud->size(),
      downsampled->size(),
      cleaned->size());
  }

  void buildOccupancyGrid(
    const Cloud & cloud,
    const std_msgs::msg::Header & header)
  {
    if (cloud.empty()) {
      return;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }

    if (!std::isfinite(min_x) || !std::isfinite(min_y) ||
        !std::isfinite(max_x) || !std::isfinite(max_y)) {
      return;
    }

    // The maximum point must fit in a cell too, hence floor(range / res) + 1.
    const double width_cells =
      std::floor((max_x - min_x) / grid_resolution_) + 1.0;
    const double height_cells =
      std::floor((max_y - min_y) / grid_resolution_) + 1.0;
    const double max_dimension =
      static_cast<double>(std::numeric_limits<std::uint32_t>::max());

    if (width_cells < 1.0 || height_cells < 1.0 ||
        width_cells > max_dimension || height_cells > max_dimension) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Map bounds are too large for an OccupancyGrid");
      return;
    }

    const auto width = static_cast<std::uint32_t>(width_cells);
    const auto height = static_cast<std::uint32_t>(height_cells);
    const std::uint64_t cell_count =
      static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
    constexpr std::uint64_t kMaximumCells = 50000000;
    if (cell_count > kMaximumCells) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "OccupancyGrid would contain %llu cells; refusing excessive allocation",
        static_cast<unsigned long long>(cell_count));
      return;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header = header;
    if (grid.header.frame_id.empty()) {
      grid.header.frame_id = map_frame_;
    }
    grid.info.map_load_time = header.stamp;
    grid.info.resolution = static_cast<float>(grid_resolution_);
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // Unknown cells stay -1. A pointcloud snapshot does not carry enough
    // ray-visibility information to mark unseen cells as free safely.
    grid.data.assign(static_cast<std::size_t>(cell_count), -1);

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z) ||
          point.z < obstacle_height_min_ ||
          point.z > obstacle_height_max_) {
        continue;
      }

      const double cell_x =
        std::floor((static_cast<double>(point.x) - min_x) / grid_resolution_);
      const double cell_y =
        std::floor((static_cast<double>(point.y) - min_y) / grid_resolution_);
      if (cell_x < 0.0 || cell_y < 0.0 ||
          cell_x >= static_cast<double>(width) ||
          cell_y >= static_cast<double>(height)) {
        continue;
      }

      const auto index =
        static_cast<std::size_t>(cell_y) * width +
        static_cast<std::size_t>(cell_x);
      grid.data[index] = 100;
    }

    grid_pub_->publish(grid);
    latest_grid_ = std::move(grid);
  }

  void saveArtifacts(
    std_srvs::srv::Trigger::Response & response)
  {
    if (!latest_cloud_ || latest_cloud_->empty() ||
        latest_grid_.info.width == 0 ||
        latest_grid_.info.height == 0 ||
        latest_grid_.data.empty()) {
      response.success = false;
      response.message = "No processed point cloud and grid are available yet.";
      return;
    }

    try {
      const std::filesystem::path output_dir(output_directory_);
      std::filesystem::create_directories(output_dir);

      const auto pcd_path = output_dir / "cleaned_map.pcd";
      const auto pgm_path = output_dir / "map.pgm";
      const auto yaml_path = output_dir / "map.yaml";

      if (pcl::io::savePCDFileBinary(
            pcd_path.string(), *latest_cloud_) != 0) {
        throw std::runtime_error("Failed to save PCD: " + pcd_path.string());
      }

      savePGM(pgm_path.string());
      saveYAML(yaml_path.string(), pgm_path.filename().string());

      response.success = true;
      response.message =
        "Saved " + pcd_path.string() + ", " +
        pgm_path.string() + ", " + yaml_path.string();
      RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
      RCLCPP_ERROR(get_logger(), "Save failed: %s", error.what());
    }
  }

  void savePGM(const std::string & path)
  {
    std::ofstream file(path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Failed to open PGM: " + path);
    }

    const auto width = latest_grid_.info.width;
    const auto height = latest_grid_.info.height;
    file << "P5\n" << width << " " << height << "\n255\n";

    // OccupancyGrid rows start at the map origin; PGM rows start at the top.
    for (int y = static_cast<int>(height) - 1; y >= 0; --y) {
      for (std::uint32_t x = 0; x < width; ++x) {
        const auto index = static_cast<std::size_t>(y) * width + x;
        const auto value = latest_grid_.data[index];
        unsigned char pixel = 205;  // unknown: gray
        if (value >= 65) {
          pixel = 0;                // occupied: black
        } else if (value >= 0) {
          pixel = 254;              // free: nearly white
        }
        file.write(reinterpret_cast<const char *>(&pixel), 1);
      }
    }

    file.flush();
    if (!file.good()) {
      throw std::runtime_error("Failed while writing PGM: " + path);
    }
  }

  void saveYAML(
    const std::string & path,
    const std::string & image_name)
  {
    std::ofstream file(path);
    if (!file) {
      throw std::runtime_error("Failed to open YAML: " + path);
    }

    file << std::setprecision(9)
         << "image: " << image_name << "\n"
         << "resolution: " << latest_grid_.info.resolution << "\n"
         << "origin: ["
         << latest_grid_.info.origin.position.x << ", "
         << latest_grid_.info.origin.position.y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
    file.flush();

    if (!file.good()) {
      throw std::runtime_error("Failed while writing YAML: " + path);
    }
  }

  std::uint64_t received_{0};

  std::string map_frame_;
  std::string output_directory_;

  double voxel_size_{0.10};
  double grid_resolution_{0.10};
  double obstacle_height_min_{0.15};
  double obstacle_height_max_{1.50};
  int outlier_mean_k_{20};
  double outlier_stddev_mul_{1.0};

  Cloud::Ptr latest_cloud_;

  nav_msgs::msg::OccupancyGrid latest_grid_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr cleaned_pub_;

  rclcpp::Publisher<
    nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr save_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int result = 0;

  try {
    rclcpp::spin(
      std::make_shared<MapProcessorNode>());
  }
  catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("map_processor_node"),
      "%s",
      error.what());

    result = 1;
  }

  rclcpp::shutdown();

  return result;
}
