#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <sstream>

#include <CLI/CLI.hpp>
#include <fmt/core.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <internal_use_only/config.hpp>

using PointCloud = sensor_msgs::PointCloud2;
using PointCloudPtr = sensor_msgs::PointCloud2::Ptr;
using PointCloudConstPtr = sensor_msgs::PointCloud2::ConstPtr;

int main(int argc, char **argv)
{
  // 01 parse argument
  auto s = fmt::format(
    "{}:point_cloud_extractor, version: {}", myproject::cmake::project_name, myproject::cmake::project_version);
  CLI::App app{ s };

  bool show_version = false;
  app.add_flag("--version", show_version, "show version information");

  std::optional<std::string> bag_file;
  app.add_option("-b,--bag_file", bag_file, "bag file to extract point cloud from");
  std::optional<std::string> topic;
  app.add_option("-t,--topic", topic, "topic to extract point cloud from");
  std::optional<std::string> output_directory;
  app.add_option("-o,--output_directory", output_directory, "output directory to save point cloud to");

  bool sequence_id = false;
  app.add_flag("--sequence_id", sequence_id, "save point cloud with sequence id");

  bool timestamp = false;
  app.add_flag("--timestamp", timestamp, "save point cloud with timestamp");

  std::optional<unsigned int> id_width;
  app.add_option("-w,--id_width", id_width, "id width of the sequence id");

  std::optional<std::string> prefix;
  app.add_option("-p,--prefix", prefix, "prefix of the output file name");

  CLI11_PARSE(app, argc, argv);

  if (show_version) {
    ROS_INFO("{}", myproject::cmake::project_version);
    return EXIT_SUCCESS;
  }

  if (!bag_file || !topic || !output_directory) {
    ROS_ERROR("bag_file, topic and output_directory are required");
    return EXIT_FAILURE;
  }

  if (!sequence_id && !timestamp) {
    ROS_ERROR("sequence_id and timestamp cannot be both false");
    ROS_ERROR("you should specify one and only one naming rule");
    return EXIT_FAILURE;
  }

  if (sequence_id && timestamp) {
    ROS_ERROR("sequence_id and timestamp cannot be both true");
    ROS_ERROR("you should specify one and only one naming rule");
    return EXIT_FAILURE;
  }

  // check if bag file exists
  if (!std::filesystem::exists(*bag_file)) {
    ROS_ERROR("bag file %s does not exist", bag_file->c_str());
    return EXIT_FAILURE;
  }

  // check if the output directory exists
  if (!std::filesystem::exists(*output_directory)) {
    ROS_INFO("output directory does not exist, creating it");
    // create the output directory
    if (!std::filesystem::create_directories(*output_directory)) {
      ROS_ERROR("failed to create output directory: %s", output_directory->c_str());
      return EXIT_FAILURE;
    } else {
      ROS_INFO("output directory created successfully");
    }
  }

  // 02 read bag file
  rosbag::Bag bag;
  bag.open(*bag_file, rosbag::bagmode::Read);
  ROS_INFO("bag file opened successfully");

  std::vector<std::string> topics;
  topics.push_back(std::string(*topic));

  rosbag::View view(bag, rosbag::TopicQuery(topics));


  uint64_t count = 0;

  for (const rosbag::MessageInstance m : view) {
    PointCloudConstPtr s = m.instantiate<PointCloud>();
    if (s != nullptr) {
      // pcl::PointCloud<pcl::PointXYZ> cloud;
      // // Convert the ROS PointCloud2 message to PCL point cloud
      // pcl::fromROSMsg(*s, cloud);
      std::stringstream ss;
      if (timestamp) {
        ss << *output_directory << "/" << s->header.stamp << ".pcd";
      } else if (sequence_id) {
        ss << *output_directory << "/" << std::setfill('0') << std::setw(*id_width) << count << ".pcd";
      }
      pcl::io::savePCDFile(ss.str(), *s, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
      ROS_INFO("point cloud saved to: %s successful!", ss.str().c_str());
      count++;
      // pcl::io::savePCDFileBinary("my_point_cloud.pcd", *s);
    }
  }

  bag.close();
}