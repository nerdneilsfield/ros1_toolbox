#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>


#include <CLI/CLI.hpp>
#include <fmt/core.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <internal_use_only/config.hpp>

namespace fs = std::filesystem;

using NavFixMsg = sensor_msgs::NavSatFix;
using NavSatStatusMsg = sensor_msgs::NavSatStatus;
using NavFixMsgPtr = sensor_msgs::NavSatFix::Ptr;
using NavSatStatusMsgPtr = sensor_msgs::NavSatStatus::Ptr;
using NavFixMsgConstPtr = sensor_msgs::NavSatFix::ConstPtr;
using NavSatStatusMsgConstPtr = sensor_msgs::NavSatStatus::ConstPtr;

int main(int argc, char **argv)
{
  auto s =
    fmt::format("{}:navfix_to_csv, version: {}", myproject::cmake::project_name, myproject::cmake::project_version);
  CLI::App app{ s };

  bool show_version = false;
  app.add_flag("--version", show_version, "show version information");

  std::optional<std::string> bag_file;
  app.add_option("-b,--bag_file", bag_file, "bag file to extract navfix information from");
  std::optional<std::string> topic;
  app.add_option("-t,--topic", topic, "topic to extract navfix information from");
  std::optional<std::string> output_filename;
  app.add_option("-o,--output_filename", output_filename, "output csv filename to save navfix information to");


  CLI11_PARSE(app, argc, argv);

  if (show_version) {
    ROS_INFO("{}", myproject::cmake::project_version);
    return EXIT_SUCCESS;
  }

  if (!bag_file || !topic || !output_filename) {
    ROS_ERROR("bag_file, topic and output_filename are required");
    return EXIT_FAILURE;
  }

  fs::path bag_path(bag_file->c_str());
  if (!fs::exists(bag_path)) { ROS_ERROR("bag file does not exist: %s", bag_path.string().c_str()); }

  // get the base directory from filename
  fs::path csv_file_path(output_filename->c_str());
  fs::path base_dir = csv_file_path.parent_path();
  if (base_dir.string() != "") {
    if (!fs::exists(base_dir)) {
      ROS_WARN("base directory does not exist: %s", base_dir.string().c_str());
      fs::create_directories(base_dir);
      ROS_INFO("base directory created: %s", base_dir.string().c_str());
    }
  }

  rosbag::Bag bag;
  bag.open(*bag_file, rosbag::bagmode::Read);
  ROS_INFO("bag_file: %s opened", bag_file->c_str());

  std::vector<std::string> topics;
  topics.push_back(topic->c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // open file for writing
  std::filebuf fb;
  fb.open(csv_file_path, std::ios::out);
  std::ostream os(&fb);

  os << std::setprecision(9);

  os << "ID,TimeStamp,FrameId,Status,Service,Latitude,Longitude,Altitude,PositionCovarianceType,";
  os << "PC[0],PC[1],PC[2],PC[3],PC[4],PC[5],PC[6],PC[7],PC[8]\n";

  uint64_t count = 0;
  for (const rosbag::MessageInstance m : view) {

    NavFixMsgConstPtr s = m.instantiate<NavFixMsg>();
    if (s != nullptr) {
      os << count << "," << s->header.stamp << ",";
      os << s->header.frame_id << ",";
      os << static_cast<int32_t>(s->status.status) << "," << s->status.service << ",";
      os << s->latitude << "," << s->longitude << "," << s->altitude << ",";
      os << static_cast<int32_t>(s->position_covariance_type) << ",";
      os << s->position_covariance[0] << "," << s->position_covariance[1] << "," << s->position_covariance[2] << ",";
      os << s->position_covariance[3] << "," << s->position_covariance[4] << "," << s->position_covariance[5] << ",";
      os << s->position_covariance[6] << "," << s->position_covariance[7] << "," << s->position_covariance[8] << "\n";
      ROS_INFO("%d navfix information with %d, lat: %f, long: %f", count, s->status.status, s->latitude, s->longitude);
      count++;
    }
  }

  //   os.close();
  fb.close();
  bag.close();
}
