#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>


#include <CLI/CLI.hpp>
#include <fmt/core.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

namespace fs = std::filesystem;

using namespace pcl;

int main(int argc, char **argv)
{
  CLI::App app("csv poses file to pcd file");

  std::optional<std::string> input_file;
  std::optional<std::string> output_file;

  app.add_option("-i,--input", input_file, "Input file")->required();
  app.add_option("-o,--output", output_file, "Output file")->required();

  CLI11_PARSE(app, argc, argv);

  if (!input_file) {
    ROS_ERROR("input file required");
    throw std::runtime_error("Input file is not specified");
    return -1;
  }
  if (!output_file) {
    ROS_ERROR("output file required");
    throw std::runtime_error("Output file is not specified");
    return -1;
  }

  fs::path input_file_path(input_file->c_str());
  fs::path output_file_path(output_file->c_str());
  if (!fs::exists(input_file_path)) {
    ROS_ERROR("input file does not exist");
    throw std::runtime_error("Input file does not exist");
    return -1;
  }

  fs::path base_dir = output_file_path.parent_path();
  if (base_dir.string() != "") {
    if (!fs::exists(base_dir)) {
      ROS_WARN("output directory does not exist: %s", base_dir.string().c_str());
      fs::create_directories(base_dir);
      ROS_INFO("output directory created: %s", base_dir.string().c_str());
    }
  }

  // read csv file
  std::ifstream csv_file(csv_file_path);
  if (!csv_file.is_open()) {
    ROS_ERROR("Unable to open file %s", input_file_path.string().c_str());
    throw std::runtime_error("Unable to open file");
    return -1;
  }
  std::string line;
  std::vector<std::string> header;
  //   while (std::getline(csv_file, line)) {
  //         std::istringstream ss(line);
  //         std::string cell;
  //         std::getline(ss, cell, ',');
  //         header.push_back(cell);
  //         if (header.size() > 1) {
  //             break;
  //         }
  //         if (header.size() == 1) {
  //             header.push_back("");
  //             break;
}