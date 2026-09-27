#include <iostream>
#include <string>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{

struct Options
{
  std::string input;
  std::string output;
  double z_min = 0.03;
  double z_max = 1.0;
  double voxel_size = 0.0;
  double radius = 0.0;
  int min_neighbors = 0;
};

bool readOption(int argc, char ** argv, const std::string & key, std::string & value)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == key) {
      value = argv[i + 1];
      return true;
    }
  }
  return false;
}

bool readOption(int argc, char ** argv, const std::string & key, double & value)
{
  std::string text;
  if (!readOption(argc, argv, key, text)) {
    return false;
  }
  value = std::stod(text);
  return true;
}

bool readOption(int argc, char ** argv, const std::string & key, int & value)
{
  std::string text;
  if (!readOption(argc, argv, key, text)) {
    return false;
  }
  value = std::stoi(text);
  return true;
}

void printUsage(const char * program)
{
  std::cerr
    << "Usage:\n"
    << "  " << program
    << " --input INPUT.pcd --output OUTPUT.pcd [--z-min 0.03] [--z-max 1.0]"
    << " [--voxel-size 0.0] [--radius 0.0] [--min-neighbors 0]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  readOption(argc, argv, "--input", options.input);
  readOption(argc, argv, "--output", options.output);
  readOption(argc, argv, "--z-min", options.z_min);
  readOption(argc, argv, "--z-max", options.z_max);
  readOption(argc, argv, "--voxel-size", options.voxel_size);
  readOption(argc, argv, "--radius", options.radius);
  readOption(argc, argv, "--min-neighbors", options.min_neighbors);

  if (options.input.empty() || options.output.empty()) {
    printUsage(argv[0]);
    return 2;
  }
  if (options.z_min > options.z_max) {
    std::cerr << "Invalid z range: z-min > z-max\n";
    return 2;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(options.input, *input_cloud) != 0) {
    std::cerr << "Failed to load input PCD: " << options.input << "\n";
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(options.z_min, options.z_max);
  pass.filter(*sliced_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud = sliced_cloud;

  if (options.voxel_size > 0.0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(processed_cloud);
    const auto leaf = static_cast<float>(options.voxel_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*voxel_cloud);
    processed_cloud = voxel_cloud;
  }

  if (options.radius > 0.0 && options.min_neighbors > 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr radius_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;
    radius_filter.setInputCloud(processed_cloud);
    radius_filter.setRadiusSearch(options.radius);
    radius_filter.setMinNeighborsInRadius(options.min_neighbors);
    radius_filter.filter(*radius_cloud);
    processed_cloud = radius_cloud;
  }

  if (pcl::io::savePCDFileBinary(options.output, *processed_cloud) != 0) {
    std::cerr << "Failed to save output PCD: " << options.output << "\n";
    return 1;
  }

  std::cout
    << "Saved sliced PCD: " << options.output
    << " points=" << processed_cloud->size()
    << " original=" << input_cloud->size()
    << " z=[" << options.z_min << ", " << options.z_max << "]\n";
  return 0;
}
