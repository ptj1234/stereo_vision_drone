#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"

void
optimizeMap(mocka::Maps::BasicInfo& in)
{
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width  = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
    {
      temp->push_back(i);
    }
  }
  for (int i = temp->size() - 1; i >= 0; i--)
  {
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i)); // erasing the enclosed points
  }
  in.cloud->width -= temp->size();

  pcl::toROSMsg(*in.cloud, *in.output);
  in.output->header.frame_id = "world";
  RCLCPP_INFO(in.nh_private->get_logger(), "finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh_private = rclcpp::Node::make_shared("mockamap");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(nh_private);

  auto pcl_pub =
    nh_private->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::msg::PointCloud2       output;
  // Fill in the cloud data

  int seed;

  int sizeX;
  int sizeY;
  int sizeZ;

  double scale;
  double update_freq;

  int type;

  param_helper::param(nh_private, "seed", seed, 4546);
  param_helper::param(nh_private, "update_freq", update_freq, 1.0);
  param_helper::param(nh_private, "resolution", scale, 0.38);
  param_helper::param(nh_private, "x_length", sizeX, 100);
  param_helper::param(nh_private, "y_length", sizeY, 100);
  param_helper::param(nh_private, "z_length", sizeZ, 10);

  param_helper::param(nh_private, "type", type, 1);

  scale = 1 / scale;
  sizeX = sizeX * scale;
  sizeY = sizeY * scale;
  sizeZ = sizeZ * scale;

  mocka::Maps::BasicInfo info;
  info.nh_private = nh_private;
  info.sizeX      = sizeX;
  info.sizeY      = sizeY;
  info.sizeZ      = sizeZ;
  info.seed       = seed;
  info.scale      = scale;
  info.output     = &output;
  info.cloud      = &cloud;

  mocka::Maps map;
  map.setInfo(info);
  map.generate(type);

  //  optimizeMap(info);

  //! @note publish loop
  rclcpp::WallRate loop_rate(update_freq);
  while (rclcpp::ok())
  {
    pcl_pub->publish(output);
    executor->spin_all(std::chrono::milliseconds(0));
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
