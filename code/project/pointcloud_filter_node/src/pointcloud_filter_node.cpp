#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>

ros::Publisher pub;
pcl::VoxelGrid<pcl::PointXYZ> voxel;


/**
 * @brief StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data. The algorithm iterates through the entire input twice: During the first iteration it will compute the average distance that each point has to its nearest k neighbors. The value of k can be set using setMeanK(). Next, the mean and standard deviation of all these distances are computed in order to determine a distance threshold. The distance threshold will be equal to: mean + stddev_mult * stddev. The multiplier for the standard deviation can be set using setStddevMulThresh(). During the next iteration the points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
 * @param input
 */
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input) {
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.setInputCloud(cloud);
  voxel.filter(*cloud);

  ROS_INFO_STREAM("[Size] pointcloud: " << cloud->size());
  // Create the filtering object
  auto start_time = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
  sor.setInputCloud(cloud);
  sor.setMeanK(25);
  sor.setStddevMulThresh(0.5);
  sor.filter(*cloud_filtered);
  auto end_time = std::chrono::high_resolution_clock::now();
  float total_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();
  ROS_INFO_STREAM("[Time] " << total_time << " [ms]\n");

  // Create a pcl::PointXYZRGB cloud for visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_visual->header = cloud->header;
  for (auto &point: *cloud_filtered) {
    pcl::PointXYZRGB point_rgb;
    point_rgb.x = point.x;
    point_rgb.y = point.y;
    point_rgb.z = point.z;
    point_rgb.r = 255; // Non-noisy points in red
    point_rgb.g = 0;
    point_rgb.b = 0;
    cloud_visual->points.push_back(point_rgb);
  }

  // 可视化离群点（白色）
  // pcl::PointIndices pcl_indices;
  // sor.getRemovedIndices(pcl_indices);
  // auto &indices = pcl_indices.indices;
  //
  // for (int index: indices) {
  //   auto point = cloud->at(index);
  //   pcl::PointXYZRGB point_rgb;
  //   point_rgb.x = point.x;
  //   point_rgb.y = point.y;
  //   point_rgb.z = point.z;
  //   point_rgb.r = 255; // noisy points in white
  //   point_rgb.g = 255;
  //   point_rgb.b = 255;
  //   cloud_visual->points.push_back(point_rgb);
  // }

  // Convert back to sensor_msgs::PointCloud2 for publication
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_visual, output);
  pub.publish(output);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "point_cloud_filter");
  ros::NodeHandle nh;


  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filter", 1, true);

  ros::spin();
  return 0;
}
