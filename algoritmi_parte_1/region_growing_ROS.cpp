#include <ros/ros.h>
#include <stdio.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>


ros::Publisher pub_color;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_conversions::toPCL(*input,pcl_pc2);
  //conversione tipo dei campi (dovuto a aggiornamenti di ros)
  pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  //conversione da PCL a PCL2
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }/*
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*colored_cloud, output); //scena intera segmentata in base al colore
  output.header.frame_id = input->header.frame_id;
  pub_color.publish(output);
*/
}


int
main (int argc, char** argv)
{
  //signal(SIGINT, salvaPCD);
  // Initialize ROS
  ros::init (argc, argv, "camera_point_cloud");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("/points2", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_cb);

  pub_color = nh.advertise<pcl::PCLPointCloud2> ("/output", 1);

  // Spin
  ros::spin ();
}
