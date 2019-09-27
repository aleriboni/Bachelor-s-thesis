#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <pcl/point_cloud.h>

int
main (int argc, char** argv)
{
/*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);
*/

  pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud2, cloud3;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the first file pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], cloud2) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the second file pcd \n");
    return (-1);
  }
/*
  cloud.fields[3].datatype = pcl::PCLPointField::FLOAT32;
  cloud2.fields[3].datatype = pcl::PCLPointField::FLOAT32;
  cloud3.fields[3].datatype = pcl::PCLPointField::FLOAT32;

  cloud3->height = 1;
  cloud3->width = cloud->points.size() + cloud2->points.size();
  cloud3->is_dense = true;
*/

  cloud += cloud2;
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> (argv[3], cloud, false);

  return (0);
}
