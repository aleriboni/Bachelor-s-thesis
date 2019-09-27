#include <ros/ros.h>
// PCL specific includes
#include <stdio.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointField.h>

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;


void
PointRGBtoHSV(pcl::PointXYZRGB &in, pcl::PointXYZHSV &out) // & &
{

  //salvo i valori min e max tra i tre rgb
  const unsigned char max = std::max (in.r, std::max (in.g, in.b));
  const unsigned char min = std::min (in.r, std::min (in.g, in.b));

  out.x = in.x;
  out.y = in.y;
  out.z = in.z;

  //h -> tonalità, s -> saturazione, v -> chiaro/scuro
  out.v = static_cast <float> (max)/255.f;

  if(max == 0){
    out.s = 0.f;
    out.h = 0.f; // h = -1.f;
    return;
  }

  const float diff = static_cast <float> (max - min);
  out.s = diff / static_cast <float> (max);

  if(min == max){
    out.h = 0; //?????
    return;
  }

  if(max == in.r)
    out.h = 60.f * (static_cast <float> (in.g - in.b) / diff);
  else if(max == in.g)
    out.h = 60.f * (2.f + static_cast <float> (in.b - in.r) / diff);
  else
    out.h = 60.f * (4.f + static_cast <float> (in.r - in.g) / diff); // max == b

  if(out.h < 0.f)
    out.h += 360.f;
}

void
PcdRGBtoHSV (pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZHSV>& out)
{
    out.width = in.width;
    out.height = in.height;

    for(size_t i = 0; i < in.points.size(); i++){
      std::cout << in.at(i).getRGBVector3i();
      pcl::PointXYZHSV p;
      PointRGBtoHSV(in.points[i], p);
      out.points.push_back(p);
    }
}


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "stampa: " << input <<std::endl;
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 output2;
  sensor_msgs::PointCloud2 output3;

  pcl::PCLPointCloud2 pcl_pc2;                                                                //struttura pc2 di pcl
  pcl_conversions::toPCL(*input,pcl_pc2);                                                     //conversione a pcl della pc2

  pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointXYZRGB out;
  pcl::PointXYZRGB out2;
  pcl::PointXYZRGB out3;

  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  pcl::PointXYZ segRegion;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
  PcdRGBtoHSV(*temp_cloud, *HSV); // RGB -> HSV



  for (pcl::PointCloud<pcl::PointXYZHSV>::iterator it = HSV->begin(); it != HSV->end(); it++){
      if ( it->z < 2) {
          out.x = it->x;
          out.y = it->y;
          out.z = it->z;
          //std::cout << "x: " << it->x << "  y: " << it->y << "  z: " << it->z << std::endl;
          // std::cout << "z: " << it->z << std::endl;


          if ((it->h >= 30 && it->h <= 180)) {
              out.r = 255;
              out.g = 255;
              out.b = 255;
              // std::cout << "  h: " << it->h << "  s: " << it->s << "  v: " << it->v << std::endl;
              //   std::cout << "x: " << it->x << "  y: " << it->y << "  z: " << it->z << "  h: " << it->h << "  s: " << it->s << "  v: " << it->v << std::endl;
              segRegion.x = it->x;
              segRegion.y = it->y;
              segRegion.z = it->z;
              cloud_seg->push_back(segRegion);

              out3.x = it->x;
              out3.y = it->y;
              out3.z = it->z;
              out3.r = 255;
              out3.g = 255;
              out3.b = 255;

              temp_cloud3->push_back(out3);


          }
          else{
              out.r = 0;
              out.g = 0;
              out.b = 0;

              out2.x = it->x;
              out2.y = it->y;
              out2.z = it->z;
              out2.r = 0;
              out2.g = 0;
              out2.b = 0;
          }

          temp_cloud1->push_back(out);
          temp_cloud2->push_back(out2);
      }
  }

  pcl::toROSMsg(*temp_cloud1,output); // riscrivo in pointCloud2 di ROS ,cl::toROSMsg(*temp_cloud,output);
  output.header.frame_id = input->header.frame_id  ;

  pcl::toROSMsg(*temp_cloud2,output2); // riscrivo in pointCloud2 di ROS ,cl::toROSMsg(*temp_cloud,output);
  output2.header.frame_id = input->header.frame_id  ;

  pcl::toROSMsg(*temp_cloud3,output3); // riscrivo in pointCloud2 di ROS ,cl::toROSMsg(*temp_cloud,output);
  output3.header.frame_id = input->header.frame_id  ;
  //std::cout << output.width << std::endl;
  // Publish the data.
  pub.publish(output);
  pub2.publish(output2);
  pub3.publish(output3);

}




int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "camera_point_cloud");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("/output", 1);
  pub2 = nh.advertise<pcl::PCLPointCloud2> ("/output2", 1);
  pub3 = nh.advertise<pcl::PCLPointCloud2> ("/output3", 1);

  // Spin
  ros::spin ();
}
