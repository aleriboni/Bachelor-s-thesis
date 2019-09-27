// PCL specific includes
#include <stdio.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointField.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


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
      //std::cout << in.at(i).getRGBVector3i();
      pcl::PointXYZHSV p;
      PointRGBtoHSV(in.points[i], p);
      out.points.push_back(p);
    }
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
  }
  /*for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << " " << cloud->points[i].x
              << " - "    << cloud->points[i].y
              << " - " << cloud->points[i].z
              << " - "    << cloud->points[i].r
              << " - " << cloud->points[i].g
              << " "    << cloud->points[i].b << std::endl;*/


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
  PcdRGBtoHSV(*cloud, *HSV); // RGB -> HSV

  /*for (size_t i = 0; i < HSV->points.size (); ++i)
      std::cout << " " << HSV->points[i].h  << " - "    << HSV->points[i].s << " - " << HSV->points[i].v << std::endl;*/

  pcl::PointXYZRGB out;
  pcl::PointXYZRGB out1;
  pcl::PointXYZRGB out2;
  pcl::PointXYZRGB out3;

  pcl::PointXYZ segRegion;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);


  //std::cout << " " << HSV->points.size()  << " - "    << cloud->points.size() << std::endl;



  for(size_t i = 0; i < HSV->points.size(); i++){
     //if ( it->z < 2) {

          //std::cout << "h: " << it->h << "  s: " << it->s << "  v: " << it->v << std::endl;
          // std::cout << "z: " << it->z << std::endl;

          //180 < h < 200 intestino e parti esterne

          if ((HSV->points[i].h >= 270 && HSV->points[i].h <= 350)) {
              // std::cout << "  h: " << it->h << "  s: " << it->s << "  v: " << it->v << std::endl;
              //   std::cout << "x: " << it->x << "  y: " << it->y << "  z: " << it->z << "  h: " << it->h << "  s: " << it->s << "  v: " << it->v << std::endl;

              out3.x = HSV->points[i].x;
              out3.y = HSV->points[i].y;
              out3.z = HSV->points[i].z;
              out3.r = cloud->points[i].r;
              out3.g = cloud->points[i].g;
              out3.b = cloud->points[i].b;

              temp_cloud3->push_back(out3);


          }
          else if(HSV->points[i].h > 75 && HSV->points[i].h < 100){

            out2.x = HSV->points[i].x;
            out2.y = HSV->points[i].y;
            out2.z = HSV->points[i].z;
            out2.r = cloud->points[i].r;
            out2.g = cloud->points[i].g;
            out2.b = cloud->points[i].b;

            temp_cloud2->push_back(out2);
          }

          else {//if((HSV->points[i].h > 100 && HSV->points[i].h < 150) && (HSV->points[i].v > 0.7 && HSV->points[i].v < 0.8) ){

            //std::cout << " " << HSV->points[i].h  << " - "    << HSV->points[i].s << " - " << HSV->points[i].v << std::endl;

            out.x = HSV->points[i].x;
            out.y = HSV->points[i].y;
            out.z = HSV->points[i].z;
            out.r = cloud->points[i].r;
            out.g = cloud->points[i].g;
            out.b = cloud->points[i].b;

            temp_cloud1->push_back(out);
          }

  }

  temp_cloud3->width = temp_cloud3->points.size ();
  temp_cloud3->height = 1;
  temp_cloud3->is_dense = true;

  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "color1.pcd";
  writer.write<pcl::PointXYZRGB> (ss.str(), *temp_cloud3, false);

  temp_cloud2->width = temp_cloud2->points.size ();
  temp_cloud2->height = 1;
  temp_cloud2->is_dense = true;

  std::stringstream ss1;
  ss1 << "color2.pcd";
  writer.write<pcl::PointXYZRGB> (ss1.str(), *temp_cloud2, false);

  temp_cloud1->width = temp_cloud1->points.size ();
  temp_cloud1->height = 1;
  temp_cloud1->is_dense = true;

  std::stringstream ss3;
  ss3 << "color3.pcd";
  writer.write<pcl::PointXYZRGB> (ss3.str(), *temp_cloud1, false);

  return 0;
}
