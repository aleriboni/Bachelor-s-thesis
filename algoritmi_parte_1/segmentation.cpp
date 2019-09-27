#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <stdio.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointField.h>


ros::Publisher pub_color;
ros::Publisher pub_white;
ros::Publisher pub_black;
ros::Publisher pub_cluster;


// Funzione di supporto che converte il singolo punto RGB in HSV
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
    out.h = 0;
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

//Funzione che converte la PintCloud RGB in HSV
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


// Funzione che segmenta la PointCloud in base ai colori della scena
void
color_segmentation(pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered, const sensor_msgs::PointCloud2ConstPtr& input)
{

  // Dichiarazione delle variabili
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 output_white;
  sensor_msgs::PointCloud2 output_black;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_black(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Punti appartenenti allaregione segmentata (white)
  pcl::PointXYZ segRegion;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointXYZRGB out;
  pcl::PointXYZRGB out_black;

  //temp_cloud punta alla cloud_filtered, ovvero la scena di input filtrata.
  *temp_cloud = cloud_filtered;

  //conversione in HSV per segmentare la scena
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
  PcdRGBtoHSV(*temp_cloud, *HSV); // RGB -> HSV


  for (pcl::PointCloud<pcl::PointXYZHSV>::iterator it = HSV->begin(); it != HSV->end(); it++){
      if ( it->z < 2) {
          out.x = it->x;
          out.y = it->y;
          out.z = it->z;

          if ((it->h >= 30 && it->h <= 180)) {  // punti della regione segmentata (white)
            out.r = 255;
            out.g = 255;
            out.b = 255;
            segRegion.x = it->x;
            segRegion.y = it->y;
            segRegion.z = it->z;
            cloud_seg->push_back(segRegion);

          }
          else {    // punti esterni alla regione segmentata (black)
            out.r = 0;
            out.g = 0;
            out.b = 0;

            out_black.x = it->x;
            out_black.y = it->y;
            out_black.z = it->z;
            out_black.r = 0;
            out_black.g = 0;
            out_black.b = 0;
          }

        temp_cloud2->push_back(out);
        temp_cloud_black->push_back(out_black);
      }
  }

  pcl::toROSMsg(*temp_cloud2,output); //scena intera segmentata in base al colore
  output.header.frame_id = input->header.frame_id;
  pub_color.publish(output);

  pcl::toROSMsg(*cloud_seg,output_white); //parte della scena segmentata (white)
  output_white.header.frame_id = input->header.frame_id;
  pub_white.publish(output_white);

  pcl::toROSMsg(*temp_cloud_black,output_black); //parte non segmentata (black)
  output_black.header.frame_id = input->header.frame_id;
  pub_black.publish(output_black);

}

// Funzione che segmenta la PointCloud in cluster
void
cluster_segmentation(pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered, const sensor_msgs::PointCloud2ConstPtr& input) {

  sensor_msgs::PointCloud2 output_cluster;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDWriter writer;
  // trasformo la PointCloud XYZRGB in PointCloud XYZ (non ho più bisogno del colore)
  copyPointCloud(cloud_filtered, *cloud_filtered_xyz);

  // Utilizzo KdTree, cerca i punti vicini che appartengono allo stesso cluster
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered_xyz);

  // Setto i paramentri dell'algoritmo
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm per le due tazze, 5 cm per 4 oggetti
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered_xyz);
  ec.extract (cluster_indices);

  int j = 0;

  //ciclo for che crea, setta i paramentri e salva su disco ogni cluster presente nella scena
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered_xyz->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;

    pcl::toROSMsg(*cloud_cluster,output_cluster);
    output_cluster.header.frame_id = input->header.frame_id  ;
    pub_cluster.publish(output_cluster);

  }

}


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  //dichiarazione variabili
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);


  //conversione in PCL
  pcl_conversions::toPCL(*input,pcl_pc2);
  //conversione tipo dei campi (dovuto a aggiornamenti di ros)
  pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  //conversione da PCL a PCL2
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);


  //salvo la pcd su disco
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "cloud_scena.pcd";
  writer.write<pcl::PointXYZRGB> (ss.str(), *cloud, false);

  // Lettura della PointCloud
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Creazione del filtro VoxelGrid (filtro minimo perchè al momento non necessario)
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f); //distanza tra i punti (cioè la densità dei punti)
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Creazione oggetto segmentazione per il modello planare e settaggio dei paramentri
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100); // Limite superiore
  seg.setDistanceThreshold (0.02);

  int i = 0;
  int nr_points = (int) cloud_filtered->points.size(); //numero di punti della scena

  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Estrazione dei planar inliers dalla PointCloud di input
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;


    // Rimozione dei planar inliers, estrazione del resto
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f; // cloud_filtered contiene la scena senza il pavimento

  }

 //Due metodi di segmentazione: color e cluster. Possibile implementarlo con un flag che indica quale dei due usare
 color_segmentation(*cloud_filtered, input);
 cluster_segmentation(*cloud_filtered, input);

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "camera_point_cloud");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("/points2", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub_color = nh.advertise<pcl::PCLPointCloud2> ("/output_color", 1);
  pub_white = nh.advertise<pcl::PCLPointCloud2> ("/output_white", 1);
  pub_black = nh.advertise<pcl::PCLPointCloud2> ("/output_black", 1);
  pub_cluster = nh.advertise<pcl::PCLPointCloud2> ("/output_cluster", 1);

  // Spin
  ros::spin ();
}
