#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <sensor_msgs/PointField.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string models[5];
int num_models;
std::string MODELLO;

//Algorithm parameters (flags used to know what use)
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_hough_ (true); //decido che, tra tutti, uso l'algoritmo hough
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

// Helper, guida per l'utilizzo del programma
void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

// Funzione che analizza i parametri passati a riga di comando e setta i paramentri degli algoritmi
// in base a cosa vuole l'utente
void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () == 0)//PRIMA ERA 2! MA HO TOLTO IL MODELLO DELLA SCENA QUINDI ADESSO E' 1. GIUSTO?????????
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }
  //std::cout << "Nr di modelli: " << filenames.size();
  num_models = filenames.size();
  for(int i = 0; i < filenames.size(); i++){
    models[i] = argv[filenames[i]];
  }
  //model_filename_ = argv[filenames[0]];

  // Comportamento del programma
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    show_keypoints_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    show_correspondences_ = true;
  }

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_ = true;
    }else if (used_algorithm.compare ("GC") == 0)
    {
      use_hough_ = false;
    }
    else
    {
      std::cout << "Wrong algorithm name.\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
}


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{


    //std::cout << "\nnr modelli: " << num_models << "\n" << models[0] << models[1];
    // Dichiarazioni delle variabili del modello e della scena
    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());//-------------------------- model
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());//-------------------------- scene
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());//---------------- model_normals
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());//---------------- scene_normals
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());//---- model_descriptor
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());//---- scene_descriptor


    //------------------------------ METTO LA SCENA IN UNA VARIABILE (scene)
    //conversione in PCL
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    //conversione tipo dei campi (dovuto a aggiornamenti di ros)
    pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    //conversione da PCL a PCL2
    pcl::fromPCLPointCloud2(pcl_pc2,*scene); //cloud in questo momento contiene la scena intera


    //------------------------------ METTO IL MODELLO DA CLASSIFICARE IN UNA VARIABILE (model)
    //
    //  Caricamento del modello (oggetto segmentato) e della scena
    //
    if (pcl::io::loadPCDFile (MODELLO, *model) < 0){ //model_filename_ settato durante parseCommandLine (argc, argv)
      std::cout << "Error loading model cloud." << std::endl;
      //showHelp (argv[0]); //IN REALTA' NON SAREBBE COMMENTATO, COSA FARE?????????????????????????????????''
      return;
    }


    //------------------------------ NORMALI IN OGNI PUNTO DEL MODELLO DA CLASSIFICARE E DELLA SCENA
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);
    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);


    //------------------------------ DETERMINO KEYPOINTS PER MODELLO E SCENA
    // Scompone ogni cloud per trovare un piccolo numero di punti chiave che saranno
    // associati a un descrittore 3D per eseguire la corrispondenza dei punti chiave e
    // determinare le corrispondenze punto a punto
    //
    pcl::PointCloud<int> sampled_indices;
    //modello
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.compute(sampled_indices);
    pcl::copyPointCloud(*model, sampled_indices.points, *model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
    //scena
    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.compute(sampled_indices);
    pcl::copyPointCloud(*scene, sampled_indices.points, *scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;



    //------------------------------ CREAZIONE DESCRITTORI 3D SUI KEYPOINTS DI MODELLO E SCENA
    //  Assegna un descrittore 3D a ciascun punto chiave della scena e del modello
    // (codice modificato rispetto a quello originale)
    //
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);
    //modello
    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);
    //scena
    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);



    //------------------------------ METTO IN CORRISPONDENZA I KEYPOINTS DEI DESCRITTORI DI SCENA E MODELLO
    // Corrispondenza punto a punto tra i descrittori del modello e quelli della scena
    //
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (model_descriptors);

    //per ogni keypoints nella scena, trova il keypoint più vicino nel modello e aggiungilo nel vettore delle corrispondenze
    for (size_t i = 0; i < scene_descriptors->size (); ++i){
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])){ //skipping NaNs
        continue;
      }
      int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);

      // parametro neigh_sqr_dists[0] modificato rispetto a quello di partenza (0,25f)
      // add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      if(found_neighs == 1 && neigh_sqr_dists[0] < 0.02f){
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }//fine for
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;




    //----------------------------- SEGMENTAZIONE CON CLUSTERIZZAZIONE
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D (utilizza i keypoints, le normali e la PointCloud)
    if (use_hough_){
      //abbinamento di modello e scena considerando valori tipici di Hugh (keypoints, le normali e la PointCloud)
      pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
      pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

      pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
      rf_est.setFindHoles (true);
      rf_est.setRadiusSearch (rf_rad_);
      //modello
      rf_est.setInputCloud (model_keypoints);
      rf_est.setInputNormals (model_normals);
      rf_est.setSearchSurface (model);
      rf_est.compute (*model_rf);
      //scena
      rf_est.setInputCloud (scene_keypoints);
      rf_est.setInputNormals (scene_normals);
      rf_est.setSearchSurface (scene);
      rf_est.compute (*scene_rf);

      //  Clustering
      pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
      clusterer.setHoughBinSize (cg_size_);
      clusterer.setHoughThreshold (cg_thresh_);
      clusterer.setUseInterpolation (true);
      clusterer.setUseDistanceWeight (false);

      clusterer.setInputCloud (model_keypoints);
      clusterer.setInputRf (model_rf);
      clusterer.setSceneCloud (scene_keypoints);
      clusterer.setSceneRf (scene_rf);
      clusterer.setModelSceneCorrespondences (model_scene_corrs);

      //creazione matrice di rotazione traslazione dell'modello messo in corrispondenza
      clusterer.recognize (rototranslations, clustered_corrs);
    }
    else{ // Using GeometricConsistency (NON FUNZIONA)
      /*
      pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
      gc_clusterer.setGCSize (cg_size_);
      gc_clusterer.setGCThreshold (cg_thresh_);

      gc_clusterer.setInputCloud (model_keypoints);
      gc_clusterer.setSceneCloud (scene_keypoints);
      gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

      //gc_clusterer.cluster (clustered_corrs);
      gc_clusterer.recognize (rototranslations, clustered_corrs);
      */
    }//fine else



    //---------------------------- VISUALIZZAZIONE MATRICE DI ROTAZIONE TRASLAZIONE DEL MODELLO MESSO IN CORRISPONDENZA
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i){
      std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
      std::cout << "Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

      // Stampa della matrice di rotazione e del vettore di traslazione
      Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
      Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

      printf ("\n");
      printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
      printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
      printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
      printf ("\n");
      printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }//fine for




    //------------------------------VISUALIZZAZIONE DELLA CORRISPONDENZA (CLASSIFICAZIONE VERA E PROPRIA)
    pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
    viewer.addPointCloud (scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

    if (show_correspondences_ || show_keypoints_){
      //  Il modello viene traslato per fare in modo che non venga visualizzato all'interno della scena
      pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

      pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
      viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
    }

    if (show_keypoints_){ // Visualizzazione dei punti chiave sulla scena
      pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
      viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

      pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
      viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
    }

    //visualizzazione modello
    for (size_t i = 0; i < rototranslations.size (); ++i){
      pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

      std::stringstream ss_cloud;
      ss_cloud << "instance" << i;

      pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
      viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

      if (show_correspondences_){ // Visualizzazione delle corrispondenze tra modello e scena
        for (size_t j = 0; j < clustered_corrs[i].size (); ++j){
          std::stringstream ss_line;
          ss_line << "correspondence_line" << i << "_" << j;
          PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
          PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

          //  Disegna una line per ogni coppia di punti corrispondente tra il modello e la scena
          viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
        }//fine for interno
      }//fine if
    }//fine for esterno


    while (!viewer.wasStopped ()){
      viewer.spinOnce ();
    }

}//fine cloud_cb

int
main (int argc, char ** argv)
{
  //------------------------------ MOSTRO MENU'
  // Controllo dei parametri passati a linea di comando
  parseCommandLine (argc, argv);
  // Initialize ROS
  ros::init (argc, argv, "camera_point_cloud");
  ros::NodeHandle nh;
  MODELLO = models[0];
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("/points2", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);



  ros::spin();

  /*  Fork: riesco a visualizzare due modelli simultaneamente

      switch(fork()){
        case 0://figlio
        // Initialize ROS
        ros::init (argc, argv, "camera_point_cloud_");
        ros::NodeHandle nh;
        MODELLO = models[0];
        // Create a ROS subscriber for the input point cloud
        ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

        ros::spin();
        exit(0);
      }//padre
    ros::init (argc, argv, "camera_point_cloud_1");
    ros::NodeHandle nh1;
    MODELLO = models[1];
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh1.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    ros::spin();
  */

}//main
