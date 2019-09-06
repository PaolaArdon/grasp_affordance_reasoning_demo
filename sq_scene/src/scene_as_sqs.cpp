#include "scene_as_sqs.hpp"

SceneAsSQs::SceneAsSQs() :
  is_computing(false),
  new_data_to_plot(false)
  {
    continuous_input_cloud_ptr = pcl::PointCloud<PointT>::ConstPtr(new pcl::PointCloud<PointT>);
    input_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  };

void SceneAsSQs::camera_cb(const pcl::PointCloud<PointT>::ConstPtr &cloud) {
  continuous_input_cloud_ptr = cloud;
  compute();
}

void SceneAsSQs::approximateObjects() {
  std::cout << "Approximating objects..." << std::endl;

  detected_sq.resize(scene_segmentation.detected_objects.size());
  for (int i = 0; i < scene_segmentation.detected_objects.size(); i++) {
    // get point cloud of the object i
    pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT> ());
    *cloud_in = scene_segmentation.detected_objects[i].obj_cloud;

    std::cout << std::endl;
    std::cout << "Object " << i << " has " << cloud_in->points.size() << " points" << std::endl;

    // store obect cloud
    char buffer_ob [50];
    sprintf (buffer_ob, "obj_%i.pcd", i);
    pcl::io::savePCDFile(buffer_ob, *cloud_in, true);

    // object filtering
    double time_filtering_1_start = pcl::getTime ();
    pcl::PointCloud<PointT>::Ptr filtered_cloud_pcl(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in);
    double time_filtering_1_stop = pcl::getTime ();

    // input cloud downsampling
    pcl::VoxelGrid<PointT> ds;
    ds.setInputCloud(cloud_in);
    ds.setLeafSize(0.005f, 0.005f, 0.005f);
    ds.filter(*cloud_in);

    // superquadric fitting
    double time_fitting_start = pcl::getTime ();
    SuperquadricFitting* sq_fit = new SuperquadricFitting(cloud_in);
    sq_fit->set_pose_est_method("pca");
    //sq_fit->set_pose_est_method("iteration");
    sq_params min_param;
    double min_fit;
    sq_fit->fit();
    sq_fit->getMinParams(min_param);
    sq_fit->getMinError(min_fit);
    double time_fitting_stop = pcl::getTime ();

    // superquadric sampling
    double time_sampling_start = pcl::getTime ();
    SuperquadricSampling *sam = new SuperquadricSampling(min_param);
    sam->sample_pilu_fisher();
    pcl::PointCloud<PointT>::Ptr s_cloud(new pcl::PointCloud<PointT>);
    sam->getCloud(s_cloud);
    detected_sq[i].obj_cloud = s_cloud;
    double time_sampling_stop = pcl::getTime ();

    // downsampling sq
    double time_filtering_2_start = pcl::getTime ();
    pcl::VoxelGrid<PointT> dssq;
    dssq.setInputCloud(s_cloud);
    dssq.setLeafSize(0.005f, 0.005f, 0.005f);
    dssq.filter(*s_cloud);
    double time_filtering_2_stop = pcl::getTime ();

    // save resulting superquadric
    char buffer_sq [50];
    sprintf (buffer_sq, "obj_%i_sampled.pcd", i);
    pcl::io::savePCDFile(buffer_sq, *s_cloud, true);

    // some print outs
    std::cout << "Minimum error["<<i<<"] is : "<<min_fit<<std::endl;
    std::cout << "Minimum parameters["<<i<<"] is: "<<"a1:"<<min_param.a1<<"  a2:"
    << min_param.a2<<" a3:"<<min_param.a3<<" e1:"<<min_param.e1<<" e2:"<<min_param.e2<<" position:"
    << min_param.pose.position.x<<" "<<min_param.pose.position.y<<min_param.pose.position.z<<" orientation:"
    << min_param.pose.orientation.x<<" "<<min_param.pose.orientation.y<<" "<<min_param.pose.orientation.z<<" "
    << min_param.pose.orientation.w<<std::endl;

    std::cout << "  filtering 1 time: " << time_filtering_1_stop - time_filtering_1_start << std::endl;
    std::cout << "  fitting time: " << time_fitting_stop - time_fitting_start << std::endl;
    std::cout << "  sampling time: " << time_sampling_stop - time_sampling_start << std::endl;
    std::cout << "  filtering 2 time: " << time_filtering_2_stop - time_filtering_2_start << std::endl;
    std::cout << std::endl;
  }
}

//void SceneAsSQs::mirrorObject(pcl::visualization::PCLVisualizer::Ptr &viewer, pcl::PointCloud<PointT>::Ptr &cloud_in, int &i) {
  // mirror
  // Eigen::Affine3f transformation_centroid = Eigen::Affine3f::Identity();
  // transformation_centroid.translation() << -middle_x, -middle_y, -middle_z;
  // pcl::transformPointCloud(*cloud_in, *cloud_in, transformation_centroid);
  // // mirror
  // Eigen::Affine3f transformation_centroid_2 = Eigen::Affine3f::Identity();
  // transformation_centroid_2(0, 0) = -1;
  // transformation_centroid_2(1, 1) = -1;
  // transformation_centroid_2(2, 2) = -1;
  // pcl::transformPointCloud(*cloud_in, *cloud_in, transformation_centroid_2);
  // // put back
  // transformation_centroid.translation() << middle_x, middle_y, middle_z;
  // pcl::transformPointCloud(*cloud_in, *cloud_in, transformation_centroid);
  //
  // char buffer_ch[50];
  // sprintf (buffer_ch, "ch %i", i);
  // viewer->addPointCloud<PointT> (cloud_in, buffer_ch);
  // std::cout << "ch " << buffer_ch << " has " << cloud_in->size() << std::endl;
//}


void SceneAsSQs::plotObjectInformation(pcl::visualization::PCLVisualizer::Ptr &viewer, pcl::PointCloud<PointT>::Ptr &cloud_in, int &i) {
  // initialise variables
  float major_value, middle_value, minor_value;
  Eigen::Matrix3f rotational_matrix_OBB;
  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
  PointT min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB;

  // get object descriptors (also performs pca)
  pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
  feature_extractor.setInputCloud (cloud_in);
  feature_extractor.compute ();
  //feature_extractor.getMomentOfInertia (moment_of_inertia);
  //feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);

  // compute and plot mathematical centre
  /*feature_extractor.getMassCenter (mass_center);
  {
    double scale = 0.1;
    pcl::PointXYZ center(mass_center (0), mass_center (1), mass_center (2));
    pcl::PointXYZ x_axis(scale * major_vector (0) + mass_center (0), scale * major_vector (1) + mass_center (1), scale * major_vector (2) + mass_center (2));
    pcl::PointXYZ y_axis(scale * middle_vector (0) + mass_center (0), scale * middle_vector (1) + mass_center (1), scale * middle_vector (2) + mass_center (2));
    pcl::PointXYZ z_axis(scale * minor_vector (0) + mass_center (0), scale * minor_vector (1) + mass_center (1), scale * minor_vector (2) + mass_center (2));
    char buffer_sq_major[50]; sprintf (buffer_sq_major, "major eigen vector math %i", i);
    char buffer_sq_middle[50]; sprintf (buffer_sq_middle, "middle eigen vector math %i", i);
    char buffer_sq_minor[50]; sprintf (buffer_sq_minor, "minor eigen vector math %i", i);
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, buffer_sq_major);
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, buffer_sq_middle);
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, buffer_sq_minor);
  }*/

  // compute and plot geometrical centre
  double middle_x = (min_point_AABB.x + max_point_AABB.x) / 2;
  double middle_y = (min_point_AABB.y + max_point_AABB.y) / 2;
  double middle_z = (min_point_AABB.z + max_point_AABB.z) / 2;
  {
    double scale = 0.1;
    pcl::PointXYZ center(middle_x, middle_y, middle_z);
    pcl::PointXYZ x_axis(scale * major_vector (0) + middle_x, scale * major_vector (1) + middle_y, scale * major_vector (2) + middle_z);
    pcl::PointXYZ y_axis(scale * middle_vector (0) + middle_x, scale * middle_vector (1) + middle_y, scale * middle_vector (2) + middle_z);
    pcl::PointXYZ z_axis(scale * minor_vector (0) + middle_x, scale * minor_vector (1) + middle_y, scale * minor_vector (2) + middle_z);
    char buffer_sq_major[50]; sprintf (buffer_sq_major, "major eigen vector geom %i", i);
    char buffer_sq_middle[50]; sprintf (buffer_sq_middle, "middle eigen vector geom %i", i);
    char buffer_sq_minor[50]; sprintf (buffer_sq_minor, "minor eigen vector geom %i", i);
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, buffer_sq_major);
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, buffer_sq_middle);
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, buffer_sq_minor);
  }

  // plot AABB
  char buffer_aabb[50];
  sprintf (buffer_aabb, "aabb %i", i);
  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, buffer_aabb);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, buffer_aabb);

  // plot OBB
  /*Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);
  char buffer_obb[50];
  sprintf (buffer_obb, "obb %i", i);
  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, buffer_obb);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, buffer_obb);
  */
  //
  // pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
  // pcl::ConvexHull<PointT> chull;
  // chull.setInputCloud(cloud_in);
  // chull.reconstruct (*cloud_hull);
  //
  // char buffer_ch[50];
  // sprintf (buffer_ch, "ch %i", i);
  // viewer->addPointCloud<PointT> (cloud_hull, buffer_ch);
  // std::cout << "ch " << buffer_ch << " has " << cloud_hull->size() << std::endl;
}

void SceneAsSQs::compute() {
  // avoid starting a new computation until the previous one is finished
  if (is_computing) {
    return;
  }
  is_computing = true;

  // make a copy of the current frame in the camera port
  *input_cloud_ptr = *continuous_input_cloud_ptr;
  scene_segmentation.input_cloud_ptr = continuous_input_cloud_ptr;

  // remove planes, cluster resulting non-planar cloud, and filter/retrieve clustered objects
  scene_segmentation.extractPlanes();
  scene_segmentation.clusterObjects();
  scene_segmentation.filterOjects();

  // approximate resulting clustered objects as SQ
  if (compute_sq) {
    approximateObjects();
  }

  is_computing = false;
  new_data_to_plot = true;
}

void SceneAsSQs::visualisationLoop() {
  // configure visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->registerKeyboardCallback(keyboardEventOccurred, 0);
  viewer->addPointCloud(scene_segmentation.lccp_labeled_cloud, "maincloud");
  viewer->addCoordinateSystem(1.0);

  // visualization loop
  while (!viewer->wasStopped ()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));

    //avoid reploting SAME data, slows down visualiser
    if (!new_data_to_plot) {
      continue;
    }
    std::cout << "new_data_to_plot available" << std::endl;

    // show original cloud
    //pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(input_cloud_ptr);
    //viewer->updatePointCloud<PointT> (input_cloud_ptr, rgb, "maincloud");
    // show segmented
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(scene_segmentation.input_cloud_ptr);
    viewer->updatePointCloud<PointT> (scene_segmentation.input_cloud_ptr, rgb, "maincloud");
    // show clusters
    //viewer->updatePointCloud ((show_supervoxels) ? scene_segmentation.sv_labeled_cloud : scene_segmentation.lccp_labeled_cloud, "maincloud");

    if (show_normals) {
      viewer->removePointCloud ("normals");
      viewer->addPointCloudNormals<pcl::PointNormal> (scene_segmentation.sv_centroid_normal_cloud, 1, normals_scale, "normals");
    }
    if (show_adjacency) {
      viewer->removeShape ("adjacency_graph");
      viewer->addModelFromPolyData (scene_segmentation.polyData, "adjacency_graph");
    }
    else {
      viewer->removeShape ("adjacency_graph");
    }
    if (show_help) {
      viewer->removeShape ("help_text");
      printText (viewer);
    }
    else {
      removeText (viewer);
      if (!viewer->updateText ("Press d to show help", 5, 10, 12, 1.0, 1.0, 1.0, "help_text"))
        viewer->addText ("Press d to show help", 5, 10, 12, 1.0, 1.0, 1.0, "help_text");
    }

    // show planes
    for (int pl = 0; pl < scene_segmentation.detected_planes.size(); pl++) {
      char buffer_pl [50];
      sprintf (buffer_pl, "plane_%i", pl);
      printf ("plotting pl %s\n", buffer_pl);
      pcl::visualization::PointCloudColorHandlerCustom <PointT> color(scene_segmentation.detected_planes[pl].contour, red[pl%6], grn[pl%6], blu[pl%6]);
      if(!viewer->updatePointCloud(scene_segmentation.detected_planes[pl].contour, color, buffer_pl))
        viewer->addPointCloud(scene_segmentation.detected_planes[pl].contour, color, buffer_pl);
    }

    for (int pl = 0; pl < scene_segmentation.detected_objects.size(); pl++) {
      char buffer_pl [50];
      sprintf (buffer_pl, "obj_%i", pl);
      printf ("plotting obj %s\n", buffer_pl);
      pcl::PointCloud<PointT>::Ptr cloud_object_i(new pcl::PointCloud<PointT>);
      *cloud_object_i = scene_segmentation.detected_objects[pl].obj_cloud;
      if(!viewer->updatePointCloud(cloud_object_i, buffer_pl))
        viewer->addPointCloud(cloud_object_i, buffer_pl);
    }

    // show objects understanding
    for (int ob = 0; ob < scene_segmentation.detected_objects.size(); ob++) {
      pcl::PointCloud<PointT>::Ptr cloud_object_i(new pcl::PointCloud<PointT>);
      *cloud_object_i = scene_segmentation.detected_objects[ob].obj_cloud;
      //plotObjectInformation(viewer, cloud_object_i, ob);
      //mirrorObject(viewer, cloud_object_i, ob);
    }

    // show superquadrics
    /*if (compute_sq) {
      for (int sq = 0; sq < detected_sq.size(); sq++) {
        char buffer_sq [50];
        sprintf (buffer_sq, "sq_%i", sq);
        if (!viewer->updatePointCloud<PointT> (detected_sq[sq].obj_cloud, buffer_sq)) {
           viewer->addPointCloud<PointT> (detected_sq[sq].obj_cloud, buffer_sq);
        }
      }
    }*/

    new_data_to_plot = false;
  }
}


int main (int argc, char ** argv) {
  SceneAsSQs scene_as_sqs;
  scene_as_sqs.compute_sq = true;
  scene_as_sqs.scene_segmentation.th_min_points_in_obj = 100;
  scene_as_sqs.scene_segmentation.th_max_points_in_obj = 250000;

  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind(&SceneAsSQs::camera_cb, &scene_as_sqs, _1);

  bool offline = (pcl::console::find_switch (argc, argv, "-offline"));
  if (offline) {
    pcl::PointCloud<PointT>::Ptr cloud_xyz(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("/home/pardon/catkin_ws/ei_cropped.pcd", *cloud_xyz);
    scene_as_sqs.continuous_input_cloud_ptr = cloud_xyz;
    PCL_INFO ("PCD loaded! \n");
  }
  else {
    // open camera and connect it to the scene segmentation class
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::signals2::connection c = interface->registerCallback(f);
    interface->start();
  }

  /*
  // Limit to things we think are roughly at the table height ------------------------------------
  pcl::PointCloud<PointT>::Ptr cloud_filteredZ(new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud_xyz);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.2, 1.5);
  //pass.setFilterLimits(table_height - 0.01, table_height + block_size + 0.02); // DTC
  pass.filter(*cloud_filteredZ);

  // Limit to things in front of the robot ---------------------------------------------------
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pass.setInputCloud(cloud_filteredZ);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(.1,4);
  pass.filter(*cloud_filtered);

  cloud_filtered->width = 1;
  cloud_filtered->height = cloud_filtered->points.size();
  cloud_filtered->resize(cloud_filtered->width*cloud_filtered->height);
  cloud_filtered->is_dense = false;*/



  //scene_as_sqs.continuous_input_cloud_ptr = cloud_filtered;
  //scene_as_sqs.continuous_input_cloud_ptr->width = 1;
  //scene_as_sqs.continuous_input_cloud_ptr->height = scene_as_sqs.continuous_input_cloud_ptr->points.size();







  // parse terminal inputs
  // supervoxel stuff
  scene_as_sqs.scene_segmentation.use_single_cam_transform = pcl::console::find_switch (argc, argv, "-tvoxel");
  scene_as_sqs.scene_segmentation.use_supervoxel_refinement = pcl::console::find_switch (argc, argv, "-refine");

  pcl::console::parse (argc, argv, "-v", scene_as_sqs.scene_segmentation.voxel_resolution);
  pcl::console::parse (argc, argv, "-s", scene_as_sqs.scene_segmentation.seed_resolution);
  pcl::console::parse (argc, argv, "-c", scene_as_sqs.scene_segmentation.color_importance);
  pcl::console::parse (argc, argv, "-z", scene_as_sqs.scene_segmentation.spatial_importance);
  pcl::console::parse (argc, argv, "-n", scene_as_sqs.scene_segmentation.normal_importance);

  normals_scale = scene_as_sqs.scene_segmentation.seed_resolution / 2.0;

  // segmentation stuff
  pcl::console::parse (argc, argv, "-ct", scene_as_sqs.scene_segmentation.concavity_tolerance_threshold);
  pcl::console::parse (argc, argv, "-st", scene_as_sqs.scene_segmentation.smoothness_threshold);
  scene_as_sqs.scene_segmentation.use_extended_convexity = pcl::console::find_switch (argc, argv, "-ec");
  if (scene_as_sqs.scene_segmentation.use_extended_convexity)
    scene_as_sqs.scene_segmentation.k_factor = 1;
  scene_as_sqs.scene_segmentation.use_sanity_criterion = pcl::console::find_switch (argc, argv, "-sc");
  pcl::console::parse (argc, argv, "-smooth", scene_as_sqs.scene_segmentation.min_segment_size);

  // wait until camera is ready
  if (!offline) {
    while (scene_as_sqs.continuous_input_cloud_ptr->size () == 0) {
      std::cout << "waiting..." << std::endl;
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }
  std::cout << "All ready!" << std::endl;

  // start loop
  scene_as_sqs.compute();
  scene_as_sqs.visualisationLoop();

  return (0);
}
