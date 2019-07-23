#include "scene_segmentation.hpp"

SceneSegmentation::SceneSegmentation() :
  voxel_resolution(0.0075f),
  seed_resolution(0.03f),
  color_importance(0.0f),
  spatial_importance(1.0),
  normal_importance(4.0f),
  use_single_cam_transform(false),
  use_supervoxel_refinement(false),
  concavity_tolerance_threshold(10),
  smoothness_threshold(0.1),
  min_segment_size(0),
  use_extended_convexity(false),
  use_sanity_criterion(false),
  k_factor(0),
  th_min_points_in_obj(100),
  th_max_points_in_obj(20000)
  {
    input_cloud_ptr = pcl::PointCloud<PointT>::ConstPtr(new pcl::PointCloud<PointT>);
    sv_centroid_normal_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    sv_labeled_cloud = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
    lccp_labeled_cloud = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
  };

void SceneSegmentation::extractPlanes() {
  std::cout << "Extracting planes..." << std::endl;

  // estimate normals
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne.setMaxDepthChangeFactor (0.03f);
  ne.setNormalSmoothingSize (20.0f);
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setInputCloud (input_cloud_ptr);
  ne.compute (*normal_cloud);

  // segment planes
  double mps_start = pcl::getTime ();
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;

  mps.setMinInliers (50000);
  mps.setAngularThreshold (0.017453 * 2.0); //3 degrees
  mps.setDistanceThreshold (0.02); //2cm
  mps.setInputNormals (normal_cloud);
  mps.setInputCloud (input_cloud_ptr);
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
  double mps_end = pcl::getTime ();

  // get data from the planes
  std::cout << "  - planes detected: " << regions.size () << std::endl;
  std::vector<int> idx;
  detected_planes.resize(regions.size());
  for (size_t i = 0; i < inlier_indices.size(); ++i) {
    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
    *plane_indices = inlier_indices[i];
    idx.insert(idx.end(), plane_indices->indices.begin(), plane_indices->indices.end());

    pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
    contour->points = regions[i].getContour();
    detected_planes[i].contour = contour;
    //detected_planes[i].centroid = regions[i].getCentroid();

    std::cout << std::endl;
    std::cout << "Plane " << i << " has " << plane_indices->indices.size() << " points" << std::endl;
  }

  // remove planes from main cloud
  pcl::PointIndices::Ptr overall_indices(new pcl::PointIndices);
  overall_indices->indices = idx;
  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input_cloud_ptr);
  extract.setIndices(overall_indices);
  extract.setNegative(true);
  extract.filter(*tmp_cloud);
  input_cloud_ptr = tmp_cloud;
}

void SceneSegmentation::clusterObjects() {
  std::cout << "Clustering environment..." << std::endl;

  if(input_cloud_ptr->points.size() == 0) {
    pcl::console::print_warn("  - no environment to cluster!");
    return;
  }

  // extract supervoxels
  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setUseSingleCameraTransform (use_single_cam_transform);
  super.setInputCloud (input_cloud_ptr);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  super.extract (supervoxel_clusters);

  if (use_supervoxel_refinement) {
    super.refineSupervoxels (2, supervoxel_clusters);
  }

  // get supervoxel adjacency
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);

  /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
  //pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);
  sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);

  // perform LCCPSegmentation
  pcl::LCCPSegmentation<PointT> lccp;
  lccp.setConcavityToleranceThreshold (concavity_tolerance_threshold);
  lccp.setSanityCheck (use_sanity_criterion);
  lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
  lccp.setKFactor (k_factor);
  lccp.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
  lccp.setMinSegmentSize (min_segment_size);
  lccp.segment ();

  // interpolation voxel cloud -> input cloud and relabeling
  sv_labeled_cloud = super.getLabeledCloud ();
  lccp_labeled_cloud = sv_labeled_cloud->makeShared ();
  lccp.relabelCloud (*lccp_labeled_cloud);
  lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization
}

vtkSmartPointer<vtkPolyData> SceneSegmentation::representAdjacencyGraph(std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> &supervoxel_clusters, pcl::PointCloud<pcl::PointNormal>::Ptr &sv_centroid_normal_cloud, SuperVoxelAdjacencyList &sv_adjacency_list, pcl::PointCloud<pcl::PointXYZL>::Ptr &sv_labeled_cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud) {
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  /// Calculate visualization of adjacency graph
  // Using lines this would be VERY slow right now, because one actor is created for every line (may be fixed in future versions of PCL)
  // Currently this is a work-around creating a polygon mesh consisting of two triangles for each edge
  using namespace pcl;

  typedef LCCPSegmentation<PointT>::VertexIterator VertexIterator;
  typedef LCCPSegmentation<PointT>::AdjacencyIterator AdjacencyIterator;
  typedef LCCPSegmentation<PointT>::EdgeID EdgeID;

  //std::set<EdgeID> edge_drawn;

  const unsigned char convex_color [3] = {255, 255, 255};
  const unsigned char concave_color [3] = {255, 0, 0};
  const unsigned char* color;

  //The vertices in the supervoxel adjacency list are the supervoxel centroids
  //This iterates through them, finding the edges
  std::pair<VertexIterator, VertexIterator> vertex_iterator_range;
  vertex_iterator_range = boost::vertices (sv_adjacency_list);

  /// Create a cloud of the voxelcenters and map: VertexID in adjacency graph -> Point index in cloud
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  colors->SetNumberOfComponents (3);
  colors->SetName ("Colors");

  // Create a polydata to store everything in
  for (VertexIterator itr = vertex_iterator_range.first; itr != vertex_iterator_range.second; ++itr)
  {
    const uint32_t sv_label = sv_adjacency_list[*itr];
    std::pair<AdjacencyIterator, AdjacencyIterator> neighbors = boost::adjacent_vertices (*itr, sv_adjacency_list);

    for (AdjacencyIterator itr_neighbor = neighbors.first; itr_neighbor != neighbors.second; ++itr_neighbor)
    {
      EdgeID connecting_edge = boost::edge (*itr, *itr_neighbor, sv_adjacency_list).first;  //Get the edge connecting these supervoxels
      if (sv_adjacency_list[connecting_edge].is_convex)
        color = convex_color;
      else
        color = concave_color;

      // two times since we add also two points per edge
      #if (VTK_MAJOR_VERSION < 7) || (VTK_MAJOR_VERSION == 7 && VTK_MINOR_VERSION == 0)
        colors->InsertNextTupleValue (color);
        colors->InsertNextTupleValue (color);
      #else
        colors->InsertNextTypedTuple (color);
        colors->InsertNextTypedTuple (color);
      #endif

      pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (sv_label);
      pcl::PointXYZRGBA vert_curr = supervoxel->centroid_;


      const uint32_t sv_neighbor_label = sv_adjacency_list[*itr_neighbor];
      pcl::Supervoxel<PointT>::Ptr supervoxel_neigh = supervoxel_clusters.at (sv_neighbor_label);
      pcl::PointXYZRGBA vert_neigh = supervoxel_neigh->centroid_;

      points->InsertNextPoint (vert_curr.data);
      points->InsertNextPoint (vert_neigh.data);

      // Add the points to the dataset
      vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
      polyLine->GetPointIds ()->SetNumberOfIds (2);
      polyLine->GetPointIds ()->SetId (0, points->GetNumberOfPoints ()-2);
      polyLine->GetPointIds ()->SetId (1, points->GetNumberOfPoints ()-1);
      cells->InsertNextCell (polyLine);
    }
  }
  // store adjacency graph
  polyData->SetPoints (points);
  polyData->SetLines (cells);
  polyData->GetPointData ()->SetScalars (colors);

  return polyData;
}

void SceneSegmentation::filterOjects() {
  std::cout << "Detecting objects..." << std::endl;
  detected_objects.resize(0);

  // go through all resulting point cloud to retrieve subclouds (i.e. objects)
  for(int i=0;i < lccp_labeled_cloud->points.size(); ++i) {
    uint32_t idx = lccp_labeled_cloud->points.at(i).label;
    if(idx >= detected_objects.size())
      detected_objects.resize(idx+1);
    PointT tmp_point_rgb;
    tmp_point_rgb = input_cloud_ptr->points.at(i);
    detected_objects[idx].obj_cloud.points.push_back(tmp_point_rgb);
    detected_objects[idx].label = (int)idx;
  }

  // delete small and huge objects
  int size = detected_objects.size();
  int i = 0;
  while(i < size) {
    if(detected_objects[i].obj_cloud.size() < th_min_points_in_obj || detected_objects[i].obj_cloud.size() > th_max_points_in_obj) {
      detected_objects.erase(detected_objects.begin()+i);
      size = detected_objects.size();
    }
    else
      i++;
  }

  // organise clouds (pcl stuff)
  for (int i = 0; i < detected_objects.size(); i++) {
    detected_objects[i].obj_cloud.width = (unsigned int)detected_objects[i].obj_cloud.size();
    detected_objects[i].obj_cloud.height = 1;
    detected_objects[i].obj_cloud.is_dense = true;
    std::cout << "  - object " << i << " has " << detected_objects[i].obj_cloud.size() << std::endl;
  }

  std::cout << "  - objects detected: " << detected_objects.size() << std::endl;
}
