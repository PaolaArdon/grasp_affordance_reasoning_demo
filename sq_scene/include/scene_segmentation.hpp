#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <stdlib.h>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

// Type Definitions
typedef pcl::PointXYZRGBA PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

struct Plane {
   pcl::PointCloud<PointT>::Ptr obj_cloud;
   pcl::PointCloud<PointT>::Ptr contour;
   //Eigen::Vector3f centroid;
};

struct Object {
   pcl::PointCloud<PointT> obj_cloud;
   int label;
};

class SceneSegmentation {
public:
  SceneSegmentation();
  ~SceneSegmentation() {};

  void extractPlanes();
  void clusterObjects();
  void filterOjects();
  vtkSmartPointer<vtkPolyData> representAdjacencyGraph(std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> &, pcl::PointCloud<pcl::PointNormal>::Ptr &, SuperVoxelAdjacencyList &, pcl::PointCloud<pcl::PointXYZL>::Ptr &, pcl::PointCloud<pcl::PointXYZL>::Ptr &);

  pcl::PointCloud<PointT>::ConstPtr input_cloud_ptr;
  std::vector<Plane> detected_planes;
  std::vector<Object> detected_objects;

  pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
  SuperVoxelAdjacencyList sv_adjacency_list;
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
  vtkSmartPointer<vtkPolyData> polyData;

  // Supervoxel stuff
  float voxel_resolution;
  float seed_resolution;
  float color_importance;
  float spatial_importance;
  float normal_importance;
  bool use_single_cam_transform;
  bool use_supervoxel_refinement;

  // LCCPSegmentation stuff
  float concavity_tolerance_threshold;
  float smoothness_threshold;
  uint32_t min_segment_size;
  bool use_extended_convexity;
  bool use_sanity_criterion;
  unsigned int k_factor;

  // object filtering stuff
  int th_min_points_in_obj;
  int th_max_points_in_obj;

private:

};

#endif // SEGMENTATION_H
