#include "sq_fitting.h"
#include "sq_sampling.h"
#include "scene_segmentation.hpp"

#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <boost/format.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//#include <pcl/surface/convex_hull.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Type Definitions
typedef pcl::PointXYZRGBA PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

// Callback and variables
bool show_normals = false, normals_changed = false;
bool show_adjacency = false;
bool show_supervoxels = false;
bool show_help = true;
float normals_scale;

struct SQ {
   pcl::PointCloud<PointT>::Ptr obj_cloud;
};

class SceneAsSQs {
public:
  SceneAsSQs();
  ~SceneAsSQs() {};

  pcl::PointCloud<PointT>::ConstPtr continuous_input_cloud_ptr;
  pcl::PointCloud<PointT>::Ptr input_cloud_ptr;
  SceneSegmentation scene_segmentation;
  std::vector<SQ> detected_sq;

  void camera_cb(const pcl::PointCloud<PointT>::ConstPtr &);
  void approximateObjects();
  void compute();
  void plotObjectInformation(pcl::visualization::PCLVisualizer::Ptr &, pcl::PointCloud<PointT>::Ptr &, int &);
  //void mirrorObject(pcl::visualization::PCLVisualizer::Ptr &, pcl::PointCloud<PointT>::Ptr &, int &);
  void visualisationLoop();

  bool compute_sq;
  bool is_computing;
  bool new_data_to_plot;


private:
  //pcl::visualization::PCLVisualizer::Ptr viewer;
  boost::mutex cloud_mutex;
};

// palette
unsigned char red [6] = {255,   0,   0, 255, 255,   0};
unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

/** \brief Callback for setting options in the visualizer via keyboard.
 *  \param[in] event_arg Registered keyboard event  */
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event_arg, void*) {
  int key = event_arg.getKeyCode ();

  if (event_arg.keyUp ())
    switch (key)
    {
      case (int) '1':
        show_normals = !show_normals;
        normals_changed = true;
        break;
      case (int) '2':
        show_adjacency = !show_adjacency;
        break;
      case (int) '3':
        show_supervoxels = !show_supervoxels;
        break;
      case (int) '4':
        normals_scale *= 1.25;
        normals_changed = true;
        break;
      case (int) '5':
        normals_scale *= 0.8;
        normals_changed = true;
        break;
      case (int) 'd':
      case (int) 'D':
        show_help = !show_help;
        break;
      default:
        break;
    }
}

/** \brief Displays info text in the specified PCLVisualizer
 *  \param[in] viewer_arg The PCLVisualizer to modify  */
void printText (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg) {
  std::string on_str = "ON";
  std::string off_str = "OFF";
  if (!viewer_arg->updateText ("Press (1-n) to show different elements (d) to disable this", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text"))
        viewer_arg->addText ("Press (1-n) to show different elements", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text");

  std::string temp = "(1) Supervoxel Normals, currently " + ( (show_normals) ? on_str : off_str);
  if (!viewer_arg->updateText (temp, 5, 60, 10, 1.0, 1.0, 1.0, "normals_text"))
        viewer_arg->addText (temp, 5, 60, 10, 1.0, 1.0, 1.0, "normals_text");

  temp = "(2) Adjacency Graph, currently " + ( (show_adjacency) ? on_str : off_str) + "\n      White: convex; Red: concave";
  if (!viewer_arg->updateText (temp, 5, 38, 10, 1.0, 1.0, 1.0, "graph_text"))
        viewer_arg->addText (temp, 5, 38, 10, 1.0, 1.0, 1.0, "graph_text");

  temp = "(3) Press to show " + ( (show_supervoxels) ? std::string ("SEGMENTATION") : std::string ("SUPERVOXELS"));
  if (!viewer_arg->updateText (temp, 5, 26, 10, 1.0, 1.0, 1.0, "supervoxel_text"))
        viewer_arg->addText (temp, 5, 26, 10, 1.0, 1.0, 1.0, "supervoxel_text");

  temp = "(4/5) Press to increase/decrease normals scale, currently " + boost::str (boost::format ("%.3f") % normals_scale);
  if (!viewer_arg->updateText (temp, 5, 14, 10, 1.0, 1.0, 1.0, "normals_scale_text"))
        viewer_arg->addText (temp, 5, 14, 10, 1.0, 1.0, 1.0, "normals_scale_text");
}

/** \brief Removes info text in the specified PCLVisualizer
 *  \param[in] viewer_arg The PCLVisualizer to modify  */
void removeText (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg) {
  viewer_arg->removeShape ("hud_text");
  viewer_arg->removeShape ("normals_text");
  viewer_arg->removeShape ("graph_text");
  viewer_arg->removeShape ("supervoxel_text");
  viewer_arg->removeShape ("normals_scale_text");
}
