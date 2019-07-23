#ifndef SAMPLING_H
#define SAMPLING_H

#include "sq_utils.h"

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;

/**
 * @brief Sample superquadrics based on provided parameters
 */
class SuperquadricSampling
{
public:
  /**
   * @brief Constructor
   * @param sq_params ros msg for superquadrics
   */
  SuperquadricSampling(const sq_params& sq_params);

  /**
   * @brief Sampling by superquadric equation
   */
  void sample();

  /**
   * @brief Sampling by Pilu Fisher method
   */
  void sample_pilu_fisher();

  /**
   * @brief obtain cloud
   * @param cloud
   */
  void getCloud(pcl::PointCloud<PointT>::Ptr& cloud);


private:
  pcl::PointCloud<PointT>::Ptr cloud_;
  sq_params params_;
  float r_, g_, b_;

  /**
   * @brief transform Cloud by SQ pose transformation
   * @param input_cloud
   * @param output_cloud
   */
  void transformCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud, pcl::PointCloud<PointT>::Ptr& output_cloud);
};

#endif // SAMPLING_H
