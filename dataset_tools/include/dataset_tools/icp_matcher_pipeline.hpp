//
// Created by stew on 27/08/19.
//

#ifndef LOCALISER_ICP_MATCHER_PIPELINE_HPP
#define LOCALISER_ICP_MATCHER_PIPELINE_HPP

#include "run_pipeline.hpp"

#include "point_xyzir.h"
#include "point_xyzirc.h"

class ICPMatcherPipeline : public RunPipeline {

public:
  ICPMatcherPipeline();
  ~ICPMatcherPipeline() {}

  void receive_message(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& poles_pointcloud,
                       const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& corners_pointcloud);
private:
  PipelineInput<pcl::PointCloud<pcl::PointXYZIRC>> input_poles, input_corners;
  PipelineOutput<nav_msgs::Odometry> output_pose;

};

#endif //LOCALISER_POINT_CLOUD_FEATURES_PIPELINE_HPP
