#ifndef ros_localiser_h
#define ros_localiser_h

#include <ros/ros.h>

#include "localiser_core/graph_optimiser.hpp"
//#include "localiser_core/gtsam_optimiser.hpp"
#include "localiser_core/linear_filter.hpp"

#include "bag_input.hpp"
#include "dataset_tools/bag_output.hpp"
#include "localiser_core/publisher.hpp"

#include "localiser_core/localiser_input.hpp"
#include "localiser_core/localiser_output.hpp"

#include <dataset_tools/point_cloud_features_pipeline.hpp>
#include <dataset_tools/icp_matcher_pipeline.hpp>

#include "localiser/instruct_localiser.h"

#include <dataset_tools/LocaliserStats.h>


/*!
 * \brief The class that manages the localisation process
 *
 */

class ROSLocaliser {
public:

  ROSLocaliser();

  void Initialise();

  std::shared_ptr<BagInput> bag_input;
  std::shared_ptr<BagOutput> bag_output;
  std::shared_ptr<Publisher> publisher;

  std::shared_ptr<LocaliserInput> localiser_input;
  std::shared_ptr<LocaliserOutput> localiser_output;

  std::shared_ptr<PointCloudFeaturesPipeline> features_pipeline;
  std::shared_ptr<ICPMatcherPipeline> icp_pipeline;

  std::shared_ptr<ImuMeasurement> imu;
  std::shared_ptr<SpeedMeasurement> speed;

  std::shared_ptr<ICPObservation> map_icp;
  std::shared_ptr<GNSSObservation> gnss;

  std::shared_ptr<GraphOptimiser> graph_optimiser;
//  std::shared_ptr<GtsamOptimiser> gtsam_optimiser;
  std::shared_ptr<PositionHeadingEKF> linear_filter;

  ros::ServiceServer service;


  bool InstructionCallback(localiser::instruct_localiser::Request& req,
                           localiser::instruct_localiser::Response& res);



};




#endif
