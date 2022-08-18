//
// Created by stew on 16/05/21.
//

#ifndef H264_BAG_PLAYBACK_BAGSTATICTRANSFORMPUBLISHER_HPP
#define H264_BAG_PLAYBACK_BAGSTATICTRANSFORMPUBLISHER_HPP

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>


class BagStaticTransformBroadcaster : public tf2_ros::StaticTransformBroadcaster {


public:

/**
 * @brief h264_bag_playback::StaticTfPublisher extract and publish /tf_static and store in member tf buffer transformer_
 * only reads in the first 10 /tf_static messages and discard all msgs after 10, to prevent a node spaming /tf_static msgs.
 * @param bag bag that contains static tf
 * @param do_publish if want to publish /tf_static defaut to true.
 */
  void StaticTfPublisher(rosbag::Bag &bag, bool do_publish, std::shared_ptr<tf2_ros::Buffer> &transformer_);

  void ApplyCorrection(std::string correction_string, std::shared_ptr<tf2_ros::Buffer> &transformer_);

  std::list<tf2_msgs::TFMessageConstPtr> static_transforms;

};

#endif //H264_BAG_PLAYBACK_BAGSTATICTRANSFORMPUBLISHER_HPP
