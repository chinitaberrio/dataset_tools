#include "run_pipeline.hpp"


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "point_xyzir.h"
#include "point_xyzirc.h"


RunPipeline::RunPipeline() {

}

RunPipeline::~RunPipeline() {
}





bool
RunPipeline::WaitForMessages() {

  bool messages_received = false;
  do {

    ros::spinOnce();

    messages_received = true;
    for (auto pipe_out: pipes_out) {
      if (!pipe_out->message_received_) {
        messages_received = false;
      }
    }


  } while (!messages_received && ros::ok());

  return messages_received;
}


void
RunPipeline::ResetMessageFlags() {

  for (auto pipe_out: pipes_out) {
    pipe_out->message_received_ = false;
  }
}


/*
bool
RunPipeline::PublishLatestTransform(std::string parent_frame, std::string child_frame) {

  // find the transform from the buffer

  // publish the transform


  return true;
}
*/

