#ifndef H264_BAG_PLAYBACK_HEADER
#define H264_BAG_PLAYBACK_HEADER

#include <iostream>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <math.h> //fabs

#include <rosbag/view.h>

#include <tf2_msgs/TFMessage.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>


#include "video.hpp"


namespace dataset_toolkit
{

class h264_bag_playback : public nodelet::Nodelet
{
public:
  h264_bag_playback();

  void bypass_init() {
    this->onInit();
  }

  ros::Timer timer;

  void timerCallback(const ros::TimerEvent& event);

  void ReadFromBag();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

protected:

  virtual void onInit();

  void BagReader(std::string file_name);

  void AdvertiseTopics(rosbag::View &view);

  // scale the camera info message for a different output size
  void ScaleCameraInfoMsg(int original_width,
                          int scaled_width,
                          int original_height,
                          int scaled_height,
                          sensor_msgs::CameraInfo::Ptr &scaled_info_msg);

  // A publisher for each topic in the bag
  std::map<std::string, ros::Publisher> publishers;

  // A video object for each video file being read
  std::map<std::string, Video> videos;

  // These are hard coded for the time being to fit the ACFR campus dataset
  std::map<std::string, std::string> frame_id_dict = {{"A0", "gmsl_centre_link"},
                                                      {"A1", "gmsl_left_link"},
                                                      {"A2", "gmsl_right_link"},
                                                      {"A3", "gmsl_back_link"},
                                                      {"B0", "gmsl_left_side_link"},
                                                      {"B1", "gmsl_right_side_link"}};


  virtual void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);
  virtual void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message);
  virtual void CameraInfoPublisher(ros::Publisher &publisher, const sensor_msgs::CameraInfoConstPtr &message);

  virtual void StaticTfPublisher(rosbag::Bag &bag, bool do_publish=true);

  ros::NodeHandle private_nh;
  ros::NodeHandle public_nh;

  image_transport::ImageTransport image_transport;

  // parameters for selecting part of the dataset
  ros::Time playback_start, playback_end;
  ros::Duration playback_duration;

  bool camera_time_bias_flag = false;
  ros::Duration camera_time_bias;

};

}


#endif