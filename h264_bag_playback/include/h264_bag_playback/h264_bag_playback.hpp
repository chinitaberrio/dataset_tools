#include <iostream>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <math.h> //fabs

#include <rosbag/view.h>

#include "video.hpp"


namespace dataset_toolkit
{

class H264BagPlayback : public nodelet::Nodelet
{
public:
  H264BagPlayback();

  void bypass_init() {
    this->onInit();
  }

private:

  virtual void onInit();

  void BagReader(std::string file_name);

  void AdvertiseTopics(rosbag::View &view);

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


  ros::NodeHandle private_nh;
  ros::NodeHandle public_nh;
  image_transport::ImageTransport image_transport;

  bool save_time_diff = true;
  ros::Duration dur;





};

PLUGINLIB_EXPORT_CLASS(dataset_toolkit::H264BagPlayback, nodelet::Nodelet);
}
