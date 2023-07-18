#ifndef H264_BAG_PLAYBACK_HEADER
#define H264_BAG_PLAYBACK_HEADER

#include <iostream>
#include <algorithm>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <math.h> //fabs

#include <rosbag/view.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>

#include <dataset_msgs/DatasetEvent.h>
#include "rosgraph_msgs/Clock.h"

#include <boost/algorithm/string.hpp>

#include "corrected_imu_playback.hpp"
#include "bag_static_transform_publisher.hpp"
#include "bag_container.hpp"
#include "bag_writer.hpp"
#include "video.hpp"
namespace dataset_toolkit {
  class h264_bag_playback : public nodelet::Nodelet {
  public:

    h264_bag_playback();

    void init_playback(std::string input_bag_file_name = "");

    void bypass_init() {
      this->onInit();
    }

    ros::Timer timer;

    void timerCallback(const ros::TimerEvent &event);

    // convenience function to open, then block while reading the bag one message at a time
    void ReadFromBag();

    // jump forwards or backwards to a specific time
    void SeekTime(ros::Time seek_time);

    std::shared_ptr <tf2_ros::Buffer> transformer_;

    ros::NodeHandle private_nh;
    ros::NodeHandle public_nh;

    void OpenBags();

    void CloseBags();

    bool ReadNextPacket();

    void ResyncPlaybackTime();

    std::shared_ptr <CorrectedImuPlayback> imu_view;
    std::shared_ptr <rosbag::View> tf_view;

    rosbag::View::iterator tf_iter;
    ros::Time last_tf_time;

    // used to determine when the message time from latest read message
    ros::Time last_packet_time;

    // the requested bag start/end time (based on the params for percentage or specific start/end times)
    ros::Time requested_start_time;
    ros::Time requested_end_time;

    std::list <std::shared_ptr<BagContainer>> bags;

    // A video object for each video file being read
    std::map <std::string, Video> videos;

    // determine whether the playback is real-time (with optional scaling) or as fast as possible
    bool limit_playback_speed;
    double scale_playback_speed = 1.0;

    // times used to synchronise the playback
    ros::Time time_sync_real;
    ros::Time time_sync_playback;
    ros::Time time_sync_latest;

    std::map <std::string, std::list<dataset_msgs::DatasetEvent::ConstPtr>> dataset_events;

    BagWriter bag_writer;

    bool play_all_cameras;

  protected:

    virtual void onInit();

    void AdvertiseTopics(std::shared_ptr <rosbag::View> view);


    // A publisher for each topic in the bag
    std::map <std::string, ros::Publisher> publishers;

    // These are hard coded for the time being to fit the ACFR campus dataset
    std::map <std::string, std::string> frame_id_dict = {{"A0", "port_a_cam_0"},
                                                         {"A1", "port_a_cam_1"},
                                                         {"B0", "port_b_cam_0"},
                                                         {"B1", "port_b_cam_1"},
                                                         {"C0", "port_c_cam_0"},
                                                         {"C1", "port_c_cam_1"},
                                                         {"D0", "port_d_cam_0"},
                                                         {"D1", "port_c_cam_1"}};


    virtual void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);

    virtual void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message);

    virtual void CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                                     const sensor_msgs::CameraInfoConstPtr &scaled_info_msg);

    image_transport::ImageTransport image_transport;

    std::string bag_file_name;
    int scaled_width;
    int scaled_height;

    ros::Duration time_offset_;

    // the start and end times from the main bag (before user params are applied)
    ros::Time bag_start_time, bag_end_time;

    // parameters for selecting part of the dataset
    ros::Time playback_start, playback_end;
    ros::Duration playback_duration;

    ros::Publisher clock_publisher;
    rosgraph_msgs::Clock clock_msg;

    bool camera_time_bias_flag = false;
    ros::Duration camera_time_bias;

    // determine whether to calculate and publish the horizon transform
    bool horizonInBuffer;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // store the total number of messages
    uint32_t total_message_count;

  public:
    BagStaticTransformBroadcaster tf_static;

    std::string additional_namespace;
  };
}

#endif
