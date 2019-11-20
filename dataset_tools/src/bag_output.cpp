#include "bag_output.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>





BagOutput::BagOutput() {

  bag = std::make_shared<rosbag::Bag>();

}

BagOutput::~BagOutput() {
  bag->close();
}


void BagOutput::publish_odom(nav_msgs::Odometry &msg, std::string topic_name) {
  if (bag->isOpen())
    bag->write(topic_name, msg.header.stamp, msg);
}


void BagOutput::publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name) {
  if (bag->isOpen())
    bag->write(topic_name, msg.header.stamp, msg);
}



void BagOutput::Initialise(std::string bag_file) {
  bag->open(bag_file, rosbag::bagmode::Write);
}
