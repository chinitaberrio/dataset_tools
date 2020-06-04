#include "replace_msg.h"
#include <rosbag/bag.h>
#include <rosbag/player.h>
#include <rosbag/structures.h>
#include <rosbag/message_instance.h>
namespace dataset_toolkit
{
replace_msg::replace_msg()
{

    bag.open("test.bag", rosbag::bagmode::Write);


//    bag.close();
}


void
replace_msg::CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                                 const sensor_msgs::CameraInfoConstPtr &scaled_info_msg) {
    std::string const& topic = message.getTopic();
    ros::Time const& header_time = message.getTime();
    bag.write(topic, header_time, scaled_info_msg);
}


void
replace_msg::MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
    std::string const& topic = message.getTopic();
    ros::Time const& header_time = message.getTime();
    bag.write(topic, header_time, message);
}
}

int main(int argc, char **argv) {

  //Initialize Node and handles
  ros::init(argc, argv, "replaceMsg_writeToBag");
  ros::NodeHandle n;

  dataset_toolkit::replace_msg bag_tools;
  //bag_tools.bypass_init();
  bag_tools.init_playback();
  bag_tools.ReadFromBag();

  return 0;
}
