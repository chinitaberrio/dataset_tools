  #include <gtest/gtest.h>

  #include <iostream>

  #include "h264_bag_playback.hpp"
  #include "bag_data.h"


  class DirectPlayback : public dataset_toolkit::h264_bag_playback {

  public:

    DirectPlayback(std::string bag_file)
        : h264_bag_playback() {

      private_nh.setParam("bag_file", bag_file);
    }


    void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {}


    void CameraInfoPublisher(ros::Publisher &publisher, const sensor_msgs::CameraInfoConstPtr &message) {}


    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
      //std::cout << "msg" << std::endl;
    }
  };



  class BagReaderTest : public testing::Test {
  protected:

    virtual void SetUp() {
    }

  };


  TEST_F(BagReaderTest, test_full_bag_read) {

    std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
    DirectPlayback playback(bag_file_name);
    playback.ReadFromBag();

    EXPECT_TRUE(true);
  }

  TEST_F(BagReaderTest, test_percentage_bag_read) {

    EXPECT_TRUE(true);
  }


  int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
  }