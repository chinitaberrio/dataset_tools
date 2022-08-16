#ifndef BAG_WRITER_HEADER
#define BAG_WRITER_HEADER

#include <ros/ros.h>

#include <rosbag/view.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <gmsl_frame_msg/FrameInfo.h>


namespace dataset_toolkit {

  class BagWriter {
  public:
    BagWriter() {}

    BagWriter(std::string &filename_prefix) {
      prefix = filename_prefix;
    }

    ~BagWriter() {
      this->CloseBags();
    }

    void SetPrefix(std::string &filename_prefix){
      prefix = filename_prefix;
    }

    void CloseBags() {
      for (auto bag: output_bags)
        bag.second->close();

      for (auto video: output_videos)
        video.second.release();

      output_bags.clear();
    }

    template <typename MsgType>
    bool WriteMessage(std::string &bag_name, std::string &topic_name, ros::Time &msg_time, MsgType msg, std::string additional_folder_name = "") {

      auto bag_instance = output_bags.find(bag_name);

      if (bag_instance == output_bags.end()) {

//std::cout << "msg 1" << std::endl;

        std::shared_ptr<rosbag::Bag> new_bag = std::make_shared<rosbag::Bag>();

        output_bags[bag_name] = new_bag;

        std::string modified_prefix = prefix;

        // inject the additional folder name to the prefix
        if (additional_folder_name != "") {
          std::size_t found = prefix.find_last_of('/');
          if (found!=std::string::npos) {
            modified_prefix = prefix.substr(0,found+1) + additional_folder_name + prefix.substr(found);
          }
        }

        std::string additional_bag_name;
        if (bag_name != "") {
          additional_bag_name = modified_prefix + "." + bag_name + ".bag";
        }
        else {
          additional_bag_name = modified_prefix + ".bag";
        }
//        ROS_INFO_STREAM("Saving data to additional bagfile named " << additional_bag_name);

        new_bag->open(additional_bag_name, rosbag::bagmode::Write);
      }

//std::cout << "msg 3" << std::endl;

      output_bags[bag_name]->write(topic_name, msg_time, msg);

//std::cout << "msg 4" << std::endl;

      return true;
    }

    bool WriteToVideo(std::string &bag_name, std::string &topic_name, ros::Time &msg_time, sensor_msgs::Image::Ptr image,
                      sensor_msgs::CameraInfoPtr camera_info_msg = NULL, std::string additional_folder_name = "") {

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
      }

      if (cv_ptr->image.cols == 0 || cv_ptr->image.rows == 0) {
        ROS_INFO_STREAM("Image to write is empty [" << cv_ptr->image.cols << "," << cv_ptr->image.rows << "] to topic [" << topic_name << "]");
        return false;
      }

      auto video_instance = output_videos.find(topic_name);

      if (video_instance == output_videos.end()) {

        std::vector<std::string> topic_parts;
        boost::split(topic_parts, topic_name, boost::is_any_of("/"));

        //_gmsl_A2_image_color_colored_pointcloud
        std::string modified_prefix = prefix;
        if (additional_folder_name != "") {
          // inject the additional folder name to the prefix

          std::size_t found = prefix.find_last_of('/');
          if (found!=std::string::npos) {
            modified_prefix = prefix.substr(0,found+1) + additional_folder_name + prefix.substr(found);
          }
        }

        std::string video_file_name = modified_prefix + "-" + topic_parts[2] + ".mp4";

        //std::string renamed_topic = topic_name;
        //std::replace(renamed_topic.begin(), renamed_topic.end(), '/', '_');
        ROS_INFO_STREAM("Opening video to write new data with name " << video_file_name);
        //return false;

        output_videos[topic_name] = cv::VideoWriter();
        message_count[topic_name] = 1;
        //output_videos[topic_name].open(video_file_name, cv::VideoWriter::fourcc('H','2','6','4'), 30, cv::Size(cv_ptr->image.cols,cv_ptr->image.rows));
        output_videos[topic_name].open(video_file_name, cv::VideoWriter::fourcc('m','p','4','v'), 30, cv::Size(cv_ptr->image.cols,cv_ptr->image.rows));
        //output_videos[topic_name].open(video_file_name, cv::VideoWriter::fourcc('X','V','I','D'), 30, cv::Size(cv_ptr->image.cols,cv_ptr->image.rows));

        if(!output_videos[topic_name].isOpened()) { // check if we succeeded
          ROS_INFO_STREAM("could not open video file size[" << cv_ptr->image.cols << "," << cv_ptr->image.rows << "]: " << video_file_name);
          return false;
        }
        else {
          ROS_INFO_STREAM("Opened video file size[" << cv_ptr->image.cols << "," << cv_ptr->image.rows << "]: " << video_file_name);
        }


        output_videos[topic_name].write(cv_ptr->image);
      }
      else {
        message_count[topic_name] += 1;
        output_videos[topic_name].write(cv_ptr->image);
      }

      gmsl_frame_msg::FrameInfo::Ptr frame_info = boost::make_shared<gmsl_frame_msg::FrameInfo>();
      frame_info->header.stamp = msg_time;
      frame_info->ros_timestamp = msg_time;
      frame_info->frame_counter = message_count[topic_name];
      frame_info->global_counter = message_count[topic_name];
      frame_info->camera_timestamp = 0;

//      ros::Time nvidia_timestamp = ros::Time((frame_info_msg->camera_timestamp) / 1000000.0, ((frame_info_msg->camera_timestamp) % 1000000) * 1000.0);

      std::string frame_info_topic = topic_name + "/frame_info";
      std::string camera_info_topic = topic_name + "/camera_info";

      this->WriteMessage(bag_name, frame_info_topic, msg_time, frame_info);

      if (camera_info_msg) {
        this->WriteMessage(bag_name, camera_info_topic, msg_time, camera_info_msg);
      }

      return true;
    }


//    sensor_msgs::Image::Ptr

    //write compressed image
    /*
void OpenCVConnector::PublishJpeg(uint8_t* image_compressed, uint32_t image_compressed_size) {
sensor_msgs::CompressedImage c_img_msg;

c_img_msg.data.resize( image_compressed_size );
memcpy(&c_img_msg.data[0], image_compressed, image_compressed_size);

std_msgs::Header header; // empty header
c_img_msg.header = header;
//c_img_msg.header.seq = counter; // user defined counter
c_img_msg.header.stamp = ros::Time::now(); // time

c_img_msg.format = "jpeg";

pub_comp.publish(  c_img_msg  );

 camera_info.roi.do_rectify=true;
pubCamInfo.publish(  camera_info ); */

    std::string prefix;

  private:
    std::map<std::string, std::shared_ptr<rosbag::Bag>> output_bags;

    std::map<std::string, cv::VideoWriter> output_videos;
    std::map<std::string, uint32_t> message_count;

  };

}



#endif