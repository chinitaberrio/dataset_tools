#include "main.hpp"
#include <glob.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <rosbag/bag.h>
#include <rosbag/player.h>
#include <rosbag/structures.h>
#include <rosbag/message_instance.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <gmsl_frame_msg/FrameInfo.h>

#include "helper_functions.hpp"
#include "video.hpp"


// Nodelet problem is https://github.com/ros/ros_comm/issues/1474

// this main function allows the nodelet to be compiled as a node.
//  this has been done because there is a problem in ros melodic where
//  the rosbag library has a conflict with the compression functions
//  of the opencv library causing a run time seg fault.
int main(int argc, char **argv) {
  //Initialize Node and handles
  ros::init(argc, argv, "h264_bag_tools_node");
  ros::NodeHandle n;

  dataset_toolkit::h264_bag_tools bag_tools;
  bag_tools.bypass_init();

  return 0;
}


namespace dataset_toolkit
{


h264_bag_tools::h264_bag_tools() : private_nh("~"), image_transport(public_nh) {
}


void h264_bag_tools::onInit() {

  int scaled_width;
  private_nh.param("output_width", scaled_width, 0);

  int scaled_height;
  private_nh.param("output_height", scaled_height, 0);

  bool limit_playback_speed;
  private_nh.param("limit_playback_speed", limit_playback_speed, true);

  if (scaled_height && scaled_width) {
    NODELET_INFO_STREAM("output images will be scaled to " << scaled_width << "x" << scaled_height);
  }
  else {
    NODELET_INFO_STREAM("output images will NOT be scaled");
  }
  std::string bag_file_name = "";
  private_nh.getParam("bag_file", bag_file_name);

  if (bag_file_name.empty()) {
    NODELET_INFO_STREAM("Could not find bagfile " << bag_file_name);
    return;
  }

  std::string file_prefix = remove_last_of_string(bag_file_name, ".");
  NODELET_INFO_STREAM("Reading from bag: " << bag_file_name << " with prefix " << file_prefix);

  std::vector<std::string> file_list;
  get_files_pattern(file_prefix + "*.h264", file_list);

  // make a video object for each video file
  for (auto file_name: file_list) {
    std::string camera_name = keep_last_of_string(remove_last_of_string(file_name, "."), "-");

    Video new_video;
    videos[camera_name] = new_video;
    if (videos[camera_name].InitialiseVideo(camera_name, file_name)){
      NODELET_INFO_STREAM("including video file: " << file_name << " for camera " << camera_name);
    }
    else {
      NODELET_INFO_STREAM("FAIL to open video file: " << file_name << " for camera " << camera_name);
      return;
    }
  }

  NODELET_INFO_STREAM("trying to OPEN bagfile " << bag_file_name);

  rosbag::Bag bag;
  bag.open(bag_file_name);

  /*if (!bag.isOpen()) {
    NODELET_INFO_STREAM("Could not OPEN bagfile " << bag_file_name);
    return;
  }*/

  rosbag::View view(bag);
  AdvertiseTopics(view);

  ros::Time start_ros_time = ros::Time::now();
  ros::Time start_frame_time = ros::Time(0);

  // For each message in the rosbag
  for(rosbag::MessageInstance const m: view)
  {
    std::string const& topic = m.getTopic();
    ros::Time const& time = m.getTime();
    std::string callerid = m.getCallerId();
    std::string callerid_topic = callerid + topic;

    std::map<std::string, ros::Publisher>::iterator pub_iter = publishers.find(callerid_topic);

    ROS_ASSERT(pub_iter != publishers.end());

    // For each camera info msg, check whether we have stored the calibration parameters for this camera
    if (keep_last_of_string(topic, "/") == "camera_info") {
      std::string camera_name = keep_last_of_string(remove_last_of_string(topic, "/"), "/");

      sensor_msgs::CameraInfo::ConstPtr s = m.instantiate<sensor_msgs::CameraInfo>();
      if (s != NULL) {
        sensor_msgs::CameraInfo scaled_info_msg = *s;


        if (scaled_height && scaled_width) {
          double scale_y = static_cast<double>(scaled_height) / s->height;
          double scale_x = static_cast<double>(scaled_width) / s->width;

          scaled_info_msg.height = scaled_height;
          scaled_info_msg.width = scaled_width;

          scaled_info_msg.K[0] = scaled_info_msg.K[0] * scale_x;  // fx
          scaled_info_msg.K[2] = scaled_info_msg.K[2] * scale_x;  // cx
          scaled_info_msg.K[4] = scaled_info_msg.K[4] * scale_y;  // fy
          scaled_info_msg.K[5] = scaled_info_msg.K[5] * scale_y;  // cy

          scaled_info_msg.P[0] = scaled_info_msg.P[0] * scale_x;  // fx
          scaled_info_msg.P[2] = scaled_info_msg.P[2] * scale_x;  // cx
          scaled_info_msg.P[3] = scaled_info_msg.P[3] * scale_x;  // T
          scaled_info_msg.P[5] = scaled_info_msg.P[5] * scale_y;  // fy
          scaled_info_msg.P[6] = scaled_info_msg.P[6] * scale_y;  // cy
        }

        if (!videos[camera_name].valid_camera_info) {
          videos[camera_name].InitialiseCameraInfo(scaled_info_msg);
        }

        pub_iter->second.publish(scaled_info_msg);
      }
    }

    // For each frame info msg, find the corresponding h.264 frame and publish/convert if necessary
    else if (keep_last_of_string(topic, "/") == "frame_info") {

      // initialise the frame start time
      if (start_frame_time == ros::Time(0)){
        start_frame_time = time;
      }

      ros::Duration time_difference = (time - start_frame_time) - (ros::Time::now() - start_ros_time);

      if (limit_playback_speed && time_difference.toSec() > 0) {
        time_difference.sleep();
      }

      std::string camera_name = keep_last_of_string(remove_last_of_string(topic, "/"), "/");

      gmsl_frame_msg::FrameInfo::ConstPtr s = m.instantiate<gmsl_frame_msg::FrameInfo>();
      if (s != NULL) {
        
        ros::Time temp_stamp,camera_stamp,temp;
      
        if (save_time_diff )
           { 

           temp = ros::Time ((s->camera_timestamp)/1000000,((s->camera_timestamp)%1000000)*1000);
           dur=s->header.stamp-temp;
           save_time_diff=false;
           }
         temp_stamp=  ros::Time ((s->camera_timestamp)/1000000,((s->camera_timestamp)%1000000)*1000);
         
         if  (fabs(dur.toSec())>0.5)
         camera_stamp=temp_stamp+dur;
         else
         camera_stamp=temp_stamp;
        
        
         
        // Check that someone has subscribed to this camera's images
        if (!(videos[camera_name].corrected_publisher.getNumSubscribers() == 0 &&
            videos[camera_name].uncorrected_publisher.getNumSubscribers() == 0))
        {

          Video &current_video = videos[camera_name];
          if (current_video.frame_counter < s->frame_counter) {

            // try to jump forward through the video
            if (current_video.video_device.set(CV_CAP_PROP_POS_FRAMES, s->frame_counter)) {
              cv::Mat new_frame;
              while (ros::ok() && current_video.frame_counter < s->frame_counter) {
                current_video.video_device >> new_frame;
                current_video.frame_counter++;
                NODELET_INFO_STREAM_THROTTLE(0.5, "throwing away frame for camera " << camera_name);
              }
            }
            // todo: this doesn't seem to work for h.264 probably due to the way it is encoded
            // this is worth revisiting
            // https://stackoverflow.com/questions/2974625/opencv-seek-function-rewind
            else {
              NODELET_INFO_STREAM("tracking camera to frame " << camera_name);
              current_video.frame_counter = s->frame_counter;
            }
          }

          // check that the frame counter aligns with the number of frames in the video
          if (current_video.frame_counter == s->frame_counter) {

            cv::Mat new_frame;

            if (scaled_height && scaled_width) {
              cv::Mat unresized_frame;
              current_video.video_device >> unresized_frame;

              cv::Size reduced_size = cv::Size(scaled_width, scaled_height);
              cv::resize(unresized_frame, new_frame, reduced_size);
            }
            else {
              current_video.video_device >> new_frame;
            }

            current_video.frame_counter++;



            if (current_video.valid_camera_info) {

              //uint32_t image_width = current_video.camera_info_msg.width;
              //uint32_t image_height = current_video.camera_info_msg.height;

              //cv::Size image_size = cv::Size(image_width, image_height);

              cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

              // Check if someone wants the corrected (undistorted) camera images
              if (videos[camera_name].corrected_publisher.getNumSubscribers() > 0) {
                cv::Mat output_image;
                //cv::Size reduced_size;

                if (current_video.camera_info_msg.distortion_model == "rational_polynomial") {
                  cv::undistort(new_frame, output_image, current_video.camera_matrix, current_video.distance_coeffs);
                }
                else if (current_video.camera_info_msg.distortion_model == "equidistant") {
                  cv::remap(new_frame, output_image, current_video.map1, current_video.map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                }
                else {
                  NODELET_INFO_STREAM("Unknown distortion model, skipping " << current_video.camera_info_msg.distortion_model);
                  continue;
                }

                /*if (scaled_height && scaled_width) {
                  cv::Size reduced_size = cv::Size(scaled_width, scaled_height);
                  cv::resize(output_image, scaled_image, reduced_size); //resize image
                  cv_ptr->image = scaled_image;
                }
                else {
                  cv_ptr->image = output_image;
                }
                */

                cv_ptr->image = output_image;
                cv_ptr->encoding = "bgr8";
                cv_ptr->header.stamp = camera_stamp;
                cv_ptr->header.frame_id = frame_id_dict[camera_name];

                current_video.corrected_publisher.publish(cv_ptr->toImageMsg());
              }

              // Check if someone wants the uncorrected camera images
              if (videos[camera_name].uncorrected_publisher.getNumSubscribers() > 0) {
                //cv::Mat scaled_image;
                //cv::Size reduced_size;

                /*if (scaled_height && scaled_width) {
                  reduced_size = cv::Size(scaled_width, scaled_height);
                  cv::resize(new_frame, scaled_image, reduced_size);//resize image
                  cv_ptr->image = scaled_image;
                }
                else {
                  cv_ptr->image = new_frame;
                }
                 */

                cv_ptr->image = new_frame;
                cv_ptr->encoding = "bgr8";
                cv_ptr->header.stamp = camera_stamp;
                cv_ptr->header.frame_id = frame_id_dict[camera_name];

                current_video.uncorrected_publisher.publish(cv_ptr->toImageMsg());
              }
            }
          }
        }
      }

      // repubish the frame info message
      pub_iter->second.publish(m);
    }
    else {
      // publish the remaining messages
      pub_iter->second.publish(m);
    }


    // Spin once so that any other ros controls/pub/sub can be actioned
    ros::spinOnce();

    if (!ros::ok())
      break;
  }

  NODELET_INFO_STREAM("completed playback");
  bag.close();
}


void
h264_bag_tools::AdvertiseTopics(rosbag::View &view) {

  // Create a publisher and advertise for all of our message types
  for(const rosbag::ConnectionInfo* c: view.getConnections())
  {
    ros::M_string::const_iterator header_iter = c->header->find("callerid");
    std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

    std::string callerid_topic = callerid + c->topic;

    std::map<std::string, ros::Publisher>::iterator pub_iter = publishers.find(callerid_topic);
    if (pub_iter == publishers.end()) {

      ros::AdvertiseOptions opts = rosbag::createAdvertiseOptions(c, 10);
      ros::Publisher pub = public_nh.advertise(opts);
      publishers.insert(publishers.begin(), std::pair<std::string, ros::Publisher>(callerid_topic, pub));

      pub_iter = publishers.find(callerid_topic);
    }
  }
}


}
