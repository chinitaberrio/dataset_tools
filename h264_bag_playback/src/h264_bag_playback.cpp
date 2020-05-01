#include "h264_bag_playback.hpp"
#include <glob.h>

#include "boost/date_time/posix_time/posix_time.hpp"

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

#include <tf2_ros/static_transform_broadcaster.h>

#include "helper_functions.hpp"
#include "video.hpp"


// Nodelet problem is https://github.com/ros/ros_comm/issues/1474

// this main function allows the nodelet to be compiled as a node.
//  this has been done because there is a problem in ros melodic where
//  the rosbag library has a conflict with the compression functions
//  of the opencv library causing a run time seg fault.
int main(int argc, char **argv) {

  //Initialize Node and handles
  ros::init(argc, argv, "h264_bag_playback_node");
  ros::NodeHandle n;

  dataset_toolkit::h264_bag_playback bag_tools;
  //bag_tools.bypass_init();
  bag_tools.ReadFromBag();

  return 0;
}


namespace dataset_toolkit
{


h264_bag_playback::h264_bag_playback() :
        transformer_(std::make_shared<tf2_ros::Buffer>()),
        private_nh("~"),
        image_transport(public_nh),
        playback_start(ros::TIME_MIN),
        playback_end(ros::TIME_MAX){
  //transformer_ = std::make_shared<tf2::BufferCore>();
}


void h264_bag_playback::onInit() {
  timer = public_nh.createTimer(ros::Duration(0.1), &h264_bag_playback::timerCallback, this);
}


void h264_bag_playback::timerCallback(const ros::TimerEvent& event) {
  ReadFromBag();
}


void
h264_bag_playback::ScaleCameraInfoMsg(int original_width,
                                      int scaled_width,
                                      int original_height,
                                      int scaled_height,
                                      sensor_msgs::CameraInfo::Ptr &scaled_info_msg) {

  double scale_y = static_cast<double>(scaled_height) / original_height;
  double scale_x = static_cast<double>(scaled_width) / original_width;

  scaled_info_msg->height = scaled_height;
  scaled_info_msg->width = scaled_width;

  scaled_info_msg->K[0] = scaled_info_msg->K[0] * scale_x;  // fx
  scaled_info_msg->K[2] = scaled_info_msg->K[2] * scale_x;  // cx
  scaled_info_msg->K[4] = scaled_info_msg->K[4] * scale_y;  // fy
  scaled_info_msg->K[5] = scaled_info_msg->K[5] * scale_y;  // cy

  scaled_info_msg->P[0] = scaled_info_msg->P[0] * scale_x;  // fx
  scaled_info_msg->P[2] = scaled_info_msg->P[2] * scale_x;  // cx
  scaled_info_msg->P[3] = scaled_info_msg->P[3] * scale_x;  // T
  scaled_info_msg->P[5] = scaled_info_msg->P[5] * scale_y;  // fy
  scaled_info_msg->P[6] = scaled_info_msg->P[6] * scale_y;  // cy
}



void h264_bag_playback::ReadFromBag() {

  // parameter to scale the size of the images from the h264 playback
  int scaled_width = 0, scaled_height = 0;
  private_nh.param("output_width", scaled_width, 0);
  private_nh.param("output_height", scaled_height, 0);

  if (scaled_height && scaled_width) {
    ROS_INFO_STREAM("output images will be scaled to " << scaled_width << "x" << scaled_height);
  }
  else {
    ROS_INFO_STREAM("output images will NOT be scaled");
  }

  // parameter to limit the speed of playback to realtime
  bool limit_playback_speed;
  private_nh.param("limit_playback_speed", limit_playback_speed, true);

  // determine the bag file to playback
  std::string bag_file_name = "";
  private_nh.getParam("bag_file", bag_file_name);

  if (bag_file_name.empty()) {
    ROS_INFO_STREAM("Could not find bagfile " << bag_file_name);
    return;
  }

  // determine the file prefixes and initialise each camera
  std::string file_prefix = remove_last_of_string(bag_file_name, ".");
  std::string dataset_name = keep_last_of_string(file_prefix, "/");
  ROS_INFO_STREAM("Reading from bag: " << bag_file_name << " dataset name " << dataset_name);

  std::vector<std::string> file_list;
  get_files_pattern(file_prefix + "*.h264", file_list);

  // make a video object for each video file
  for (auto file_name: file_list) {
    std::string camera_name = keep_last_of_string(remove_last_of_string(file_name, "."), "-");

    Video new_video;
    videos[camera_name] = new_video;
    if (videos[camera_name].InitialiseVideo(camera_name, file_name)){
      ROS_INFO_STREAM("including video file: " << file_name << " for camera " << camera_name);
    }
    else {
      ROS_INFO_STREAM("FAIL to open video file: " << file_name << " for camera " << camera_name);
      return;
    }
  }

  // Attempt to open the bag file
  rosbag::Bag bag;
  bag.open(bag_file_name);

  if (!bag.isOpen()) {
    ROS_INFO_STREAM("Could not OPEN bagfile " << bag_file_name);
    return;
  }

  // determine whether to apply a time bias correction
  // (some datasets have an offset between the nvidia computer time and the ROS time)
  std::map<std::string, double> dataset_time_correction;
  private_nh.getParam("dataset_time_correction", dataset_time_correction);

  ros::Duration time_offset(0.0);
  if (dataset_time_correction.count(dataset_name)) {
    time_offset = ros::Duration(dataset_time_correction.at(dataset_name));
    ROS_INFO_STREAM("Time correction of " << time_offset.toSec() << " is being applied");
  }
  else {
    ROS_INFO_STREAM("No time correction parameters available for this dataset");
  }

  rosbag::View overall_view(bag);

  ros::Time bag_start_time = overall_view.getBeginTime();
  ros::Time bag_end_time = overall_view.getEndTime();
  auto bag_duration = (bag_end_time-bag_start_time).toSec();


  ROS_INFO_STREAM("Bag start time " << boost::posix_time::to_iso_extended_string(bag_start_time.toBoost()));
  ROS_INFO_STREAM("Bag end time " << boost::posix_time::to_iso_extended_string(bag_end_time.toBoost()));
  ROS_INFO_STREAM("Bag duration: " << bag_duration << " seconds");


  std::string start_time_param_string, end_time_param_string;
  float start_percentage, end_percentage;
  private_nh.getParam("time_start", start_time_param_string);
  private_nh.getParam("time_end", end_time_param_string);
  private_nh.param<float>("percentage_start", start_percentage, 0); // -1.1 is a random placeholder value to denote no param received
  private_nh.param<float>("percentage_end", end_percentage, 100);

  ROS_INFO_STREAM("Requested start percentage " << start_percentage );
  ROS_INFO_STREAM("Requested end percentage " << end_percentage );

  ros::Time requested_start_time = bag_start_time;
  ros::Time requested_end_time = bag_end_time;

  try {
    ros::Time test_start_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(start_time_param_string));
    requested_start_time = test_start_time;
    ROS_INFO_STREAM("Requested start time " << start_time_param_string << " is " << requested_start_time);
  }
  catch (...) {
    ROS_INFO_STREAM("Couldn't read start time string " << start_time_param_string);
  }

  try {
    ros::Time test_end_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(end_time_param_string));
    requested_end_time = test_end_time;
    ROS_INFO_STREAM("Requested end time " << end_time_param_string << " is " << requested_end_time);
  }
  catch (...) {
    ROS_INFO_STREAM("Couldn't read end time string " << end_time_param_string);
  }

  if(requested_start_time == bag_start_time && requested_end_time == bag_end_time){
      if(!(start_percentage>=0 && start_percentage<100)){
          start_percentage = 0;
      }
      if(!(end_percentage<=100 && end_percentage>0)){
          end_percentage = 100;
      }
      auto duration_percentage = end_percentage - start_percentage;
      if(duration_percentage<100 && duration_percentage>0){
          ROS_INFO_STREAM("Reading bag from " << start_percentage << "% to " << end_percentage << "%");

          requested_start_time = bag_start_time + ros::Duration(bag_duration / 100 * start_percentage);
          requested_end_time = bag_start_time + ros::Duration(bag_duration / 100 * end_percentage);
      }
  }


  if(requested_start_time == bag_start_time){
    ROS_INFO_STREAM("starting from the beginning: " << start_time_param_string);
  }else{
    ROS_INFO_STREAM("Playback start from : " << boost::posix_time::to_iso_extended_string(requested_start_time.toBoost()));
  }
  if(requested_end_time == bag_end_time){
    ROS_INFO_STREAM("running through to the end of the bag: " << end_time_param_string);
  }else{
    ROS_INFO_STREAM("Play until : " << boost::posix_time::to_iso_extended_string(requested_end_time.toBoost()));
  }
  ROS_INFO_STREAM("Playback duration : " << requested_end_time-requested_start_time << " seconds");

  // create a view and advertise each of the topics to publish
  rosbag::View view(bag, requested_start_time, requested_end_time);
  AdvertiseTopics(view);

  ros::Time start_ros_time = ros::Time::now();
  ros::Time start_frame_time = ros::Time(0);

  // tf static should be published by static tf broadcaster
  // so that if bag isn't played from begining, static will still be published
  StaticTfPublisher(bag);

  // creat a tf bag view object so that we can view future tf msgs
  std::vector<std::string> tf_topics{"tf", "/tf"};
  rosbag::View tf_view(bag, rosbag::TopicQuery(tf_topics));
  rosbag::View::iterator tf_iter = tf_view.begin();
  ros::Time last_tf_time(0.1);

  // for each message in the rosbag
  for(rosbag::MessageInstance const m: view)
  {
    std::string const& topic = m.getTopic();
    ros::Time const& time = m.getTime();

    if (topic == "/tf_static" || topic == "tf_static") {
      // static transforms are handled separately
      continue;
    }

    // all of the publishers should be available due to the AdvertiseTopics function
    std::map<std::string, ros::Publisher>::iterator pub_iter = publishers.find(m.getCallerId() + topic);
    ROS_ASSERT(pub_iter != publishers.end());

    // query the bag tf msgs so that transformer buffers a tf tree from (current msg time - 8s) to (current msg time + 2s)
    // this however does not affect replay publishing of tf msgs. Tf msgs are still published at their bag times
    while (tf_iter != tf_view.end()
        && last_tf_time < time + ros::Duration(2)) {
      // Load more transforms into the TF buffer
      auto tf = tf_iter->instantiate<tf2_msgs::TFMessage>();

      for (const auto &transform: tf->transforms) {
        transformer_->setTransform(transform, "zio", false);
        last_tf_time = transform.header.stamp;
      }
      tf_iter++;
    }


// MOVE THIS INTO THE BAG VIEW OBJECT
//    if (time < playback_start || time > playback_end)
//      continue;

    // For each camera info msg, check whether we have stored the calibration parameters for this camera
    if (keep_last_of_string(topic, "/") == "camera_info") {
      std::string camera_name = keep_last_of_string(remove_last_of_string(topic, "/"), "/");

      sensor_msgs::CameraInfo::ConstPtr cam_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
      if (cam_info_msg != NULL) {
        sensor_msgs::CameraInfo::Ptr scaled_info_msg(new sensor_msgs::CameraInfo());

        // copy the camera info parameters to the rescaled version
        *scaled_info_msg = *cam_info_msg;

        if (scaled_height && scaled_width) {
          ScaleCameraInfoMsg(cam_info_msg->width,
              scaled_width,
              cam_info_msg->height,
              scaled_height,
              scaled_info_msg);
        }

        if (!videos[camera_name].valid_camera_info) {
          videos[camera_name].InitialiseCameraInfo(*scaled_info_msg);
        }

        // todo: make this go through the MessagePublisher structure
        //MessagePublisher(pub_iter->second, scaled_info_msg);
        CameraInfoPublisher(pub_iter->second, scaled_info_msg);

        ros::spinOnce();
      }
    }

    // For each frame info msg, find the corresponding h.264 frame and publish/convert if necessary
    else if (keep_last_of_string(topic, "/") == "frame_info") {

      // initialise the frame start time
      if (start_frame_time == ros::Time(0)){
        start_frame_time = time;
      }

      ros::Duration time_difference = (time - start_frame_time) - (ros::Time::now() - start_ros_time);

      // hold back the playback to be realtime if limit_playback_speed parameter is set
      if (limit_playback_speed && time_difference.toSec() > 0) {
        time_difference.sleep();
      }

      std::string camera_name = keep_last_of_string(remove_last_of_string(topic, "/"), "/");

      gmsl_frame_msg::FrameInfo::ConstPtr frame_info_msg = m.instantiate<gmsl_frame_msg::FrameInfo>();
      if (frame_info_msg != NULL) {

        ros::Time nvidia_timestamp = ros::Time((frame_info_msg->camera_timestamp) / 1000000.0, ((frame_info_msg->camera_timestamp) % 1000000) * 1000.0);

        // calculate the time bias between the camera/ROS if not already done
        if (!camera_time_bias_flag) {
           camera_time_bias = frame_info_msg->header.stamp - nvidia_timestamp;
           camera_time_bias_flag = true;
        }

        ros::Time adjusted_image_stamp;

        if (fabs(camera_time_bias.toSec()) > 0.5) {
          adjusted_image_stamp = nvidia_timestamp + camera_time_bias - time_offset;
        }
        else {
          adjusted_image_stamp = nvidia_timestamp - time_offset;
        }

        // Check that someone has subscribed to this camera'frame_info_msg images
        if (!(videos[camera_name].corrected_publisher.getNumSubscribers() == 0 &&
            videos[camera_name].uncorrected_publisher.getNumSubscribers() == 0))
        {

          Video &current_video = videos[camera_name];
          if (current_video.frame_counter < frame_info_msg->frame_counter) {

            // try to jump forward through the video
            if (current_video.video_device.set(CV_CAP_PROP_POS_FRAMES, frame_info_msg->frame_counter)) {
              cv::Mat new_frame;
              while (ros::ok() && current_video.frame_counter < frame_info_msg->frame_counter) {
                current_video.video_device >> new_frame;
                current_video.frame_counter++;
                ROS_INFO_STREAM_THROTTLE(0.5, "throwing away frame for camera " << camera_name);
              }
            }
            // todo: this doesn't seem to work for h.264 probably due to the way it is encoded
            // this is worth revisiting
            // https://stackoverflow.com/questions/2974625/opencv-seek-function-rewind
            else {
              ROS_INFO_STREAM("tracking camera to frame " << camera_name);
              current_video.frame_counter = frame_info_msg->frame_counter;
            }
          }

          // check that the frame counter aligns with the number of frames in the video
          if (current_video.frame_counter == frame_info_msg->frame_counter) {

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

              cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

              // Check if someone wants the corrected (undistorted) camera images
              if (videos[camera_name].corrected_publisher.getNumSubscribers() > 0) {
                cv::Mat output_image;

                if (current_video.camera_info_msg.distortion_model == "rational_polynomial") {
                  cv::undistort(new_frame, output_image, current_video.camera_matrix, current_video.distance_coeffs);
                }
                else if (current_video.camera_info_msg.distortion_model == "equidistant") {
                  cv::remap(new_frame, output_image, current_video.map1, current_video.map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                }
                else {
                  ROS_INFO_STREAM("Unknown distortion model, skipping " << current_video.camera_info_msg.distortion_model);
                  continue;
                }

                cv_ptr->image = output_image;
                cv_ptr->encoding = "bgr8";
                cv_ptr->header.stamp = adjusted_image_stamp;
                cv_ptr->header.frame_id = frame_id_dict[camera_name];
                cv_ptr->header.seq = frame_info_msg->global_counter;

                auto image_message = cv_ptr->toImageMsg();
                ImagePublisher(current_video.corrected_publisher, image_message);
                ros::spinOnce();
                //current_video.corrected_publisher.publish(cv_ptr->toImageMsg());
              }

              // Check if someone wants the uncorrected camera images
              if (videos[camera_name].uncorrected_publisher.getNumSubscribers() > 0) {
                cv_ptr->image = new_frame;
                cv_ptr->encoding = "bgr8";
                cv_ptr->header.stamp = adjusted_image_stamp;
                cv_ptr->header.frame_id = frame_id_dict[camera_name];
                cv_ptr->header.seq = frame_info_msg->global_counter;

                //current_video.uncorrected_publisher.publish(cv_ptr->toImageMsg());
                auto image_message = cv_ptr->toImageMsg();
                ImagePublisher(current_video.uncorrected_publisher, image_message);
                ros::spinOnce();
              }
            }
          }
        }
      }

      // repubish the frame info message
      //pub_iter->second.publish(m);
      MessagePublisher(pub_iter->second, m);
      ros::spinOnce();
    }
    else {
      // publish the remaining messages
      //pub_iter->second.publish(m);
      MessagePublisher(pub_iter->second, m);
      ros::spinOnce();
    }

    // Spin once so that any other ros controls/pub/sub can be actioned
    ros::spinOnce();

    if (!ros::ok())
      break;
  }

  ROS_INFO_STREAM("completed playback");
  bag.close();
}



void
h264_bag_playback::CameraInfoPublisher(ros::Publisher &publisher, const sensor_msgs::CameraInfoConstPtr &message) {
  publisher.publish(message);
}


void
h264_bag_playback::MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
  publisher.publish(message);
}


void
h264_bag_playback::ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {
  publisher.publish(message);
}


void h264_bag_playback::StaticTfPublisher(rosbag::Bag &bag, bool do_publish) {

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  // Build a TF tree
  std::cout << "Loading static TF tree data" << std::endl;

  std::vector<std::string> topics;

  topics.push_back(std::string("tf_static"));
  topics.push_back(std::string("/tf_static"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for(rosbag::MessageInstance const &m: view) {
    const auto tf = m.instantiate<tf2_msgs::TFMessage>();
    if(do_publish)
      static_broadcaster.sendTransform(tf->transforms);
    for (const auto &transform: tf->transforms) {
      transformer_->setTransform(transform, "zio", true);
    }
  }
}

void
h264_bag_playback::AdvertiseTopics(rosbag::View &view) {

  // Create a publisher and advertise for all of our message types
  for(const rosbag::ConnectionInfo* c: view.getConnections())
  {
    // skip adding tf static. static should be published by static tf broadcaster
    // so that if bag isn't played from begining, static will still be published
    if (c->topic == "/tf_static" || c->topic == "tf_static") {
        continue;
    }

    ros::M_string::const_iterator header_iter = c->header->find("callerid");
    std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

    std::string callerid_topic = callerid + c->topic;

    std::map<std::string, ros::Publisher>::iterator pub_iter = publishers.find(callerid_topic);
    if (pub_iter == publishers.end()) {

      ros::AdvertiseOptions opts = rosbag::createAdvertiseOptions(c, 10);

      ros::Publisher pub = public_nh.advertise(opts);
      publishers.insert(publishers.begin(), std::pair<std::string, ros::Publisher>(callerid_topic, pub));

//      pub_iter = publishers.find(callerid_topic); // this line seems to be doing nothing here
    }
  }
}

  PLUGINLIB_EXPORT_CLASS(dataset_toolkit::h264_bag_playback, nodelet::Nodelet);

}
