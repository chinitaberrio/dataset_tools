#include "bag_static_transform_publisher.hpp"

#include <boost/algorithm/string.hpp>


void BagStaticTransformBroadcaster::ApplyCorrection(std::string correction_string, std::shared_ptr<tf2_ros::Buffer> &transformer_) {
    
    std::stringstream ss(correction_string);
    std::vector<std::string> result;

    while(ss.good()) {
      std::string substr;
      std::getline( ss, substr, ',' );
      boost::trim(substr);
      result.push_back( substr );
    }

    if (result.size() != 9) {
      ROS_ERROR_STREAM("Correction \"" << correction_string << "\" cannot be applied. Expecting comma separated values in the form frame_1, frame_2, x, y, z, qx, qy, qz, qw");
      return;
    }

    ROS_INFO_STREAM("Applying Correction \"" << correction_string << "\"");

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = result[0];
    transformStamped.child_frame_id = result[1];

    transformStamped.transform.translation.x = std::stof(result[2]);
    transformStamped.transform.translation.y = std::stof(result[3]);
    transformStamped.transform.translation.z = std::stof(result[4]);

    transformStamped.transform.rotation.x = std::stof(result[5]);
    transformStamped.transform.rotation.y = std::stof(result[6]);
    transformStamped.transform.rotation.z = std::stof(result[7]);
    transformStamped.transform.rotation.w = std::stof(result[8]);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static_broadcaster.sendTransform(transformStamped);
    transformer_->setTransform(transformStamped, "zio", true);
}
/*
    // replace base->velodyne for testing
    Eigen::Quaternionf velodyne_correction_rot(0.997, 0.00271, 0.0664, -0.0407);
    Eigen::Vector3f velodyne_correction_xyz(1.2, 0, 1.37);
  
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link" ;
    transformStamped.child_frame_id = "velodyne_front_link";

    transformStamped.transform.translation.x = velodyne_correction_xyz(0);
    transformStamped.transform.translation.y = velodyne_correction_xyz(1);
    transformStamped.transform.translation.z = velodyne_correction_xyz(2);

    transformStamped.transform.rotation.x = velodyne_correction_rot.x();
    transformStamped.transform.rotation.y = velodyne_correction_rot.y();
    transformStamped.transform.rotation.z = velodyne_correction_rot.z();
    transformStamped.transform.rotation.w = velodyne_correction_rot.w();

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static_broadcaster.sendTransform(transformStamped);
    transformer_->setTransform(transformStamped, "zio", true);
*/

//        tf_corrected->transforms.push_back(transformStamped);

//        tf2::Transform odom_tf_2;
//        tf2::fromMsg(transformStamped.transform, odom_tf_2);
//        odom_tf_2.getBasis().getRPY(r,p,yaw);
//        ROS_ERROR_STREAM("velodyne tf now reads yaw "<<yaw << " pitch " << p << " roll " << r);





void BagStaticTransformBroadcaster::StaticTfPublisher(rosbag::Bag &bag, bool do_publish, std::shared_ptr<tf2_ros::Buffer> &transformer_) {

    //static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    ros::NodeHandle private_nh("~");
    std::vector <std::string> topics;

    topics.push_back(std::string("tf_static"));
    topics.push_back(std::string("/tf_static"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int n_static_tf = 0;
    for (rosbag::MessageInstance const &m: view) {
        const auto tf = m.instantiate<tf2_msgs::TFMessage>();
        static_transforms.push_back(tf);

        for (const auto &transform: tf->transforms) {
    /*
        if (transform.child_frame_id == "velodyne_front_link") {
            // replace base->velodyne for testing

            tf2::Transform odom_tf;
            tf2::fromMsg(transform.transform, odom_tf);
            double r, p, yaw;
            odom_tf.getBasis().getRPY(r, p, yaw);
            ROS_ERROR_STREAM("original velodyne tf yaw " << yaw << " pitch " << p << " roll " << r);

            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "base_link";
            transformStamped.child_frame_id = "velodyne_front_link";

            transformStamped.transform.translation.x = transform.transform.translation.x;
            transformStamped.transform.translation.y = transform.transform.translation.y;
            transformStamped.transform.translation.z = transform.transform.translation.z;

            float new_roll, new_pitch, new_yaw;
            private_nh.param<float>("new_roll", new_roll, 0.);
            private_nh.param<float>("new_pitch", new_pitch, 0.);
            private_nh.param<float>("new_yaw", new_yaw, 0.);
            ROS_ERROR_STREAM("offseting velodyne tf yaw " << new_yaw << " pitch " << new_pitch << " roll " << new_roll);

            new_yaw += yaw;
            new_pitch += p;
            new_roll += r;

            tf2::Quaternion quat;
            quat.setRPY(new_roll, new_pitch, new_yaw);

            transformStamped.transform.rotation.x = quat.x();
            transformStamped.transform.rotation.y = quat.y();
            transformStamped.transform.rotation.z = quat.z();
            transformStamped.transform.rotation.w = quat.w();

            this->sendTransform(transformStamped);
            transformer_->setTransform(transformStamped, "zio", true);

            tf2::Transform odom_tf_2;
            tf2::fromMsg(transformStamped.transform, odom_tf_2);
            odom_tf_2.getBasis().getRPY(r, p, yaw);
            ROS_ERROR_STREAM("velodyne tf now reads yaw " << yaw << " pitch " << p << " roll " << r);

        } else 
        
        if (transform.child_frame_id == "utm") {
            continue;
        } 
        else */
        {
            transformer_->setTransform(transform, "zio", true);
        }

        if (do_publish) {
            //this->sendTransform(tf->transforms);
            this->sendTransform(transform);
        }
    }
    n_static_tf++;
    if (n_static_tf > 10)
    break;

  }
  ROS_INFO_STREAM("Loaded static TF tree data");
}
