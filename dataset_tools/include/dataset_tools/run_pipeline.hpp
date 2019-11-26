#ifndef run_pipeline_h
#define run_pipeline_h

#include <ros/ros.h>

// include messages to write to bag file
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>



template <class MessageType>
class PipelineInput {
public:

  PipelineInput() {}
  ~PipelineInput() {}

  typedef boost::shared_ptr<const MessageType> MessageTypePtr;


  void Initialise(std::string topic) {
    ros::NodeHandle nh;
    publisher_ = nh.advertise<MessageType>(topic, 1);
  }

  void PublishMessage(MessageTypePtr message) {
    publisher_.publish(message);
  }

private:

  std::string topic_;
  ros::Publisher publisher_;

};

class PipelineConnectorOutput {
public:

  PipelineConnectorOutput(): message_received_(false) {

  }

  bool message_received_;

};


template <class MessageType>
class PipelineOutput : public PipelineConnectorOutput {
public:

  typedef boost::shared_ptr<MessageType> MessageTypePtr;

  PipelineOutput() {
    MessageTypePtr last_message_(new MessageType);
  }
  ~PipelineOutput() {}

  void Initialise(std::string topic, std::vector<PipelineConnectorOutput*> &pipe_container) {
    ros::NodeHandle nh;
    subscriber_ = nh.subscribe<MessageTypePtr>(topic, 1, &PipelineOutput::SubscriberCallback, this);
    pipe_container.push_back(this);
  }

  MessageTypePtr last_message;

private:

  void SubscriberCallback(const MessageTypePtr message) {
    last_message = message;
    message_received_ = true;
  }


  std::string topic_;
  ros::Subscriber subscriber_;
};


#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>

template <class MessageType>
class PipelineOutputImage : public PipelineConnectorOutput {
public:

  typedef boost::shared_ptr<const MessageType> MessageTypePtr;

  PipelineOutputImage() {
    MessageTypePtr last_message_(new MessageType);
  }
  ~PipelineOutputImage() {}

  void Initialise(std::string topic, std::vector<PipelineConnectorOutput*> &pipe_container) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    subscriber_ = it.subscribe(topic, 1, &PipelineOutputImage::SubscriberCallback, this);
    pipe_container.push_back(this);
  }

  MessageTypePtr last_message;

private:

  void SubscriberCallback(const MessageTypePtr &message) {
    last_message = message;
    message_received_ = true;
  }


  std::string topic_;
  image_transport::Subscriber subscriber_;
};


template <class MessageType>
class PipelineInputImage {
public:

  PipelineInputImage() {}
  ~PipelineInputImage() {}

  typedef boost::shared_ptr<const MessageType> MessageTypePtr;


  void Initialise(std::string topic) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    publisher_ = it.advertise(topic, 1);
  }

  void PublishMessage(MessageTypePtr message) {
    publisher_.publish(message);
  }

private:

  std::string topic_;
  image_transport::Publisher publisher_;

};



/*!
 * \brief Call an external pipeline (ros node) and wait for the responses
 *
 */

class RunPipeline {
public:
  RunPipeline();
  ~RunPipeline();

  bool WaitForMessages(float time_out = 1.);
  void ResetMessageFlags();

protected:

  std::vector<PipelineConnectorOutput*> pipes_out;
};




#endif
