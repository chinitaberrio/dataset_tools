#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <QDomDocument>
#include <QDomNodeList>

#include <QFileInfo>
#include <QThread>

#include <QDateTime>
#include <QGraphicsPathItem>
#include <QSlider>

#include <QPainterPath>
#include <QGraphicsSceneMouseEvent>

#include <QPainter>
#include <QLineEdit>
#include <QRegularExpression>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QString>

#include <QGraphicsSvgItem>
#include <QSvgWidget>


#include <geometry_msgs/Twist.h>

#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "bus_stop_panel.h"

#include "ros/ros.h"

#include "zio_navigation_msgs/GetBusStops.h"

#include <cstdlib>

#include <QTimer>


BusStopPanel::BusStopPanel( QWidget* parent )
    : QFrame( parent ),
      workerThread(NULL)
{
  nh_ = ros::NodeHandle("/");

  /*
  XmlRpc::XmlRpcValue pose_set;
  nh_.getParam("qt_bus_stop/pose_set", pose_set);

  std::list<std::string> pose_names;

  for (auto sample_pose = pose_set.begin(); sample_pose != pose_set.end(); ++sample_pose) {
    ROS_INFO_STREAM("reading destination: " << sample_pose->first);
    pose_names.push_back(sample_pose->first);
  }

  for (auto pose_name: pose_names) {
    //auto v = pose_set[pose_name];
    if (pose_set[pose_name].hasMember("position") &&
        pose_set[pose_name].hasMember("orientation")) {

      destination_map[pose_name] = geometry_msgs::PoseStamped();
      destination_map[pose_name].pose.position.x = pose_set[pose_name]["position"]["x"];
      destination_map[pose_name].pose.position.y = pose_set[pose_name]["position"]["y"];
      destination_map[pose_name].pose.position.z = pose_set[pose_name]["position"]["z"];
      destination_map[pose_name].pose.orientation.x = pose_set[pose_name]["orientation"]["x"];
      destination_map[pose_name].pose.orientation.y = pose_set[pose_name]["orientation"]["y"];
      destination_map[pose_name].pose.orientation.z = pose_set[pose_name]["orientation"]["z"];
      destination_map[pose_name].pose.orientation.w = pose_set[pose_name]["orientation"]["w"];

      destination_map[pose_name].header.frame_id = "map";
    }
  }

  XmlRpc::XmlRpcValue relations_set;
  nh_.getParam("qt_bus_stop/relationships", relations_set);

  //for (auto relation = relations_set.begin(); relation != relations_set.end(); ++relation) {
  for (int i = 0; i < relations_set.size(); ++i) {
    std::cout << relations_set[i]["from"] << ": " << relations_set[i]["to"] << std::endl;
    connections.push_back(std::make_pair<std::string, std::string>(std::string(relations_set[i]["from"]), std::string(relations_set[i]["to"])));
  }


  graphics_scene = new QBusStopScene();
    graphics_view = new QBusStopView();
    graphics_view->setScene(graphics_scene);
    graphics_view->setRenderHint(QPainter::Antialiasing);
    graphics_view->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    graphics_view->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
*/
   // start_msg_pub = nh_.advertise<std_msgs::Empty>("zio_navigation_server/start_exe", 1);
   // stop_msg_pub = nh_.advertise<actionlib_msgs::GoalID>("zio_navigation_server/stop_exe", 1);
   // bus_stop_name_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    //bus_stop_state_machine_sub = nh_.subscribe("bus_stop_bt/tip", 1,
    //    &BusStopPanel::StatusCallback, this);

  //  bus_stops_service_client_ = nh_.serviceClient<zio_navigation_msgs::GetBusStops>(
    //    "zio_navigation_server/get_bus_stops");


    file_select_button_ = new QPushButton( "Select file");







    slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 999);
    slider->setValue(0);

    start_button_ = new QPushButton( "Play");
    pause_button_ = new QPushButton( "Pause");
    stop_button_ = new QPushButton( "Stop");


    //start_button_->setEnabled(false);
    start_button_->setFixedHeight(100);
    pause_button_->setFixedHeight(100);
    stop_button_->setFixedHeight(100);

    QFont font = start_button_->font();
    font.setPointSize(32);
    font.setWeight(QFont::Bold);
    start_button_->setFont(font);
    start_button_->setStyleSheet("");

    pause_button_->setFont(font);
    pause_button_->setStyleSheet("");

    stop_button_->setFont(font);
    stop_button_->setStyleSheet("");

    // Add dummy data to the bus stop display
//    std::vector<QString> items;
//    items.push_back(QString("Waiting"));
//    items.push_back(QString("No Bus Stops"));
//    std::pair<std::string, std::string> connection("Waiting","No Bus Stops");

//    connections.push_back(connection);
//    graphics_view->ViewGraph(items, connections);
//    graphics_view->fitInView(graphics_scene->itemsBoundingRect(), Qt::KeepAspectRatio);
//    RefreshTrajectoryList();

    statusText = new QTextEdit();
    currentTimeText = new QLineEdit();


    QHBoxLayout* button_layout = new QHBoxLayout;
    button_layout->addWidget( start_button_ );
    button_layout->addWidget( pause_button_ );
    button_layout->addWidget( stop_button_ );

    // Lay out the fields
    QVBoxLayout* main_layout = new QVBoxLayout;
//    main_layout->addWidget(graphics_view);

    main_layout->addLayout(button_layout);
    main_layout->addWidget(file_select_button_);

    main_layout->addWidget(statusText);

    main_layout->addWidget(slider);
    main_layout->addWidget(currentTimeText);

    setLayout( main_layout );



    connect( file_select_button_, SIGNAL( pressed() ), this, SLOT( selectBagFile() ));
    connect( start_button_, SIGNAL( pressed() ), this, SLOT( startPressed() ));
    connect( pause_button_, SIGNAL( pressed() ), this, SLOT( pausePressed() ));
    connect( stop_button_, SIGNAL( pressed() ), this, SLOT( stopPressed() ));

    connect( slider, SIGNAL(sliderPressed()), this, SLOT(sliderPressed()));
    connect( slider, SIGNAL(sliderMoved(int)), this, SLOT(sliderMoved(int)));
    connect( slider, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

 //   connect( graphics_scene, SIGNAL(NewBusStopSelection(QString)), this, SLOT(BusMapPressed(QString)));

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(PollROS()));
    timer->start(100);

    ROS_INFO_STREAM("Bus stop RViz plugin initialised");
}


void BusStopPanel::selectBagFile(){
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open dataset file"), "/media/stew", tr("Bag Files (*.bag)"));
  std::cout << fileName.toStdString() << std::endl;

  workerThread = new DatasetThread();//this);

  workerThread->file_name = fileName.toStdString();

  connect(workerThread, SIGNAL(resultReady(QString)), this, SLOT(handleResults(QString)));
  connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));
  workerThread->start();

  statusText->setText("Loading dataset");

  while (workerThread->current_state == DatasetThread::PLAYBACK_LOADING) {
//    std::cout << "loading..." << std::endl;
    ros::spinOnce();
  }

  double start_time = workerThread->bag_playback.requested_start_time.toSec();
  double end_time = workerThread->bag_playback.requested_end_time.toSec();
  double current_time = workerThread->bag_playback.last_packet_time.toSec();

  statusText->setText("Dataset details");
  statusText->append("Start time: " + QDateTime::fromSecsSinceEpoch(start_time).toString());
  statusText->append("End time: " + QDateTime::fromSecsSinceEpoch(end_time).toString());

  // get folder where the files are stored
  QFileInfo bag_file_folder(workerThread->bag_playback.bags.front()->bag_file_name.c_str());
  statusText->append("location: " + bag_file_folder.absoluteDir().dirName());


  for (auto bag: workerThread->bag_playback.bags) {
    QFileInfo bag_file(QString(bag->bag_file_name.c_str()));
    statusText->append("bag: " + bag_file.fileName());
  }

  for (auto video: workerThread->bag_playback.videos) {
    QFileInfo video_file(QString(video.second.file_name.c_str()));
    statusText->append("video: " + video_file.fileName());
  }


  currentTimeText->setText(QDateTime::fromSecsSinceEpoch(current_time).toString());

/*
  std::cout << "start ros time " << workerThread->bag_playback.start_ros_time << std::endl <<
               "requested start time " << workerThread->bag_playback.requested_start_time << std::endl <<
               "requested end time " << workerThread->bag_playback.requested_end_time << std::endl <<
               "start ros time " << workerThread->bag_playback.start_ros_time << std::endl;
*/
}



void BusStopPanel::sliderPressed(){
    // pause the playback

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }

}


void BusStopPanel::sliderMoved(int new_value) {

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {
      double start_time = workerThread->bag_playback.requested_start_time.toSec();
      double end_time = workerThread->bag_playback.requested_end_time.toSec();
      double requested_time = start_time + ((end_time - start_time) * new_value / 1000.);

      currentTimeText->setText("seek to time: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());
  }
}


void BusStopPanel::sliderReleased() {

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {

    int requested_value = slider->value();

    double start_time = workerThread->bag_playback.requested_start_time.toSec();
    double end_time = workerThread->bag_playback.requested_end_time.toSec();
    double requested_time = start_time + ((end_time - start_time) * requested_value / 1000.);

    currentTimeText->setText("loading time: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());

    workerThread->seek_time = ros::Time(requested_time);
    workerThread->current_state = DatasetThread::PLAYBACK_SEEK;

    currentTimeText->setText("still seeking: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());

    while (workerThread->current_state == DatasetThread::PLAYBACK_SEEK) {
      ros::spinOnce();
    }

  }
}


void BusStopPanel::PollROS() {
  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    double start_time = workerThread->bag_playback.requested_start_time.toSec();
    double end_time = workerThread->bag_playback.requested_end_time.toSec();
    double current_time = workerThread->bag_playback.last_packet_time.toSec();

    double percentage_through = 0.;
    if (current_time - start_time > 0)
       percentage_through = 1000. * (double(current_time - start_time) / double(end_time - start_time));

    slider->setValue(percentage_through);
    currentTimeText->setText(QDateTime::fromSecsSinceEpoch(current_time).toString());
    //std::cout << workerThread->bag_playback.requested_start_time << ", " <<
//                 workerThread->bag_playback.requested_end_time << ", " <<
//                 workerThread->bag_playback.last_packet_time << std::endl;
    //std::cout << start_time << ", " << current_time << ", " << end_time << ", " << percentage_through << std::endl;
  }
  ros::spinOnce();
}


void BusStopPanel::BusMapPressed(QString stop_name) {
    ROS_INFO_STREAM(stop_name.toStdString().c_str() << " pressed");


    geometry_msgs::PoseStamped &send_pose = destination_map[stop_name.toStdString()];
    send_pose.header.stamp = ros::Time::now();
    bus_stop_name_pub.publish(send_pose);

}

void BusStopPanel::StatusCallback(const py_trees_msgs::BehaviourConstPtr& msg) {
 /* if (msg->name == "NewBusStop" || msg->name == "GetGoal") {
    // Clear any stops
    RefreshTrajectoryList();
    // Uncolour the start button
    start_stop_button_->setStyleSheet(QString(""));
    start_stop_button_->setText("Start");
    graphics_view->setEnabled(true);
    start_stop_button_->setEnabled(false);
  } else if (msg->name == "WaitForStart") {
    // Set start to green
    start_stop_button_->setStyleSheet(QString("QPushButton {background: rgb(0,230,0);}"));
    start_stop_button_->setEnabled(true);
    // also set the text to "Start"
    start_stop_button_->setText("Start");
  } else {
    // Set start to orange when running
    start_stop_button_->setStyleSheet(QString("QPushButton {background: rgb(230,0,0);}"));
    start_stop_button_->setText("Stop");
    start_stop_button_->setEnabled(true);
    graphics_view->setEnabled(false);
  }
  */
}

void BusStopPanel::RefreshTrajectoryList() {

  std::vector<QString> items;
  for (auto &destination: destination_map) {
    items.push_back(QString(destination.first.c_str()));
  }

  //std::vector<std::pair<unsigned int, unsigned int>> connections;
  // Assume start->end pairs
/*  auto data = srv.response.connections.data;
  for (unsigned int idx=0; idx<data.size(); idx+=2) {
    std::pair<unsigned int, unsigned int> connection(data[idx],data[idx+1]);
    connections.push_back(connection);
  }
*/
  graphics_view->ViewGraph(items, connections);

  /*
    ROS_INFO("Requesting bus stop names from list service");

    zio_navigation_msgs::GetBusStops srv;
    if (bus_stops_service_client_.call(srv))
    {
        ROS_INFO_STREAM("Received a list of " << srv.response.names.size()
            << " bus stops from list service");
        if (srv.response.names.size()==0) {
          return;
        }


        std::vector<QString> items;
        for( auto &bus_stop_name: srv.response.names) {
            items.push_back(QString(bus_stop_name.data.c_str()));
        }
        std::vector<std::pair<unsigned int, unsigned int>> connections;
        // Assume start->end pairs
        auto data = srv.response.connections.data;
        for (unsigned int idx=0; idx<data.size(); idx+=2) {
          std::pair<unsigned int, unsigned int> connection(data[idx],data[idx+1]);
          connections.push_back(connection);
        }

        graphics_view->ViewGraph(items, connections);
    }
    else
    {
        ROS_INFO("Failed to get a response from bus stops list service");
    }
    */
}


void BusStopPanel::startPressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_START;
  }
}

void BusStopPanel::stopPressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_STOP;

    std::cout << "exiting playback thread" << std::endl;
    workerThread->quit();
    std::cout << "waiting for finish" << std::endl;
    workerThread->wait();
    std::cout << "deleting playback thread" << std::endl;

    delete workerThread;
    workerThread = NULL;

  }
}

void BusStopPanel::pausePressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }
/*  if (start_stop_button_->text() == "Start") {
    std_msgs::Empty msg;
    start_msg_pub.publish(msg);
    ROS_INFO("Requesting the vehicle to start a trajectory");
  } else {
    actionlib_msgs::GoalID msg;
    stop_msg_pub.publish(msg);
    ROS_INFO("Requesting the vehicle to stop a trajectory");
  }*/
}


