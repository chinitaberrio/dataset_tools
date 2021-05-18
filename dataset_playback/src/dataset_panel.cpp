#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <QFileInfo>
#include <QThread>

#include <QDateTime>
#include <QSlider>

#include <QLineEdit>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QString>

#include "dataset_panel.h"

#include "ros/ros.h"
#include <cstdlib>


DatasetPanel::DatasetPanel( QWidget* parent )
    : QFrame( parent ),
      workerThread(NULL)
{
  nh_ = ros::NodeHandle("/");

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

    statusText = new QTextEdit();
    currentTimeText = new QLineEdit();

    QHBoxLayout* button_layout = new QHBoxLayout;
    button_layout->addWidget( start_button_ );
    button_layout->addWidget( pause_button_ );
    button_layout->addWidget( stop_button_ );

    // Lay out the fields
    QVBoxLayout* main_layout = new QVBoxLayout;

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

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(PollROS()));
    timer->start(100);

}


void DatasetPanel::selectBagFile(){
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

}



void DatasetPanel::sliderPressed(){
    // pause the playback

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }

}


void DatasetPanel::sliderMoved(int new_value) {

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {
      double start_time = workerThread->bag_playback.requested_start_time.toSec();
      double end_time = workerThread->bag_playback.requested_end_time.toSec();
      double requested_time = start_time + ((end_time - start_time) * new_value / 1000.);

      currentTimeText->setText("seek to time: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());
  }
}


void DatasetPanel::sliderReleased() {

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


void DatasetPanel::PollROS() {
  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    double start_time = workerThread->bag_playback.requested_start_time.toSec();
    double end_time = workerThread->bag_playback.requested_end_time.toSec();
    double current_time = workerThread->bag_playback.last_packet_time.toSec();

    double percentage_through = 0.;
    if (current_time - start_time > 0)
       percentage_through = 1000. * (double(current_time - start_time) / double(end_time - start_time));

    slider->setValue(percentage_through);
    currentTimeText->setText(QDateTime::fromSecsSinceEpoch(current_time).toString());
  }
  ros::spinOnce();
}


void DatasetPanel::startPressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_START;
  }
}

void DatasetPanel::stopPressed() {
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

void DatasetPanel::pausePressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }
}


