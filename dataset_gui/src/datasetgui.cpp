#include "datasetgui.h"
#include "ui_datasetgui.h"

#include <cstdlib>
#include "terminal_colours.h"
#include <QTimer>
#include <QDateTime>
#include <dataset_msgs/DatasetEvent.h>

void DatasetGUI::InitialiseWindow() {

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string initial_folder = "";
  private_nh.param<std::string>("folder", initial_folder, "/");
  boost::filesystem::current_path(initial_folder);

  std::cout << "read folder parameter " << initial_folder << std::endl;

  std::string path_check = boost::filesystem::current_path().string();
  ROS_INFO_STREAM("setting current_path to " << path_check);

//    ui->timeSlider->setValue(0);
//    ui->progressBar->setValue(0);
    
  resetGUI();    

  for (int column = 0; column < tree_model->columnCount(); ++column)
    ui->treeView->resizeColumnToContents(column);

  ui->checkPlaybackRealtime->setChecked(true);

  // colour of text edit areas
  QPalette p = ui->preprocessingTextEdit->palette(); // define pallete for textEdit.. 
  p.setColor(QPalette::Base, Qt::darkGray); // set color "Red" for textedit base
  //p.setColor(QPalette::Text, color); // set text color which is selected from color pallete
  ui->preprocessingTextEdit->setPalette(p); // change textedit palette

  p = ui->statusText->palette(); // define pallete for textEdit.. 
  p.setColor(QPalette::Base, Qt::darkGray); // set color "Red" for textedit base
  p.setColor(QPalette::Text, Qt::white); // set text color which is selected from color pallete
  ui->statusText->setPalette(p); // change textedit palette

    connect( ui->selectButton, SIGNAL( pressed() ), this, SLOT( selectBagFile() ));
    connect( ui->startButton, SIGNAL( pressed() ), this, SLOT( startPressed() ));
    connect( ui->pauseButton, SIGNAL( pressed() ), this, SLOT( pausePressed() ));
    connect( ui->stopButton, SIGNAL( pressed() ), this, SLOT( stopPressed() ));
    connect( ui->convertButton, SIGNAL( pressed() ), this, SLOT( convertPressed() ));
    connect( ui->eventGeneratorButton, SIGNAL( pressed() ), this, SLOT( eventGeneratorPressed() ));

//    connect( ui->angleLeftLane, SIGNAL( pressed() ), this, SLOT( angleLeftLane() ));
//    connect( ui->angleRightLane, SIGNAL( pressed() ), this, SLOT( angleRightLane() ));

    connect( ui->addOneMin, SIGNAL( pressed() ), this, SLOT( addMinute() ));
    connect( ui->subtractOneMin, SIGNAL( pressed() ), this, SLOT( subtractMinute() ));
    connect( ui->addOneSec, SIGNAL( pressed() ), this, SLOT( addSecond() ));
    connect( ui->subtractOneSec, SIGNAL( pressed() ), this, SLOT( subtractSecond() ));

    connect( ui->recordEventButton, SIGNAL( pressed() ), this, SLOT( recordEvent() ));
    connect( ui->saveEventsButton, SIGNAL( pressed() ), this, SLOT( saveEventsToFile() ));

    connect(ui->treeView,SIGNAL(clicked(QModelIndex)),this,SLOT(OnClickedTree(QModelIndex)));

    connect( ui->checkPlaybackFast, SIGNAL( pressed() ), this, SLOT( changePlaybackFast() ));
    connect( ui->checkPlaybackDouble, SIGNAL( pressed() ), this, SLOT( changePlaybackDouble() ));
    connect( ui->checkPlaybackRealtime, SIGNAL( pressed() ), this, SLOT( changePlaybackRealtime() ));
    connect( ui->checkPlaybackHalf, SIGNAL( pressed() ), this, SLOT( changePlaybackHalf() ));
    connect( ui->checkPlaybackQuarter, SIGNAL( pressed() ), this, SLOT( changePlaybackQuarter() ));

    connect( ui->bagAnalysePosOnly, SIGNAL( pressed() ), this, SLOT(bagAnalysePosOnly() ));


    connect( ui->timeSlider, SIGNAL(sliderPressed()), this, SLOT(sliderPressed()));
    connect( ui->timeSlider, SIGNAL(sliderMoved(int)), this, SLOT(sliderMoved(int)));
    connect( ui->timeSlider, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(PollROS()));
    timer->start(100);

}

DatasetGUI::DatasetGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::DatasetGUI),
    workerThread(NULL),
    processorThread(NULL),
    tree_model(NULL)
{
    ui->setupUi(this);


}

DatasetGUI::~DatasetGUI()
{
    delete ui;
}


void DatasetGUI::selectBagFile(){

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string initial_folder = "";
  private_nh.param<std::string>("folder", initial_folder, "/");

  QString fileName = QFileDialog::getOpenFileName(this, tr("Open dataset file"), initial_folder.c_str(), tr("Bag Files (*.bag)"));
  std::cout << fileName.toStdString() << std::endl;

  resetGUI();

  workerThread = new DatasetThread();//this);

  workerThread->file_name = fileName.toStdString();

  connect(workerThread, SIGNAL(resultReady(QString)), this, SLOT(handleResults(QString)));
  connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));
  workerThread->start();

  ui->statusText->setText("Loading dataset");

  while (workerThread->current_state == DatasetThread::PLAYBACK_LOADING) {
//    std::cout << "loading..." << std::endl;
    ros::spinOnce();
  }

  double start_time = workerThread->bag_playback.requested_start_time.toSec();
  double end_time = workerThread->bag_playback.requested_end_time.toSec();
  double current_time = workerThread->bag_playback.last_packet_time.toSec();

  ui->statusText->setText("Dataset details");
  ui->statusText->append("Start time: " + QDateTime::fromSecsSinceEpoch(start_time).toString());
  ui->statusText->append("End time: " + QDateTime::fromSecsSinceEpoch(end_time).toString());

  // get folder where the files are stored
  QFileInfo bag_file_folder(workerThread->bag_playback.bags.front()->bag_file_name.c_str());
  ui->statusText->append("location: " + bag_file_folder.absoluteDir().dirName());


  for (auto bag: workerThread->bag_playback.bags) {
    QFileInfo bag_file(QString(bag->bag_file_name.c_str()));
    ui->statusText->append("bag: " + bag_file.fileName());
  }

  for (auto video: workerThread->bag_playback.videos) {
    QFileInfo video_file(QString(video.second.file_name.c_str()));
    ui->statusText->append("video: " + video_file.fileName());
  }


  ui->currentTimeText->setText(QDateTime::fromSecsSinceEpoch(current_time).toString());

  for (auto event_list: workerThread->bag_playback.dataset_events) {
    QString category = QString(event_list.first.c_str()).split('/').last();

    std::cout << "read a category " << category.toStdString() << std::endl;

    for (auto event_category_list: event_list.second) {
      addEventToCategory(category.toStdString(), event_category_list);
      std::cout << "data " << event_category_list->header.stamp.toSec() << ", " << event_category_list->event_description << std::endl;
    }
  }

  boost::filesystem::path p(fileName.toStdString());
  boost::filesystem::path dir = p.parent_path();
  boost::filesystem::current_path(dir);

  ui->checkPlaybackRealtime->setChecked(true);
  changePlaybackRealtime();
  //ui->statusText->append("playback time scaled to " + QString::number(workerThread->bag_playback.scale_playback_speed));

}



void DatasetGUI::resetGUI(){
  // reset GUI
  ui->timeSlider->setValue(0.);
  ui->progressBar->setValue(0.);
  ui->statusText->clear();
  ui->currentTimeText->clear();
  
  ui->currentEventTimeText->clear();
  ui->eventText->clear();

  ui->checkPlaybackRealtime->setChecked(true);


  if (tree_model) {
    delete tree_model;
  }

  QStringList headers;
  headers << "Event time" << "Event description";

  QString data = "\tlocation\t\t\nnavigation\t\t\t\nmovement\t\t\t\n";

  tree_model = new TreeModel(headers, data);

  ui->treeView->setModel(tree_model);

  
}



void DatasetGUI::saveEventsToFile() {

  if (workerThread && !workerThread->bag_playback.bags.empty()) {
    QString event_bag_name = workerThread->bag_playback.bags.front()->bag_file_name.c_str();
    event_bag_name.insert(event_bag_name.lastIndexOf(".bag"), ".event");
    std::cout << "Writing events to " << event_bag_name.toStdString() << std::endl;

    rosbag::Bag event_bag;
    event_bag.open(event_bag_name.toStdString(), rosbag::bagmode::Write);

    //  Get top-level first.
    for (int i = 0; i < tree_model->rowCount(); ++i) {
      //  Use whatever column you are interested in.

      std::string category = tree_model->index(i, 0).data().toString().toLower().replace(' ', '_').toStdString();

      std::cout << "category: " << category << std::endl;

      for (int j = 0; j < tree_model->rowCount(tree_model->index(i, 0)); ++j) {
        //children << children[i].child( j, 0 );

        dataset_msgs::DatasetEvent new_event;

        auto boost_time = boost::posix_time::from_iso_extended_string(tree_model->index(i, 0).child(j, 0).data().toString().toStdString());
        boost::posix_time::time_duration diff = boost_time - boost::posix_time::from_time_t(0);

        new_event.header.stamp.sec = diff.total_seconds();
        new_event.header.stamp.nsec = diff.fractional_seconds()*1000;

        //new_event.header.stamp = requested_time;
        new_event.event_description = tree_model->index(i, 0).child(j, 1).data().toString().toStdString();

        event_bag.write("/event/" + category, new_event.header.stamp, new_event);

        std::cout << "events: " << tree_model->index(i, 0).child(j, 0).data().toString().toStdString() << ", "
                  << tree_model->index(i, 0).child(j, 1).data().toString().toStdString() << std::endl;
      }
    }

    event_bag.close();
  }
}

void DatasetGUI::recordEvent() {

  {
    QModelIndexList children;

    //  Get top-level first.
    for (int i = 0; i < tree_model->rowCount(); ++i) {
      children << tree_model->index(i, 0);  //  Use whatever column you are interested in.
    }

    for (auto child: children) {
      std::cout << "root: " << child.data().toString().toStdString() << std::endl;

    }


    /*
    // Now descend through the generations.
    for ( int i = 0; i < children.size(); ++i ) {
      for ( int j = 0; j < tree_model->rowCount( children[i] ); ++j ) {
        children << children[i].child( j, 0 );
      }
    }
     */


    // print each category
    QModelIndex index = ui->treeView->selectionModel()->currentIndex().sibling(ui->treeView->selectionModel()->currentIndex().row(), 0);

    QAbstractItemModel *test_model = ui->treeView->model();

    QModelIndex parent_index = index;
    while (parent_index.parent().isValid())
      parent_index = parent_index.parent();

    std::cout << "selected parent " << parent_index.data().toString().toStdString() << std::endl;


    std::cout << index.data().toString().toStdString() << std::endl;

    for (int row = 0; row < tree_model->rowCount(); ++row) {
      QModelIndex child = tree_model->index(index.row() + 1, 1, index.parent());
      std::cout << child.data().toString().toStdString() << std::endl;
    }


    test_model->insertRow(0, parent_index);
    test_model->setData(tree_model->index(0, 0, parent_index), QVariant(ui->currentEventTimeText->text()), Qt::EditRole);
    test_model->setData(tree_model->index(0, 1, parent_index), QVariant(ui->eventText->text()), Qt::EditRole);

  }

  {
    //  Get top-level first.
  for (int i = 0; i < tree_model->rowCount(); ++i) {
    //  Use whatever column you are interested in.
    std::cout << "category: " << tree_model->index(i, 0).data().toString().toStdString() << std::endl;

    for ( int j = 0; j < tree_model->rowCount( tree_model->index(i, 0) ); ++j ) {
      //children << children[i].child( j, 0 );
      std::cout << "events: " << tree_model->index(i, 0).child(j,0).data().toString().toStdString() << ", " << tree_model->index(i, 0).child(j,1).data().toString().toStdString() << std::endl;
    }
  }


  /*
  // Now descend through the generations.
  for ( int i = 0; i < children.size(); ++i ) {
    for ( int j = 0; j < tree_model->rowCount( children[i] ); ++j ) {
      children << children[i].child( j, 0 );
    }
  }
   */

}

}



void DatasetGUI::addEventToCategory(std::string category, dataset_msgs::DatasetEvent::ConstPtr event) {

  QAbstractItemModel *test_model = ui->treeView->model();

  for (int i = 0; i < tree_model->rowCount(); ++i) {
    auto parent_index = tree_model->index(i, 0);
    if (category == parent_index.data().toString().toStdString()) {
      test_model->insertRow(0, parent_index);
      std::string time_string = boost::posix_time::to_iso_extended_string(event->header.stamp.toBoost());
      test_model->setData(tree_model->index(0, 0, parent_index), QVariant(time_string.c_str()), Qt::EditRole);
      test_model->setData(tree_model->index(0, 1, parent_index), QVariant(event->event_description.c_str()), Qt::EditRole);
    }
  }

}






void DatasetGUI::eventGeneratorPressed() {
  /*
  if (workerThread) {
    ui->preprocessingTextEdit->setText("Running event generator\n");
    setTextTermFormatting(ui->preprocessingTextEdit, this->exec(std::string("rosrun event_generator run.sh " + workerThread->file_name).c_str()).c_str());
    ui->preprocessingTextEdit->append("Event generation process complete"); 
  }
  else {
    ui->preprocessingTextEdit->setText("No bag file has been selected - select a bag before generating the events");
  }
*/

  if (workerThread) {

    ui->preprocessingTextEdit->setText("Generating events for selected bag file based on motion and location\nNote: this can take several minutes. The results will be displayed here when it is completed");

    if (processorThread) {
      std::cout << "cannot start processing, existing process is in progress" << std::endl;  
      return;
    }

    processorThread = new ProcessingThread();
    processorThread->command = std::string("rosrun event_generator run.sh " + workerThread->file_name);
    processorThread->resultsDestination = ui->preprocessingTextEdit;
    connect(processorThread, SIGNAL(resultReady(QString)), this, SLOT(processingCompleted(QString)));
    connect(processorThread, SIGNAL(partialReady(QString)), this, SLOT(processingPartial(QString)));
    processorThread->start();

  }
  else {
    ui->preprocessingTextEdit->setText("No bag file has been selected - select a bag before generating the events");
  }

}


void DatasetGUI::processingPartial(QString results) {
  if (processorThread) {
    if (processorThread->resultsDestination) {
      processorThread->resultsDestination->setText("");
      setTextTermFormatting(processorThread->resultsDestination, results);
    }
  }
}

void DatasetGUI::processingCompleted(QString results) {
  if (processorThread) {
    if (processorThread->resultsDestination) {
      processorThread->resultsDestination->setText("");
      setTextTermFormatting(processorThread->resultsDestination, results);
    }
    ui->preprocessingTextEdit->append("Conversion process complete");

    delete processorThread;
    processorThread = NULL;
  }
  else {
    std::cout << "processor thread not found" << std::endl;
  }
}



void DatasetGUI::angleRightLane(QString results) {
  ui->preprocessingTextEdit->setText("Running conversion between H264 and MP4\n");

  if (processorThread) {
    std::cout << "cannot start processing, existing process is in progress" << std::endl;  
    return;
  }

  processorThread = new ProcessingThread();
  processorThread->command = "rosrun h264_bag_playback convert_folder.sh";
  processorThread->resultsDestination = ui->preprocessingTextEdit;
  connect(processorThread, SIGNAL(resultReady(QString)), this, SLOT(processingCompleted(QString)));
  connect(processorThread, SIGNAL(partialReady(QString)), this, SLOT(processingPartial(QString)));
  processorThread->start();

  //ui->preprocessingTextEdit->append("Conversion process complete");
}



void DatasetGUI::runCommand(QTextEdit *destination, QString command, QString display_info) {

  destination->setText(display_info);

  if (processorThread) {
    std::cout << "cannot start processing, existing process is in progress" << std::endl;  
    return;
  }

  processorThread = new ProcessingThread();
  processorThread->command = command.toStdString();
  processorThread->resultsDestination = destination;
  connect(processorThread, SIGNAL(resultReady(QString)), this, SLOT(processingCompleted(QString)));
  connect(processorThread, SIGNAL(partialReady(QString)), this, SLOT(processingPartial(QString)));
  processorThread->start();
}


void DatasetGUI::bagAnalysePosOnly() {
  if (workerThread) {
    std::ostringstream oss;
    oss << "rosrun bag_analysis bag-analyse.py -bag " << workerThread->file_name << " -pos";
    runCommand(ui->preprocessingTextEdit, oss.str().c_str(), "Running bag analysis - plotting position (please wait, this might take a minute or two)");
  }
  else {
    std::cout << "no file is loaded to analyse" << std::endl;    
  }

}

void DatasetGUI::angleLeftLane(QString results) {
  runCommand(ui->preprocessingTextEdit, "rosrun h264_bag_playback convert_folder.sh", "Running conversion between H264 and MP4");
}


void DatasetGUI::convertPressed() {

  ui->preprocessingTextEdit->setText("Running conversion between H264 and MP4\n");

  if (processorThread) {
    std::cout << "cannot start processing, existing process is in progress" << std::endl;  
    return;
  }

  processorThread = new ProcessingThread();
  processorThread->command = "rosrun h264_bag_playback convert_folder.sh";
  processorThread->resultsDestination = ui->preprocessingTextEdit;
  connect(processorThread, SIGNAL(resultReady(QString)), this, SLOT(processingCompleted(QString)));
  connect(processorThread, SIGNAL(partialReady(QString)), this, SLOT(processingPartial(QString)));
  processorThread->start();

  //ui->preprocessingTextEdit->append("Conversion process complete");
}

void DatasetGUI::sliderPressed(){
    // pause the playback

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }

}



void DatasetGUI::sliderMoved(int new_value) {

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {
      double start_time = workerThread->bag_playback.requested_start_time.toSec();
      double end_time = workerThread->bag_playback.requested_end_time.toSec();
      double requested_time = start_time + ((end_time - start_time) * new_value / 1000.);

      ui->currentTimeText->setText("seek to time: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());
  }
}




void DatasetGUI::OnClickedTree(QModelIndex clicked_item) {

  std::cout << "clicked on the treeview" << std::endl;

  ros::Time requested_time(0);// =  boost::posix_time::to_iso_extended_string(workerThread->bag_playback.last_packet_time.toBoost()).c_str()
  QString requested_time_string;

  try {
  // print each category
  QModelIndex index = clicked_item.sibling(ui->treeView->selectionModel()->currentIndex().row(), 0);
  std::cout << "data :" << index.data().toString().toStdString() << std::endl;

  auto boost_time = boost::posix_time::from_iso_extended_string(index.data().toString().toStdString());



  boost::posix_time::time_duration diff = boost_time - boost::posix_time::from_time_t(0);

  requested_time.sec = diff.total_seconds();
  requested_time.nsec = diff.fractional_seconds()*1000;

  requested_time_string = boost::posix_time::to_iso_extended_string(requested_time.toBoost()).c_str();

  }
  catch(...) {
    // couldn't convert the time
    std::cout << "clicked on the treeview, could not convert the value into a time" << std::endl;
    return;
  }

  if (workerThread) {

    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;

    while (workerThread->current_state != DatasetThread::PLAYBACK_PAUSE) {
      ros::spinOnce();
    }

    ui->currentTimeText->setText("loading time: " + requested_time_string);

    workerThread->seek_time = ros::Time(requested_time);
    workerThread->current_state = DatasetThread::PLAYBACK_SEEK;

    ui->currentTimeText->setText("still seeking: " + requested_time_string);

    while (workerThread->current_state == DatasetThread::PLAYBACK_SEEK) {
      ros::spinOnce();
    }

    //workerThread->bag_playback.limit_playback_speed = !ui->checkPlaybackRealtime->isChecked();

    ui->currentTimeText->setText("finished seek");
  }
}




void DatasetGUI::addDuration(float offset) {
    if (workerThread) {
    workerThread->seek_time = workerThread->bag_playback.last_packet_time + ros::Duration(offset);
    workerThread->current_state = DatasetThread::PLAYBACK_SEEK;

    while (workerThread->current_state == DatasetThread::PLAYBACK_SEEK) {
      ros::spinOnce();
    }

    workerThread->current_state = DatasetThread::PLAYBACK_START;
  }
}



void DatasetGUI::addMinute() {
  addDuration(60.);
}


void DatasetGUI::addSecond() {
  addDuration(1.);
}


void DatasetGUI::subtractMinute() {
  addDuration(-60.);
}

void DatasetGUI::subtractSecond() {
  addDuration(-1.);
}



void DatasetGUI::sliderReleased() {

  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_PAUSE) {

    int requested_value = ui->timeSlider->value();

    double start_time = workerThread->bag_playback.requested_start_time.toSec();
    double end_time = workerThread->bag_playback.requested_end_time.toSec();
    double requested_time = start_time + ((end_time - start_time) * requested_value / 1000.);

    ui->currentTimeText->setText("loading time: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());

    workerThread->seek_time = ros::Time(requested_time);
    workerThread->current_state = DatasetThread::PLAYBACK_SEEK;

    ui->currentTimeText->setText("still seeking: " + QDateTime::fromSecsSinceEpoch(requested_time).toString());

    while (workerThread->current_state == DatasetThread::PLAYBACK_SEEK) {
      ros::spinOnce();
    }

    ui->currentTimeText->setText("finished seek");
  }
}


void DatasetGUI::PollROS() {
  if (workerThread && workerThread->current_state == DatasetThread::PLAYBACK_START) {
    double start_time = workerThread->bag_playback.requested_start_time.toSec();
    double end_time = workerThread->bag_playback.requested_end_time.toSec();
    double current_time = workerThread->bag_playback.last_packet_time.toSec();

    ui->currentEventTimeText->setText(boost::posix_time::to_iso_extended_string(workerThread->bag_playback.last_packet_time.toBoost()).c_str());

    double percentage_through = 0.;
    if (current_time - start_time > 0)
       percentage_through = 1000. * (double(current_time - start_time) / double(end_time - start_time));

    ui->timeSlider->setValue(percentage_through);
    ui->progressBar->setValue(percentage_through/10.);
    ui->currentTimeText->setText(QDateTime::fromSecsSinceEpoch(current_time).toString());
  }
  ros::spinOnce();
}


void DatasetGUI::changePlaybackFast(){
  if (workerThread) {
    workerThread->bag_playback.limit_playback_speed = false; //!ui->checkPlaybackRealtime->isChecked();
    workerThread->bag_playback.scale_playback_speed = 1000.;
//    ROS_INFO_STREAM("is checked " << workerThread->bag_playback.limit_playback_speed);
  }
}


void DatasetGUI::changePlaybackHalf() {
  if (workerThread) {
    workerThread->bag_playback.limit_playback_speed = true; //!ui->checkPlaybackRealtime->isChecked();
    workerThread->bag_playback.scale_playback_speed = .5;
//    ROS_INFO_STREAM("is checked " << workerThread->bag_playback.limit_playback_speed);
  }
}

void DatasetGUI::changePlaybackQuarter() {
  if (workerThread) {
    workerThread->bag_playback.limit_playback_speed = true; //!ui->checkPlaybackRealtime->isChecked();
    workerThread->bag_playback.scale_playback_speed = .25;
//    ROS_INFO_STREAM("is checked " << workerThread->bag_playback.limit_playback_speed);
  }
}

void DatasetGUI::changePlaybackDouble() {
  if (workerThread) {
    workerThread->bag_playback.limit_playback_speed = true; //!ui->checkPlaybackRealtime->isChecked();
    workerThread->bag_playback.scale_playback_speed = 2.;
//    ROS_INFO_STREAM("is checked " << workerThread->bag_playback.limit_playback_speed);
  }
}

void DatasetGUI::changePlaybackRealtime(){
  if (workerThread) {
    workerThread->bag_playback.limit_playback_speed = true; //!ui->checkPlaybackRealtime->isChecked();
    workerThread->bag_playback.scale_playback_speed = 1.;
//    ROS_INFO_STREAM("is checked " << workerThread->bag_playback.limit_playback_speed);
  }
}


void DatasetGUI::startPressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_START;
  }
}

void DatasetGUI::stopPressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_STOP;

    resetGUI();

    std::cout << "exiting playback thread" << std::endl;
    workerThread->quit();
    std::cout << "waiting for finish" << std::endl;
    workerThread->wait();
    std::cout << "deleting playback thread" << std::endl;

    delete workerThread;
    workerThread = NULL;

  }
}

void DatasetGUI::pausePressed() {
  if (workerThread){
    workerThread->current_state = DatasetThread::PLAYBACK_PAUSE;
  }
}


