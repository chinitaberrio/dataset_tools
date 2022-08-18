#pragma once

#include <ros/ros.h>

#include <QFileDialog>

#include <QTextEdit>
#include <QLineEdit>

#include <QThread>
#include <QThread>
#include <QSlider>
#include <QMouseEvent>

#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>

#include <h264_bag_playback/h264_bag_playback.hpp>

#include "datasetgui.h"

#include <QTreeView>
#include "treemodel.h"


class ProcessingThread : public QThread
{

public:

  ProcessingThread() : completed(false), resultsDestination(NULL) {}

  Q_OBJECT

signals:
  void resultReady(const QString &s);
    

public:
  void run() {
    result = this->exec(command.c_str());
    completed = true;
    emit resultReady(QString(result.c_str()));
  }

  std::string exec(const char* cmd) {
      std::array<char, 128> buffer;
      std::string result;
      std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
      if (!pipe) {
          throw std::runtime_error("popen() failed!");
      }
      while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
          result += buffer.data();
      }
      return result;
  }

  std::string command;
  bool completed;
  std::string result;
  QTextEdit *resultsDestination;

};


class DatasetThread : public QThread
{


  Q_OBJECT

  void run() {
        QString result;
        /* expensive or blocking operation  */
        ros::NodeHandle("~").setParam("bag_file", file_name);

        bag_playback.init_playback();
        bag_playback.limit_playback_speed = false;

        bag_playback.OpenBags();

        current_state = PLAYBACK_PAUSE;

        do
        {
            while(current_state == PLAYBACK_PAUSE) {
            // Spin once so that any other ros controls/pub/sub can be actioned
              ros::Duration(0.2).sleep();
              ros::spinOnce();
            }

            if (!ros::ok() || current_state == PLAYBACK_STOP) {
              break;
            }

            if (current_state == PLAYBACK_SEEK) {
              /*if (seek_time < bag_playback.last_packet_time) {
                // need to reload the data as the iterator only goes one way
                bag_playback.init_playback();

                for (auto video: bag_playback.videos) {
                  video.second.frame_counter = 0;
                }

                bag_playback.OpenBags();
              }
               */
              bag_playback.SeekTime(seek_time);
              current_state = PLAYBACK_START;
            }

            if (!bag_playback.ReadNextPacket()) {
              current_state = PLAYBACK_PAUSE;
            }

        } while (true);

        bag_playback.CloseBags();

        emit resultReady(result);
    }
signals:
    void resultReady(const QString &s);

public:
    std::string file_name;

    ros::Time seek_time;

    typedef enum PLAYBACK_CONTROL {
      PLAYBACK_LOADING,
      PLAYBACK_STOP,
      PLAYBACK_START,
      PLAYBACK_PAUSE,
      PLAYBACK_SEEK
    } PLAYBACK_CONTROL;

    PLAYBACK_CONTROL current_state;

    dataset_toolkit::h264_bag_playback bag_playback;

    DatasetThread() {current_state = PLAYBACK_LOADING;}

};

