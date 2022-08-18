#ifndef DATASETGUI_H
#define DATASETGUI_H

#include <QMainWindow>
#include "treemodel.h"

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/filesystem.hpp>

#include <QFileInfo>
#include <QThread>

#include "dataset_thread.h"
#include <h264_bag_playback/h264_bag_playback.hpp>

#include <rosgraph_msgs/Log.h>

class DatasetThread;
class ProcessingThread;

namespace Ui {
class DatasetGUI;
}

class DatasetGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit DatasetGUI(QWidget *parent = 0);
    ~DatasetGUI();

    void InitialiseWindow();

    DatasetThread *workerThread;
    ProcessingThread *processorThread;

protected Q_SLOTS:

    // slot for when the start button is pressed
    void startPressed();
    void stopPressed();
    void pausePressed();
    void convertPressed();
    void eventGeneratorPressed();

    void sliderPressed();
    void sliderMoved(int new_value);
    void sliderReleased();

    void addMinute();
    void addSecond();
    void subtractMinute();
    void subtractSecond();

    void changePlaybackFast();
    void changePlaybackDouble();
    void changePlaybackRealtime();
    void changePlaybackHalf();
    void changePlaybackQuarter();

    void OnClickedTree(QModelIndex clicked_item);

    void recordEvent();
    void saveEventsToFile();

    void PollROS();

    void selectBagFile();

    void bagAnalysePosOnly();

    void angleLeftLane(QString results);
    void angleRightLane(QString results);

    void panorama3Cameras();
    void panorama5Cameras();

    void processingCompleted(QString results);


public:

    void newLogMessage(rosgraph_msgs::Log::ConstPtr new_message);

    void runCommand(QTextEdit *destination, QString command, QString display_info);

    void addEventToCategory(std::string category, dataset_msgs::DatasetEvent::ConstPtr event);

    void addDuration(float offset);
    void resetGUI();

//    std::string exec(const char* cmd);

    TreeModel* tree_model;


private:
    Ui::DatasetGUI *ui;

    ros::Subscriber log_subscriber_;
};

#endif // DATASETGUI_H
