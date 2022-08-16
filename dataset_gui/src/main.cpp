#include "datasetgui.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "DatasetGUI");

    QApplication a(argc, argv);
    DatasetGUI w;
    w.InitialiseWindow();

    w.show();

    return a.exec();
}
