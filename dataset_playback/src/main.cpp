
#include "bus_stop_panel.h"

#include <QtGui>
#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>



int main(int argv, char **args)
{
  ros::init(argv, args, "QtBusStopPanel");

  QApplication app(argv, args);
  
  BusStopPanel *textEdit = new BusStopPanel;
  
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(textEdit);
  
  QWidget window;
  window.setLayout(layout);
  
  window.show();
  
  return app.exec();
}




