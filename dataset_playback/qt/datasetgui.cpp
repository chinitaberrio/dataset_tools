#include "datasetgui.h"
#include "ui_datasetgui.h"

DatasetGUI::DatasetGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::DatasetGUI)
{
    ui->setupUi(this);
}

DatasetGUI::~DatasetGUI()
{
    delete ui;
}
