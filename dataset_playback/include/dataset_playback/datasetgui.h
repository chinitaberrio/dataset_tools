/********************************************************************************
** Form generated from reading UI file 'datasetgui.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef DATASETGUI_H
#define DATASETGUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DatasetGUI
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QTabWidget *tabWidget;
    QWidget *dataset_playback;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_3;
    QPushButton *startButton;
    QPushButton *pauseButton;
    QSpacerItem *verticalSpacer;
    QPushButton *selectButton;
    QPushButton *stopButton;
    QCheckBox *checkPlaybackRealtime;
    QGroupBox *statusGroupBox;
    QVBoxLayout *verticalLayout_7;
    QTextEdit *statusText;
    QLineEdit *currentTimeText;
    QGroupBox *eventGroupBox;
    QVBoxLayout *verticalLayout_5;
    QTreeView *treeView;
    QFormLayout *formLayout;
    QPushButton *recordEventButton;
    QLineEdit *eventText;
    QLineEdit *currentEventTimeText;
    QPushButton *saveEventsButton;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *timeSelectionGroupBox;
    QVBoxLayout *verticalLayout_6;
    QSlider *timeSlider;
    QProgressBar *progressBar;
    QWidget *dataset_processing;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *DatasetGUI)
    {
        if (DatasetGUI->objectName().isEmpty())
            DatasetGUI->setObjectName(QStringLiteral("DatasetGUI"));
        DatasetGUI->resize(1052, 669);
        centralwidget = new QWidget(DatasetGUI);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        QFont font;
        font.setPointSize(16);
        tabWidget->setFont(font);
        dataset_playback = new QWidget();
        dataset_playback->setObjectName(QStringLiteral("dataset_playback"));
        verticalLayout_4 = new QVBoxLayout(dataset_playback);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        startButton = new QPushButton(dataset_playback);
        startButton->setObjectName(QStringLiteral("startButton"));
        startButton->setMinimumSize(QSize(0, 80));

        verticalLayout_3->addWidget(startButton);

        pauseButton = new QPushButton(dataset_playback);
        pauseButton->setObjectName(QStringLiteral("pauseButton"));
        pauseButton->setMinimumSize(QSize(0, 80));

        verticalLayout_3->addWidget(pauseButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        selectButton = new QPushButton(dataset_playback);
        selectButton->setObjectName(QStringLiteral("selectButton"));
        selectButton->setMinimumSize(QSize(0, 50));

        verticalLayout_3->addWidget(selectButton);

        stopButton = new QPushButton(dataset_playback);
        stopButton->setObjectName(QStringLiteral("stopButton"));
        stopButton->setMinimumSize(QSize(0, 50));

        verticalLayout_3->addWidget(stopButton);

        checkPlaybackRealtime = new QCheckBox(dataset_playback);
        checkPlaybackRealtime->setObjectName(QStringLiteral("checkPlaybackRealtime"));
        QFont font1;
        font1.setPointSize(12);
        checkPlaybackRealtime->setFont(font1);

        verticalLayout_3->addWidget(checkPlaybackRealtime);


        horizontalLayout_2->addLayout(verticalLayout_3);

        statusGroupBox = new QGroupBox(dataset_playback);
        statusGroupBox->setObjectName(QStringLiteral("statusGroupBox"));
        statusGroupBox->setFont(font1);
        verticalLayout_7 = new QVBoxLayout(statusGroupBox);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        statusText = new QTextEdit(statusGroupBox);
        statusText->setObjectName(QStringLiteral("statusText"));

        verticalLayout_7->addWidget(statusText);

        currentTimeText = new QLineEdit(statusGroupBox);
        currentTimeText->setObjectName(QStringLiteral("currentTimeText"));

        verticalLayout_7->addWidget(currentTimeText);


        horizontalLayout_2->addWidget(statusGroupBox);

        eventGroupBox = new QGroupBox(dataset_playback);
        eventGroupBox->setObjectName(QStringLiteral("eventGroupBox"));
        eventGroupBox->setFont(font1);
        verticalLayout_5 = new QVBoxLayout(eventGroupBox);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        treeView = new QTreeView(eventGroupBox);
        treeView->setObjectName(QStringLiteral("treeView"));

        verticalLayout_5->addWidget(treeView);

        formLayout = new QFormLayout();
        formLayout->setObjectName(QStringLiteral("formLayout"));
        recordEventButton = new QPushButton(eventGroupBox);
        recordEventButton->setObjectName(QStringLiteral("recordEventButton"));
        recordEventButton->setFont(font1);

        formLayout->setWidget(0, QFormLayout::FieldRole, recordEventButton);

        eventText = new QLineEdit(eventGroupBox);
        eventText->setObjectName(QStringLiteral("eventText"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(eventText->sizePolicy().hasHeightForWidth());
        eventText->setSizePolicy(sizePolicy);

        formLayout->setWidget(0, QFormLayout::LabelRole, eventText);

        currentEventTimeText = new QLineEdit(eventGroupBox);
        currentEventTimeText->setObjectName(QStringLiteral("currentEventTimeText"));
        sizePolicy.setHeightForWidth(currentEventTimeText->sizePolicy().hasHeightForWidth());
        currentEventTimeText->setSizePolicy(sizePolicy);

        formLayout->setWidget(1, QFormLayout::LabelRole, currentEventTimeText);

        saveEventsButton = new QPushButton(eventGroupBox);
        saveEventsButton->setObjectName(QStringLiteral("saveEventsButton"));
        saveEventsButton->setFont(font1);

        formLayout->setWidget(1, QFormLayout::FieldRole, saveEventsButton);


        verticalLayout_5->addLayout(formLayout);


        horizontalLayout_2->addWidget(eventGroupBox);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        timeSelectionGroupBox = new QGroupBox(dataset_playback);
        timeSelectionGroupBox->setObjectName(QStringLiteral("timeSelectionGroupBox"));
        timeSelectionGroupBox->setFont(font1);
        verticalLayout_6 = new QVBoxLayout(timeSelectionGroupBox);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        timeSlider = new QSlider(timeSelectionGroupBox);
        timeSlider->setObjectName(QStringLiteral("timeSlider"));
        timeSlider->setOrientation(Qt::Horizontal);

        verticalLayout_6->addWidget(timeSlider);

        progressBar = new QProgressBar(timeSelectionGroupBox);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setValue(24);

        verticalLayout_6->addWidget(progressBar);


        verticalLayout_2->addWidget(timeSelectionGroupBox);


        verticalLayout->addLayout(verticalLayout_2);


        verticalLayout_4->addLayout(verticalLayout);

        tabWidget->addTab(dataset_playback, QString());
        dataset_processing = new QWidget();
        dataset_processing->setObjectName(QStringLiteral("dataset_processing"));
        tabWidget->addTab(dataset_processing, QString());

        horizontalLayout->addWidget(tabWidget);

        DatasetGUI->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(DatasetGUI);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        DatasetGUI->setStatusBar(statusbar);

        retranslateUi(DatasetGUI);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(DatasetGUI);
    } // setupUi

    void retranslateUi(QMainWindow *DatasetGUI)
    {
        DatasetGUI->setWindowTitle(QApplication::translate("DatasetGUI", "MainWindow", Q_NULLPTR));
        startButton->setText(QApplication::translate("DatasetGUI", "Start", Q_NULLPTR));
        pauseButton->setText(QApplication::translate("DatasetGUI", "Pause", Q_NULLPTR));
        selectButton->setText(QApplication::translate("DatasetGUI", "Select", Q_NULLPTR));
        stopButton->setText(QApplication::translate("DatasetGUI", "Stop", Q_NULLPTR));
        checkPlaybackRealtime->setText(QApplication::translate("DatasetGUI", "Real-time", Q_NULLPTR));
        statusGroupBox->setTitle(QApplication::translate("DatasetGUI", "Status", Q_NULLPTR));
        eventGroupBox->setTitle(QApplication::translate("DatasetGUI", "Events", Q_NULLPTR));
        recordEventButton->setText(QApplication::translate("DatasetGUI", "Mark Event", Q_NULLPTR));
        saveEventsButton->setText(QApplication::translate("DatasetGUI", "Events to File", Q_NULLPTR));
        timeSelectionGroupBox->setTitle(QApplication::translate("DatasetGUI", "Change playback time", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(dataset_playback), QApplication::translate("DatasetGUI", "Playback", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(dataset_processing), QApplication::translate("DatasetGUI", "Processing", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class DatasetGUI: public Ui_DatasetGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // DATASETGUI_H
