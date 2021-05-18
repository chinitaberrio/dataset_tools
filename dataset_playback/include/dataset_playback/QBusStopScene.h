#ifndef QBUSSTOPSCENE
#define QBUSSTOPSCENE

#include <ros/ros.h>

#include <rviz/panel.h>

#include <QMouseEvent>
#include <QSvgWidget>
#include <QSvgRenderer>

#include <QListWidget>
#include <QPushButton>

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>




class TextRectangle : public QGraphicsRectItem
{

public:
    TextRectangle(QGraphicsItem *parent = 0) : QGraphicsRectItem(parent) {}
    TextRectangle(double x, double y, double w, double h, QGraphicsItem *parent = 0) : QGraphicsRectItem(x,y,w,h,parent) {}

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0)
    {
        painter->setPen(Qt::white);
        if (isSelected()) {
            painter->setBrush(QBrush(QColor(60,160,60)));
        }
        else {
            painter->setBrush(QBrush(QColor(60,60,60)));
        }
        painter->drawEllipse(this->boundingRect());
        painter->setPen(Qt::white);
        painter->drawText(this->boundingRect(), /*Qt::AlignVCenter || Qt::AlignCenter*/ Qt::AlignCenter, item_text);
    }

    QString item_text;


};



class QBusStopView : public QGraphicsView {

public:
    QRectF bounding_rect;

    void ViewGraph(std::vector<QString> items,
        std::vector<std::pair<std::string, std::string>> connections);

protected:
    void resizeEvent(QResizeEvent *event) override;
};




class QBusStopScene : public QGraphicsScene {
Q_OBJECT

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);

signals:
    void NewBusStopSelection(QString stop_name);

};


#endif
