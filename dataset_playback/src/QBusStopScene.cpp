#include "QBusStopScene.h"

#include <QDomDocument>
#include <QDomNodeList>

#include <QGraphicsPathItem>
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

#include <graphviz/gvc.h>

#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h> /* For O_* constants */


// This has to be outside the scope of a QObject as it uses a function with the same name (close)
QString MakeGraph(double aspect_ratio, std::vector<QString> items,
    std::vector<std::pair<std::string, std::string>> connections);


void QBusStopView::resizeEvent(QResizeEvent *event)
{
    fitInView(bounding_rect, Qt::KeepAspectRatio);
    QGraphicsView::resizeEvent(event);
}


void QBusStopView::ViewGraph(std::vector<QString> items,
    std::vector<std::pair<std::string, std::string>> connections)
{
    // clear all of the current elements
    scene()->clear();
    viewport()->update();

    QDomDocument doc;

    // find the aspect ratio of the display
    double aspect_ratio = (float)viewport()->height() / (float)viewport()->width();

    QString svg_graph = MakeGraph(aspect_ratio, items, connections);
    if (doc.setContent(svg_graph)) {

        QDomNodeList svg_elements = doc.elementsByTagName("g");
        for (int i = 0; i < svg_elements.size(); i++) {

            QDomElement ellipse = svg_elements.item(i).firstChildElement("ellipse");
            if (!ellipse.isNull()) {
                QString node_text = "unknown";
                QDomElement text = svg_elements.item(i).firstChildElement("text");
                if (!text.isNull()) {
                    node_text = text.toElement().text();
                }

                TextRectangle *new_rect = new TextRectangle(ellipse.attribute("cx").toDouble() - ellipse.attribute("rx").toDouble(),
                                                                    ellipse.attribute("cy").toDouble() - ellipse.attribute("ry").toDouble(),
                                                                    ellipse.attribute("rx").toDouble() * 2.,
                                                                    ellipse.attribute("ry").toDouble() * 2.);
                new_rect->item_text = node_text;
                new_rect->setFlag(QGraphicsItem::ItemIsSelectable);
                new_rect->setSelected(false);
                scene()->addItem(new_rect);
            }

            QDomElement path = svg_elements.item(i).firstChildElement("path");
            if (!path.isNull()) {
                QString path_definition = path.attribute("d");
                // make some assumptions here - there is only a start point (M x,y) and a cubic to the destination (C x1, y1, x2, y2, x3, y3)
                QString start_string = path_definition.left(path_definition.lastIndexOf("C"));
                QString cubic_string = path_definition.right(path_definition.length() - path_definition.lastIndexOf("C"));

                if (start_string.startsWith("M") && cubic_string.startsWith("C"))  {
                    // valid curve
                    start_string.remove(0, 1);
                    cubic_string.remove(0, 1);

                    QStringList start_coordinates = start_string.split(",");

                    QPainterPath path;

                    path.moveTo(start_string.split(",").at(0).toFloat(), start_string.split(",").at(1).toFloat());

                    QStringList curve_coordinates = cubic_string.split(" ");
                    QStringList p1 = curve_coordinates.at(0).split(",");
                    QStringList p2 = curve_coordinates.at(1).split(",");
                    QStringList p3 = curve_coordinates.at(2).split(",");

                    path.cubicTo((p1.at(0)).toFloat(), (p1.at(1)).toFloat(),
                                 (p2.at(0)).toFloat(), (p2.at(1)).toFloat(),
                                 (p3.at(0)).toFloat(), (p3.at(1)).toFloat());


                    QGraphicsPathItem *new_item = new QGraphicsPathItem(path);
                    new_item->setPen(QPen(QColor(Qt::black), 3));
                    new_item->setZValue(-1);
                    scene()->addItem(new_item);
                }
            }

            QDomElement polygon = svg_elements.item(i).firstChildElement("polygon");
            if (!polygon.isNull()) {
                QString fill_colour = polygon.attribute("fill");

                QString polygon_string = polygon.attribute("points");
                QStringList polygon_points = polygon_string.split(" ");

                QPolygonF polygon;

                for (auto &polygon_point: polygon_points) {
                    QStringList coords = polygon_point.split(",");
                    if (coords.size() > 1) {
                        polygon << QPointF(coords.front().toFloat(), coords.back().toFloat());
                    }
                }

                QGraphicsPolygonItem *new_item = new QGraphicsPolygonItem(polygon);
                if (fill_colour.toLower() == "white" || fill_colour.toLower() == "#ffffff") {
                    bounding_rect = polygon.boundingRect();
                    new_item->setBrush(QBrush(QColor(Qt::white)));
                    new_item->setPen(QPen(QColor(Qt::white)));
                    new_item->setZValue(-1);

                }
                else{
                    new_item->setBrush(QBrush(QColor(Qt::black)));
                    new_item->setZValue(1);
                }

                scene()->addItem(new_item);
            }
        }
    }

    fitInView(bounding_rect, Qt::KeepAspectRatio);
    centerOn(bounding_rect.center());
}


QString
MakeGraph(double aspect_ratio, std::vector<QString> items,
    std::vector<std::pair<std::string, std::string>> connections)
{
    std::ostringstream aspect_ratio_string;
    aspect_ratio_string << "-Gratio=" << aspect_ratio;

    // Simulate the command line parameters (for the input to the graph library)
    char* args[] = {const_cast<char*>("dot"),
                    const_cast<char*>("-Gsize=1,8\\!"),
                    const_cast<char*>("-Grankdir=LR"),
                    const_cast<char*>("-Goverlap=false"),
                    const_cast<char*>("-Gminlen=0.1"),
                    const_cast<char*>(aspect_ratio_string.str().c_str())
    };

    const int argc = sizeof(args)/sizeof(args[0]);

    // store the pointers to the nodes and edges
    std::vector<Agnode_t*> nodes;
    std::vector<Agedge_t*> edges;

    // Pointer to the graph object
    Agraph_t *g;

    // set up a graphviz context
    GVC_t *gvc = gvContext();

    // parse command line args
    gvParseArgs(gvc, argc, args);

    // Create a simple digraph
    const char *context_name = "g";
    g = agopen((char*)context_name, Agdirected, NIL(Agdisc_t *));

    std::map<std::string, unsigned int> mapping;

    for (auto &item: items) {
      mapping[item.toStdString()] = nodes.size();
      nodes.push_back(agnode(g, (char*)(item.toStdString().c_str()), 1));
    }

    for (auto connection: connections) {
      edges.push_back(agedge(g, nodes[mapping[connection.first]], nodes[mapping[connection.second]], 0, 1));
    }

    // Compute a layout using a layout engine
    gvLayout(gvc, g, "neato");

    QString return_string;

    size_t size = 1e5;
    char *ptr = 0;

    FILE *out_file = open_memstream(&ptr, &size);
    if (out_file == NULL)
        std::cout << "error opening open_memstream" << std::endl;

    char stdout_buffer[32765] = {0};
    int out_pipe[2];
    int saved_stdout;

    fflush(stdout);

    // save stdout for display later
    saved_stdout = dup(STDOUT_FILENO);

    // make a pipe
    if( pipe(out_pipe) != 0 ) {
      exit(1);
    }

    // redirect stdout to the pipe
    dup2(out_pipe[1], STDOUT_FILENO);
    close(out_pipe[1]);

    // render the graph, output the graph to the out_file
    //  NOTE: this does not seem to give the rendered output, only the dot file
    gvRender(gvc, g, "svg", out_file);
    fflush(stdout);

    // read from stdout pipe into buffer
    read(out_pipe[0], stdout_buffer, 32764);

    // reconnect stdout pipe
    dup2(saved_stdout, STDOUT_FILENO);

    fclose(out_file);
    free(ptr);

    return_string = QString::fromUtf8(stdout_buffer);

    // Free components
    gvFreeLayout(gvc, g);
    agclose(g);
    gvFreeContext(gvc);

    return return_string;
}


void QBusStopScene::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    if (event->button() == Qt::LeftButton)
    {
        QGraphicsItem *item = itemAt(event->scenePos(), QTransform());// it is your clicked item, you can do everything what you want. for example send it somewhere
        TextRectangle *ell = qgraphicsitem_cast<TextRectangle*>(item);
        if(ell) {
            ell->setSelected(true);
            // unselect all other
            for (auto &graphics_item: items()) {
                TextRectangle *test_rectangle = qgraphicsitem_cast<TextRectangle*>(graphics_item);
                if (test_rectangle && test_rectangle != ell) {
                    test_rectangle->setSelected(false);
                }
            }
          emit NewBusStopSelection(ell->item_text);
        }
        else {
        }
    }
}



