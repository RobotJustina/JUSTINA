#include <GL/glew.h>
#include <QApplication>
#include <QtOpenGL/QGLFormat>
#include "QtRosNode.h"

#include "mainwindow.h"

int main( int argc, char* argv[] )
{
    QApplication a( argc, argv );

    ros::init(argc, argv, "graphics_viz_node");
    ros::NodeHandle n;

    Q_INIT_RESOURCE(resource);

    QtRosNode qtRosNode;
    qtRosNode.setNodeHandle(&n);
    qtRosNode.start();

    MainWindow w;
    w.setQtRosNode(&qtRosNode);
    w.show();

    return a.exec();
}
