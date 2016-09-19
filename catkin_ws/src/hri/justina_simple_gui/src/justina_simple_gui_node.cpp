#include <iostream>
#include <QApplication>
#include "MainWindow.h"
#include "QtRosNode.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING JUSTINA GUI BY MARCOSOFT" << std::endl;
    ros::init(argc, argv, "justina_simple_gui");
    ros::NodeHandle n;

    QtRosNode qtRosNode;
    qtRosNode.setNodeHandle(&n);
    qtRosNode.start();

    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.setRosNode(&qtRosNode);

    mainWindow.show();
    return app.exec();
}
