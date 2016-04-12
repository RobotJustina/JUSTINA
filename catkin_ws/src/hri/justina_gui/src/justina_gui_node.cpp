#include <iostream>
#include "MainWindow.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING JUSTINA GUI BY MARCOSOFT" << std::endl;
    ros::init(argc, argv, "justina_gui");

    QtRosNode qtRosNode;
    qtRosNode.start();
    
    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.setWindowTitle(QString::fromUtf8("JUSTINA GUI BY MARCOSOFT"));
    mainWindow.resize(640, 480);
    mainWindow.setRosNode(&qtRosNode);
    
    mainWindow.show();
    return app.exec();
}
