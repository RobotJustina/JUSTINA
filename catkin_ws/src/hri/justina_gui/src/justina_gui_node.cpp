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
    /*
    QWidget window;
    window.setFixedSize(200, 200);
    QPushButton* button = new QPushButton("Hello world!", &window);
    button->setText("MyText");
    button->setToolTip("The tooltip");
    QFont font("Courier");
    button->setFont(font);
    button->setIcon(QIcon::fromTheme("face-smile"));
    button->setGeometry(100, 100, 80, 30);
    //button.show();
    window.show();
    return app.exec();
    */
}
