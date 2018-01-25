#include <QApplication>
#include <ros/ros.h>
#include "mainwindow.h"
#include "QtRosNode.h"

int main(int argc, char **argv){
  ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
  ros::NodeHandle n;

  std::string locationsPath = "";
  for (int i = 0; i < argc; i++) {
    std::string strParam(argv[i]);
    if (strParam.compare("-p") == 0)
      locationsPath = argv[++i];
  }

  std::string configFile = "";
  std::cout << "--------------------------------------------------------------------------------configFile:" << configFile << std::endl;
  for (int i = 0; i < argc; i++) {
    std::string strParam(argv[i]);
    if (strParam.compare("-d") == 0)
      configFile = argv[++i];
  }

  QApplication app( argc, argv );

  QtRosNode qtRosNode;
  qtRosNode.setNodeHandle(&n);
  qtRosNode.start();

  MainWindow* window = new MainWindow(configFile);
  window->setRosNode(&qtRosNode);
  window->setPathKnownLoc(locationsPath);
  window->show();

  app.exec();

  delete window;
}
