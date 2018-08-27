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
  for (int i = 0; i < argc; i++) {
    std::string strParam(argv[i]);
    if (strParam.compare("-d") == 0)
      configFile = argv[++i];
  }

  std::string configFileViz = "";
  for (int i = 0; i < argc; i++) {
    std::string strParam(argv[i]);
    if (strParam.compare("-dv") == 0)
      configFileViz = argv[++i];
  }

  std::cout << "FileViz: ---------------------------- " << configFileViz << std::endl;

  QApplication app( argc, argv );

  QtRosNode qtRosNode;
  qtRosNode.setNodeHandle(&n);
  qtRosNode.start();

  MainWindow* window = new MainWindow(configFile, configFileViz);
  window->setRosNode(&qtRosNode);
  window->setPathKnownLoc(locationsPath);
  window->show();
  //window->showFullScreen();

  app.exec();

  delete window;
}
