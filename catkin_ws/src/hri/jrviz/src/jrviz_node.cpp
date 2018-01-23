#include <QApplication>
#include <ros/ros.h>
#include "mainwindow.h"
#include "QtRosNode.h"

int main(int argc, char **argv){
  ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
  ros::NodeHandle n;

  QApplication app( argc, argv );

  QtRosNode qtRosNode;
  qtRosNode.setNodeHandle(&n);
  qtRosNode.start();

  MainWindow* window = new MainWindow();
  window->setRosNode(&qtRosNode);
  window->show();

  app.exec();

  delete window;
}
