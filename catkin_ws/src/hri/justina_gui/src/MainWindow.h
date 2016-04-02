#pragma once
#include <iostream>
#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QTabWidget>
#include <QLineEdit>
#include <QLabel>
#include <QCloseEvent>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "QtRosNode.h"

class MainWindow : public QWidget
{
Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    
    QTabWidget* tabWidget;
    QWidget* tabPlanning;
    QWidget* tabNavigation;
    QWidget* tabManipulation;
    QLineEdit* navTxtGoalPose;
    QLineEdit* navTxtStartPose;
    QPushButton* navBtnCalcPath;
    QPushButton* navBtnExecPath;
    QLabel* navLblGoalPose;
    QLabel* navLblStartPose;
    QLabel* navLblRobotPose;

    QtRosNode* qtRosNode;
    float robotX;
    float robotY;
    float robotTheta;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);
    
signals:

public slots:
    //Slots for signals emitted in this window (e.g.: pressing buttons)
    void navBtnCalcPath_pressed();

    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void currentPoseReceived(float currentX, float currentY, float currentTheta);
};
