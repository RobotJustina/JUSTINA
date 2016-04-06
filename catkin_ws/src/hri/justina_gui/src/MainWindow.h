#pragma once
#include <iostream>
#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QTabWidget>
#include <QLineEdit>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QCloseEvent>
#include <QGroupBox>
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
    QWidget* tabGeneral;
    QWidget* tabPlanning;
    QWidget* tabManipulation;
    /*
      Widgets in tabGeneral
    */
    //Widgets for navigation
    QLineEdit* navTxtGoalPose;
    QLineEdit* navTxtStartPose;
    QPushButton* navBtnCalcPath;
    QPushButton* navBtnExecPath;
    QLabel* navLblGoalPose;
    QLabel* navLblStartPose;
    QLabel* navLblRobotPose;
    //Widgets for head
    QLineEdit* hdTxtPan;
    QLineEdit* hdTxtTilt;
    QPushButton* hdBtnPanLeft;
    QPushButton* hdBtnPanRight;
    QPushButton* hdBtnTiltUp;
    QPushButton* hdBtnTiltDown;
    QLabel* hdLblTilt;
    QLabel* hdLblPan;
    QLabel* hdLblHeadPose;
    
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
    void hdPanTiltChanged();

    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void currentPoseReceived(float currentX, float currentY, float currentTheta);
};
