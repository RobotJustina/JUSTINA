#include <iostream>
#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QTabWidget>
#include <QLineEdit>
#include <QLabel>
#include "ros/ros.h"

class MainWindow : public QWidget
{
Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
//    virtual ~MainWindow(){};
    
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
    
signals:

public slots:
    void navBtnCalcPath_pressed();
};
