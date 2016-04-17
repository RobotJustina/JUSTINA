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
#include "justina_tools/JustinaHardware.h"
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
    QLabel* navLblStatus;
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
    QLabel* hdLblStatus;
    //Widgets for arms
    QLabel* laLabel;
    QLabel* raLabel;
    std::vector<QLabel*> laLblAngles;
    std::vector<QLabel*> raLblAngles;
    std::vector<QLineEdit*> laTxtAngles;
    std::vector<QLineEdit*> raTxtAngles;
    QLabel* laLblStatus;
    QLabel* raLblStatus;
    //Widgets for speech synthesis
    QLabel* spgLabel;
    QLineEdit* spgTxtSay;
    QPushButton* spgBtnSay;
    //Widgets for speech recog
    QLabel* sprLabel;
    QLineEdit* sprTxtRecognized;
    QPushButton* sprBtnRecognized;
        
    QtRosNode* qtRosNode;
    float robotX;
    float robotY;
    float robotTheta;
    float headPan;
    float headTilt;
    std::vector<float> leftArmPoses;
    std::vector<float> leftArmTorques;
    std::vector<float> rightArmPoses;
    std::vector<float> rightArmTorques;
    nav_msgs::Path calculatedPath;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);
    
signals:

public slots:
    //Slots for signals emitted in this window (e.g.: pressing buttons)
    void navBtnCalcPath_pressed();
    void navBtnExecPath_pressed();
    void hdBtnPanLeft_pressed();
    void hdBtnPanRight_pressed();
    void hdBtnTiltUp_pressed();
    void hdBtnTiltDown_pressed();
    void hdPanTiltChanged();
    void laAnglesChanged();
    void raAnglesChanged();
    void spgSayChanged();
    void sprRecognizedChanged();

    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void currentRobotPoseReceived(float currentX, float currentY, float currentTheta);
    void robotGoalPoseReached(bool success);
    void currentHeadPoseReceived(float pan, float tilt);
    void headGoalPoseReached(bool success);
    void currentLeftArmPoseReceived(std::vector<float> angles);
    void leftArmGoalPoseReached(bool success);
    void currentRightArmPoseReceived(std::vector<float> angles);
    void rightArmGoalPoseReached(bool success);
    void navigGoalReachedReceived(bool success);
};
