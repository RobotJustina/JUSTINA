#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"
#include "QtRosNode.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

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
    //void hdBtnTiltUp_pressed();
    //void hdBtnTiltDown_pressed();
    //void hdPanTiltChanged();
    //void laAnglesChanged();
    //void raAnglesChanged();
    //void spgSayChanged();
    //void sprRecognizedChanged();

    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    //void updateGraphicsReceived();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
