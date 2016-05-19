#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"
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
    bool laIgnoreValueChanged;
    bool raIgnoreValueChanged;
    int laLastRadioButton;
    int raLastRadioButton;
    nav_msgs::Path calculatedPath;
    bool recSavingVideo;
    bool sktRecognizing;
    bool facRecognizing;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);

signals:

public slots:
    //Slots for signals emitted in this window (e.g.: pressing buttons)
    void stopRobot();
    void navBtnCalcPath_pressed();
    void navBtnExecPath_pressed();
    void hdPanTiltChanged(double d);
    void laAnglesChanged(double);
    void raAnglesChanged(double);
    void laRadioButtonClicked();
    void raRadioButtonClicked();
    void spgSayChanged();
    void sprFakeRecognizedChanged();
    void recSaveVideoChanged();
    void recSaveImageChanged();
    void sktBtnStartClicked();
    void facBtnStartClicked();

    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
