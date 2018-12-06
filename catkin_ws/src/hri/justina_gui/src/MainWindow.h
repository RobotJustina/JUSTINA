#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaRepresentation.h"
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
    bool hriFollowing;
    bool hriFindingLegs;
    bool navDetectingObstacles;
    bool initKnownLoacations;
    bool defInitKnownLoacations;
    bool updateKnownLoacations;
    bool enableInteractiveEdit;
    bool faceRecognition;

    std::map<std::string, std::vector<std::string> > locations;
    std::map<std::string, std::vector<std::string> > objects;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);
    void setPathKnownLoc(const std::string pathKnownLoc);

private:
    bool strToFloatArray(std::string str, std::vector<float>& result);

public slots:
    //Slots for signals emitted in this window (e.g.: pressing buttons)
    void stopRobot();
    //Navigation
    void navBtnCalcPath_pressed();
    void navBtnExecPath_pressed();
    void navMoveChanged();
    void navObsDetectionEnableClicked();
    //Hardware
    void hdPanTiltChanged(double d);
    void laAnglesChanged(double);
    void raAnglesChanged(double);
    void laValuesChanged();
    void raValuesChanged();
    void laOpenGripperChanged(double d);
    void raOpenGripperChanged(double d);
    void laCloseGripperChanged(double d);
    void raCloseGripperChanged(double d);
    void laRadioButtonClicked();
    void raRadioButtonClicked();
    void torsoPoseChanged(double d);
    void torsoLocChanged();
    //Speech synthesis and recog
    void spgSayChanged();
    void sprFakeRecognizedChanged();
    //Vision
    void recSaveVideoChanged();
    void recSaveImageChanged();
    void sktBtnStartClicked();
    void facBtnStartClicked();
    void facRecogPressed();
    void facTrainPressed();
    void facClearPressed();
    void objRecogObjectChanged();
    void vsnFindLinesClicked();
    //HRI
    void hriBtnFollowClicked();
    void hriBtnLegsClicked();
    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();
    //Slots for signad to tab knowledge
    void on_enInteractiveEdit_clicked();
    void on_removeLoc_clicked();
    void on_locTableWidget_itemSelectionChanged();
    void on_addLoc_clicked();
    void on_GetRobotPose_clicked();
    void on_loadFromFile_clicked();
    void on_SaveInFile_clicked();
    void quesReqChanged();
    //slots for knowledge representation
    void enterCommandChanged();
    void loadCommandChanged();
    void on_runCLIPS_clicked();
    void setPathKR();
    void setlocClips();

private slots:


    void on_resetCLIPS_clicked();

    void on_factsCLIPS_clicked();

    void on_rulesCLIPS_clicked();

    void on_agendaCLIPS_clicked();

    void on_openFileCommand_clicked();

    void on_addCLIPSloc_clicked();

    void on_addCLIPSobj_clicked();

    void on_locCLIPStab_itemSelectionChanged();

    void on_objCLIPStab_itemSelectionChanged();

    void on_rotateButton_clicked();

    void on_trainObjButton_clicked();

    void on_pushButtonDownTorso_clicked();

    void on_pushButtonUpTorso_clicked();

private:
    Ui::MainWindow *ui;
    std::string pathKnownLoc;

    enum Column{
        NAME, X, Y, A, C1, C2, C3, C4
    };
};

#endif // MAINWINDOW_H
