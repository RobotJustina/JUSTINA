#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>
#include <QActionGroup>

#include "QtRosNode.h"

#include "rviz/yaml_config_reader.h"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::string configFile, std::string configFileViz, QWidget *parent = 0);
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
    bool facDetection;
    bool facRecognizing;
    bool facRecognizing2D;
    bool hriFollowing;
    bool hriFindingLegs;
    bool navDetectingObstacles;
    bool initKnownLoacations;
    bool defInitKnownLoacations;
    bool updateKnownLoacations;
    bool enableInteractiveEdit;
    bool enableObjDetectYOLO;

    std::map<std::string, std::vector<std::string> > locations;
    std::map<std::string, std::vector<std::string> > objects;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);
    void setPathKnownLoc(const std::string pathKnownLoc);
    void setConfigFile(const std::string configFile);

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
    void facDBtnStartClicked();
    void facBtnStartClicked();
    void facBtnStartClicked2D();
    void facRecogPressed();
    void facTrainPressed();
    void facClearPressed();
    void objRecogObjectChanged();
    void vsnFindLinesClicked();
    void detectObjYOLOClicked();
    void enableObjYOLOClicked();
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

    void on_typeView_currentIndexChanged(const QString &arg1);

    void on_actBtnExecRobocup_pressed();


private:
    Ui::MainWindow *ui;
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    QToolBar* toolbar_;
    QAction* add_tool_action_;
    QMenu* remove_tool_menu_;
    QActionGroup* toolbar_actions_;
    std::string pathKnownLoc;
    std::string configFile;
    std::string configFileViz;
    rviz::Config configNav;
    rviz::Config configViz;

    void initToolbars();

    enum Column{
        NAME, X, Y, A, C1, C2, C3, C4
    };

    enum ColumnObj{
        ID, CONFIDENCE
    };


};

#endif // MAINWINDOW_H
