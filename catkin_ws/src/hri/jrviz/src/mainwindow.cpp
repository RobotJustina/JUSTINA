#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <boost/program_options.hpp>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <QtWidgets/QFileDialog>
#include <QToolBar>

#include<fstream>
#include<string.h>
#include<iostream>
#include<QDebug>
#include <QMessageBox>

MainWindow::MainWindow(std::string configFile, std::string configFileViz, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow){
    ui->setupUi(this);
    this->configFile = configFile;

    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanel();
    //QVBoxLayout* main_layout = new QVBoxLayout;
    //main_layout->addLayout( controls_layout );
    //main_layout->addWidget( render_panel_ );
    this->ui->rvizLayout->addWidget(render_panel_);

    // Set the top-level layout for this MyViz widget.
    // setLayout( main_layout );

    // Make signal/slot connections.

    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // std::string actual_load_path = "/opt/codigo/JUSTINA/catkin_ws/src/planning/knowledge/hri/rviz_config.rviz";
    // std::string acutal_load_path_nav = "/opt/codigo/JUSTINA/catkin_ws/src/planning/knowledge/hri/rviz_config_nav.rviz";
    rviz::YamlConfigReader readerNav;
    rviz::YamlConfigReader readerViz;
    readerViz.readFile(configViz, QString::fromStdString(configFile));
    readerNav.readFile(configNav, QString::fromStdString(configFileViz));
    manager_->load(configNav.mapGetChild("Visualization Manager"));

    this->ui->typeView->addItem("Navigation");
    this->ui->typeView->addItem("Visualization");
    this->ui->typeView->setCurrentIndex(0);

    this->ui->actCmbRobocup->addItem("Storing Groceries");

    this->ui->laRbArticular->setChecked(true);
    this->ui->raRbArticular->setChecked(true);
    this->laLastRadioButton = 0;
    this->raLastRadioButton = 0;
    this->recSavingVideo = false;
    this->facDetection = false;
    this->facRecognizing = false;
    this->facRecognizing2D = false;
    this->sktRecognizing = false;
    this->hriFollowing = false;
    this->hriFindingLegs = false;
    this->navDetectingObstacles = false;
    this->enableInteractiveEdit = false;
    this->enableObjDetectYOLO = false;
    setPathKR();

    QObject::connect(ui->btnStop, SIGNAL(clicked()), this, SLOT(stopRobot()));
    //Navigation
    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));
    QObject::connect(ui->navTxtMove, SIGNAL(returnPressed()), this, SLOT(navMoveChanged()));
    QObject::connect(ui->navBtnStartObsDetection, SIGNAL(clicked()), this, SLOT(navObsDetectionEnableClicked()));
    //Hardware
    QObject::connect(ui->hdTxtPan, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    QObject::connect(ui->hdTxtTilt, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    QObject::connect(ui->laTxtAngles0, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtOpenGripper, SIGNAL(valueChanged(double)), this, SLOT(laOpenGripperChanged(double)));
    QObject::connect(ui->laTxtCloseGripper, SIGNAL(valueChanged(double)), this, SLOT(laCloseGripperChanged(double)));
    QObject::connect(ui->raTxtAngles0, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtOpenGripper, SIGNAL(valueChanged(double)), this, SLOT(raOpenGripperChanged(double)));
    QObject::connect(ui->raTxtCloseGripper, SIGNAL(valueChanged(double)), this, SLOT(raCloseGripperChanged(double)));
    QObject::connect(ui->laTxtXYZ, SIGNAL(returnPressed()), this, SLOT(laValuesChanged()));
    QObject::connect(ui->laTxtRPY, SIGNAL(returnPressed()), this, SLOT(laValuesChanged()));
    QObject::connect(ui->laTxtElbow, SIGNAL(returnPressed()), this, SLOT(laValuesChanged()));
    QObject::connect(ui->raTxtXYZ, SIGNAL(returnPressed()), this, SLOT(raValuesChanged()));
    QObject::connect(ui->raTxtRPY, SIGNAL(returnPressed()), this, SLOT(raValuesChanged()));
    QObject::connect(ui->raTxtElbow, SIGNAL(returnPressed()), this, SLOT(raValuesChanged()));
    QObject::connect(ui->laRbCartesian, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbCartesianRobot, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbArticular, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->raRbCartesian, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbCartesianRobot, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbArticular, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    //Torso
    QObject::connect(ui->trsTxtSpine, SIGNAL(valueChanged(double)), this, SLOT(torsoPoseChanged(double)));
    QObject::connect(ui->trsTxtWaist, SIGNAL(valueChanged(double)), this, SLOT(torsoPoseChanged(double)));
    QObject::connect(ui->trsTxtShoulders, SIGNAL(valueChanged(double)), this, SLOT(torsoPoseChanged(double)));
    QObject::connect(ui->trsTxtLoc, SIGNAL(returnPressed()), this, SLOT(torsoLocChanged()));
    //Speech synthesis and recog
    QObject::connect(ui->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgSayChanged()));
    QObject::connect(ui->sprTxtFakeRecog, SIGNAL(returnPressed()), this, SLOT(sprFakeRecognizedChanged()));
    //Vision
    QObject::connect(ui->recBtnSaveVideo, SIGNAL(clicked()), this, SLOT(recSaveVideoChanged()));
    QObject::connect(ui->recTxtImgFile, SIGNAL(returnPressed()), this, SLOT(recSaveImageChanged()));
    QObject::connect(ui->recBtnSaveImg, SIGNAL(clicked()), this, SLOT(recSaveImageChanged()));
    QObject::connect(ui->sktBtnStartRecog, SIGNAL(clicked()), this, SLOT(sktBtnStartClicked()));
    QObject::connect(ui->facBtnStartDetec, SIGNAL(clicked()), this, SLOT(facDBtnStartClicked()));
    QObject::connect(ui->facBtnStartRecog, SIGNAL(clicked()), this, SLOT(facBtnStartClicked()));
    QObject::connect(ui->facBtnStartRecog2D, SIGNAL(clicked()), this, SLOT(facBtnStartClicked2D()));

    QObject::connect(ui->facTxtRecog, SIGNAL(returnPressed()), this, SLOT(facRecogPressed()));
    QObject::connect(ui->facTxtTrain, SIGNAL(returnPressed()), this, SLOT(facTrainPressed()));
    QObject::connect(ui->facTxtClear, SIGNAL(returnPressed()), this, SLOT(facClearPressed()));
    QObject::connect(ui->objTxtGoalObject, SIGNAL(returnPressed()), this, SLOT(objRecogObjectChanged()));
    QObject::connect(ui->vsnBtnFindLines, SIGNAL(clicked()), this, SLOT(vsnFindLinesClicked()));
    QObject::connect(ui->detectObjYOLO, SIGNAL(clicked()), this, SLOT(detectObjYOLOClicked()));
    QObject::connect(ui->enObjDetectYOLO, SIGNAL(clicked()), this, SLOT(enableObjYOLOClicked()));
    //HRI
    QObject::connect(ui->hriBtnStartFollow, SIGNAL(clicked()), this, SLOT(hriBtnFollowClicked()));
    QObject::connect(ui->hriBtnStartLegs, SIGNAL(clicked()), this, SLOT(hriBtnLegsClicked()));
    //Knowledge
    QObject::connect(ui->locTableWidget->horizontalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(on_removeLoc_clicked()));
    QObject::connect(ui->quesReq, SIGNAL(returnPressed()), this, SLOT(quesReqChanged()));
    //K_representation
    QObject::connect(ui->enterCommand, SIGNAL(returnPressed()), this, SLOT(enterCommandChanged()));
    QObject::connect(ui->loadCommand, SIGNAL(returnPressed()), this, SLOT(loadCommandChanged()));

    this->robotX = 0;
    this->robotY = 0;
    this->robotTheta = 0;
    this->laIgnoreValueChanged = false;
    this->raIgnoreValueChanged = false;
    this->initKnownLoacations = false                                                                   ;
    this->defInitKnownLoacations = true;
    this->updateKnownLoacations = false;

    QStringList titles;
    titles << "Name" << "X" << "Y" << "A";
    this->ui->locTableWidget->setColumnCount(4);
    this->ui->locTableWidget->setHorizontalHeaderLabels(titles);

    QStringList tableObjYOLOTitles;
    tableObjYOLOTitles << "ID" << "CONFIDENCE";
    this->ui->objTableWidgetYOLO->setColumnCount(2);
    this->ui->objTableWidgetYOLO->setHorizontalHeaderLabels(tableObjYOLOTitles);

    QStringList locClipsTitles;
    locClipsTitles << "Type" << "Name" << "Quantity" << "Room";
    this->ui->locCLIPStab->setColumnCount(4);
    this->ui->locCLIPStab->setHorizontalHeaderLabels(locClipsTitles);

    QStringList objClipsTitles;
    objClipsTitles << "Name" << "Category" << "Location" << "Room" << "Weight" << "Size" << "Color" << "Quantity"<<"Biggest"<<"Smallest"<<"Heaviest"<<"Lightest";
    this->ui->objCLIPStab->setColumnCount(12);
    this->ui->objCLIPStab->setHorizontalHeaderLabels(objClipsTitles);


    QStringList peopleClipsTitles;
    peopleClipsTitles <<"Name"<<"Age"<<"Gender";
    this->ui->peopleCLIPStab->setColumnCount(3);
    this->ui->peopleCLIPStab->setHorizontalHeaderLabels(peopleClipsTitles);

    QStringList categoryClipsTitles;
    categoryClipsTitles<<"Name"<<"Zone"<<"Quantity"<<"Biggest"<<"Smallest"<<"Heaviest"<<"Lightest";
    this->ui->categoryCLIPStab->setColumnCount(7);
    this->ui->categoryCLIPStab->setHorizontalHeaderLabels(categoryClipsTitles);

    setlocClips();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode){
    this->qtRosNode = qtRosNode;
    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));
}
//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

void MainWindow::setPathKnownLoc(const std::string pathKnownLoc){
  this->pathKnownLoc = pathKnownLoc;
}

void MainWindow::setConfigFile(const std::string configFile){
    this->configFile = configFile;
}

bool MainWindow::strToFloatArray(std::string str, std::vector<float>& result)
{
    result.clear();
    std::vector<std::string> parts;
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    for(size_t i=0; i < parts.size(); i++)
    {
        std::stringstream ssValue(parts[i]);
        float value;
        if(!(ssValue >> value))
            return false;
        result.push_back(value);
    }
    return true;
}

void MainWindow::stopRobot()
{
    JustinaHardware::stopRobot();
}

void MainWindow::navBtnCalcPath_pressed()
{
    float startX, startY, startTheta;
    float goalX = 0;
    float goalY = 0;
    float goalTheta;
    std::string start_location = "";
    std::string goal_location = "";
    std::vector<std::string> parts;

    std::string str = this->ui->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    {
        this->ui->navTxtStartPose->setText("Robot");
        JustinaNavigation::getRobotPose(this->robotX, this->robotY, this->robotTheta);
        startX = this->robotX;
        startY = this->robotY;
        startTheta = this->robotTheta;
    }
    else if(parts.size() >= 2) //Given data correspond to numbers
    {
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
        if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else //Given data correspond to location
        start_location = parts[0];

    str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else
        goal_location = parts[0];

    if(start_location.compare("") == 0 && goal_location.compare("") == 0)
        JustinaNavigation::planPath(startX, startY, goalX, goalY, this->calculatedPath);
    else if(start_location.compare("") == 0 && goal_location.compare("") != 0)
        JustinaNavigation::planPath(startX, startY, goal_location, this->calculatedPath);
    else if(start_location.compare("") != 0 && goal_location.compare("") == 0)
        JustinaNavigation::planPath(start_location, goalX, goalY, this->calculatedPath);
    else
        JustinaNavigation::planPath(start_location, goal_location, this->calculatedPath);
    std::vector<std::string> locations = JustinaKnowledge::getRoomsFromPath(this->calculatedPath);
    std::cout << "QMainWindow.->Locations visit with path:";
    for(int i = 0; i < locations.size(); i++){
        std::cout << locations[i];
        if(i < locations.size() - 1)
            std::cout << ", ";
    }
    std::cout << std::endl;
}

void MainWindow::navBtnExecPath_pressed()
{
    float goalX = 0;
    float goalY = 0;
    float goalTheta;
    std::string goal_location = "";
    std::vector<std::string> parts;

    std::string str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
        if(parts.size() > 2)
        {
            std::stringstream ssGoalAngle(parts[2]);
            if(!(ssGoalAngle >> goalTheta))
            {
                this->ui->navTxtStartPose->setText("Invalid format");
                return;
            }
            //this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
            JustinaNavigation::startGetClose(goalX, goalY, goalTheta);
        }
        else
        {
            // this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
            JustinaNavigation::startGetClose(goalX, goalY);
        }
        return;
    }
    else
    {
        goal_location = parts[0];
        //this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
        JustinaNavigation::startGetClose(goal_location);
    }
}

void MainWindow::navMoveChanged()
{
    std::vector<std::string> parts;
    std::string str = this->ui->navTxtMove->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() < 1)
        return;

    if(parts[0].compare("l") == 0)
    {
        if(parts.size() < 2)
            return;
        std::stringstream ssLateral(parts[1]);
        float lateral;
        if(!(ssLateral >> lateral))
            return;
        JustinaNavigation::startMoveLateral(lateral);
        return;
    }

    float dist = 0;
    float angle = 0;
    std::stringstream ssDist(parts[0]);
    if(!(ssDist >> dist))
        return;
    if(parts.size() > 1)
    {
        std::stringstream ssAngle(parts[1]);
        if(!(ssAngle >> angle))
            return;
    }
    JustinaNavigation::startMoveDistAngle(dist, angle);
}

void MainWindow::navObsDetectionEnableClicked()
{
    if(this->navDetectingObstacles)
    {
        JustinaNavigation::enableObstacleDetection(false);
        this->navDetectingObstacles = false;
        this->ui->navBtnStartObsDetection->setText("Enable");
    }
    else
    {
        JustinaNavigation::enableObstacleDetection(true);
        this->navDetectingObstacles = true;
        this->ui->navBtnStartObsDetection->setText("Disable");
    }
}

void MainWindow::hdPanTiltChanged(double)
{
    float goalPan = this->ui->hdTxtPan->value();
    float goalTilt = this->ui->hdTxtTilt->value();
    std::cout << "QMainWindow.->Setting new head goal pose: " << goalPan << "  " << goalTilt  << std::endl;
    //JustinaHardware::setHeadGoalPose(goalPan, goalTilt);
    JustinaManip::startHdGoTo(goalPan, goalTilt);
}

void MainWindow::laAnglesChanged(double d)
{
    if(this->laIgnoreValueChanged)
        return;

    std::vector<float> goalAngles;
    goalAngles.push_back(this->ui->laTxtAngles0->value());
    goalAngles.push_back(this->ui->laTxtAngles1->value());
    goalAngles.push_back(this->ui->laTxtAngles2->value());
    goalAngles.push_back(this->ui->laTxtAngles3->value());
    goalAngles.push_back(this->ui->laTxtAngles4->value());
    goalAngles.push_back(this->ui->laTxtAngles5->value());
    goalAngles.push_back(this->ui->laTxtAngles6->value());

    JustinaManip::startLaGoToArticular(goalAngles);
}

void MainWindow::raAnglesChanged(double d)
{
    if(this->raIgnoreValueChanged)
        return;

    std::vector<float> goalAngles;
    goalAngles.push_back(this->ui->raTxtAngles0->value());
    goalAngles.push_back(this->ui->raTxtAngles1->value());
    goalAngles.push_back(this->ui->raTxtAngles2->value());
    goalAngles.push_back(this->ui->raTxtAngles3->value());
    goalAngles.push_back(this->ui->raTxtAngles4->value());
    goalAngles.push_back(this->ui->raTxtAngles5->value());
    goalAngles.push_back(this->ui->raTxtAngles6->value());

    JustinaManip::startRaGoToArticular(goalAngles);
}

void MainWindow::laValuesChanged()
{
    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> values;
    this->strToFloatArray(this->ui->laTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->laTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->laTxtElbow->text().toStdString(), elbow);
    values.insert(values.end(), xyz.begin(), xyz.end());
    values.insert(values.end(), rpy.begin(), rpy.end());
    values.insert(values.end(), elbow.begin(), elbow.end());
    bool success = values.size() == 7 || values.size() == 6 || values.size() == 3;
    if(!success) //If cannot get floats, then it is assumed that a predefined position is given
    {
        std::string goalLoc = this->ui->laTxtXYZ->text().toStdString();
        if(goalLoc.compare("") != 0)
        {
            JustinaManip::startLaGoTo(goalLoc);
            //JustinaManip::laGoTo(goalLoc, 5000);
        }
    }
    else
    {
        if(this->ui->laRbCartesianRobot->isChecked())
            JustinaManip::startLaGoToCartesianWrtRobot(values);
        else if(this->ui->laRbCartesian->isChecked())
            JustinaManip::startLaGoToCartesian(values);
        else if(this->ui->laRbArticular->isChecked())
            JustinaManip::startLaGoToArticular(values);
    }
}

void MainWindow::raValuesChanged()
{
    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> values;
    this->strToFloatArray(this->ui->raTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->raTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->raTxtElbow->text().toStdString(), elbow);
    values.insert(values.end(), xyz.begin(), xyz.end());
    values.insert(values.end(), rpy.begin(), rpy.end());
    values.insert(values.end(), elbow.begin(), elbow.end());
    bool success = values.size() == 7 || values.size() == 6 || values.size() == 3;
    if(!success) //If cannot get floats, then it is assumed that a predefined position is given
    {
        std::string goalLoc = this->ui->raTxtXYZ->text().toStdString();
        if(goalLoc.compare("") != 0)
        {
            JustinaManip::startRaGoTo(goalLoc);
            //JustinaManip::raGoTo(goalLoc, 5000);
        }
    }
    else
    {
        if(this->ui->raRbCartesianRobot->isChecked())
            JustinaManip::startRaGoToCartesianWrtRobot(values);
        else if(this->ui->raRbCartesian->isChecked())
            JustinaManip::startRaGoToCartesian(values);
        else if(this->ui->raRbArticular->isChecked())
            JustinaManip::startRaGoToArticular(values);
    }
}

void MainWindow::laOpenGripperChanged(double d)
{
    JustinaManip::startLaOpenGripper((float)d);
}

void MainWindow::raOpenGripperChanged(double d)
{
    JustinaManip::startRaOpenGripper((float)d);
}

void MainWindow::laCloseGripperChanged(double d)
{
    JustinaManip::startLaCloseGripper((float)d);
}

void MainWindow::raCloseGripperChanged(double d)
{
    JustinaManip::startRaCloseGripper((float)d);
}

void MainWindow::laRadioButtonClicked()
{
    int currentRb = -1;
    if(this->ui->laRbArticular->isChecked()) currentRb = 0;
    else if(this->ui->laRbCartesian->isChecked()) currentRb = 1;
    else if(this->ui->laRbCartesianRobot->isChecked()) currentRb = 2;
    else return;

    if(currentRb == this->laLastRadioButton)
        return;

    this->laIgnoreValueChanged = true;

    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> oldValues;
    std::vector<float> newValues;
    this->strToFloatArray(this->ui->laTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->laTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->laTxtElbow->text().toStdString(), elbow);
    oldValues.insert(oldValues.end(), xyz.begin(), xyz.end());
    oldValues.insert(oldValues.end(), rpy.begin(), rpy.end());
    oldValues.insert(oldValues.end(), elbow.begin(), elbow.end());
    bool success = oldValues.size() == 7;

    if(!success)
    {
        this->laIgnoreValueChanged = false;
        if(this->laLastRadioButton == 0) this->ui->laRbArticular->setChecked(true);
        if(this->laLastRadioButton == 1) this->ui->laRbCartesian->setChecked(true);
        if(this->laLastRadioButton == 2) this->ui->laRbCartesianRobot->setChecked(true);
        return;
    }

    if(this->ui->laRbArticular->isChecked())
    {
        this->ui->laLblGoalValues->setText("Angles:");
        if(this->laLastRadioButton == 2)
            JustinaTools::transformPose("base_link", oldValues, "left_arm_link0", oldValues);
        success = JustinaManip::inverseKinematics(oldValues, newValues);
    }
    else if(this->ui->laRbCartesian->isChecked())
    {
        this->ui->laLblGoalValues->setText("XYZ  RPY  Elbow:");
        if(this->laLastRadioButton == 0)
            success = JustinaManip::directKinematics(newValues, oldValues);
        else
            success = JustinaTools::transformPose("base_link", oldValues, "left_arm_link1", newValues);
    }
    else
    {
        this->ui->laLblGoalValues->setText("XYZ  RPY  Elbow:");
        if(this->laLastRadioButton == 0)
            success = JustinaManip::directKinematics(oldValues, oldValues);

        success = JustinaTools::transformPose("left_arm_link1", oldValues, "base_link", newValues);
    }
    if(!success)
    {
        this->laIgnoreValueChanged = false;
        if(this->laLastRadioButton == 0) this->ui->laRbArticular->setChecked(true);
        if(this->laLastRadioButton == 1) this->ui->laRbCartesian->setChecked(true);
        if(this->laLastRadioButton == 2) this->ui->laRbCartesianRobot->setChecked(true);
        return;
    }

    QString str = QString::number(newValues[0],'f',3)+"  "+QString::number(newValues[1],'f',3)+"  "+QString::number(newValues[2],'f',3);
    this->ui->laTxtXYZ->setText(str);
    str = QString::number(newValues[3],'f',3)+"  "+QString::number(newValues[4],'f',3)+"  "+QString::number(newValues[5],'f',3);
    this->ui->laTxtRPY->setText(str);
    str = QString::number(newValues[6],'f',3);
    this->ui->laTxtElbow->setText(str);

    this->laLastRadioButton = currentRb;
    this->laIgnoreValueChanged = false;
}

void MainWindow::raRadioButtonClicked()
{
    int currentRb = -1;
    if(this->ui->raRbArticular->isChecked()) currentRb = 0;
    else if(this->ui->raRbCartesian->isChecked()) currentRb = 1;
    else if(this->ui->raRbCartesianRobot->isChecked()) currentRb = 2;
    else return;

    if(currentRb == this->raLastRadioButton)
        return;

    this->raIgnoreValueChanged = true;

    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> oldValues;
    std::vector<float> newValues;
    this->strToFloatArray(this->ui->raTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->raTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->raTxtElbow->text().toStdString(), elbow);
    oldValues.insert(oldValues.end(), xyz.begin(), xyz.end());
    oldValues.insert(oldValues.end(), rpy.begin(), rpy.end());
    oldValues.insert(oldValues.end(), elbow.begin(), elbow.end());
    bool success = oldValues.size() == 7;

    if(!success)
    {
        this->raIgnoreValueChanged = false;
        if(this->raLastRadioButton == 0) this->ui->raRbArticular->setChecked(true);
        if(this->raLastRadioButton == 1) this->ui->raRbCartesian->setChecked(true);
        if(this->raLastRadioButton == 2) this->ui->raRbCartesianRobot->setChecked(true);
        return;
    }

    if(this->ui->raRbArticular->isChecked())
    {
        this->ui->raLblGoalValues->setText("Angles:");
        if(this->raLastRadioButton == 2)
            JustinaTools::transformPose("base_link", oldValues, "right_arm_link1", oldValues);
        success = JustinaManip::inverseKinematics(oldValues, newValues);
    }
    else if(this->ui->raRbCartesian->isChecked())
    {
        this->ui->raLblGoalValues->setText("XYZ  RPY  Elbow:");
        if(this->raLastRadioButton == 0)
            success = JustinaManip::directKinematics(newValues, oldValues);
        else
            success = JustinaTools::transformPose("base_link", oldValues, "right_arm_link0", newValues);
    }
    else
    {
        this->ui->raLblGoalValues->setText("XYZ  RPY  Elbow:");
        if(this->raLastRadioButton == 0)
            success = JustinaManip::directKinematics(oldValues, oldValues);

        success = JustinaTools::transformPose("right_arm_link1", oldValues, "base_link", newValues);
    }
    if(!success)
    {
        this->raIgnoreValueChanged = false;
        if(this->raLastRadioButton == 0) this->ui->raRbArticular->setChecked(true);
        if(this->raLastRadioButton == 1) this->ui->raRbCartesian->setChecked(true);
        if(this->raLastRadioButton == 2) this->ui->raRbCartesianRobot->setChecked(true);
        return;
    }

    QString str = QString::number(newValues[0],'f',3)+"  "+QString::number(newValues[1],'f',3)+"  "+QString::number(newValues[2],'f',3);
    this->ui->raTxtXYZ->setText(str);
    str = QString::number(newValues[3],'f',3)+"  "+QString::number(newValues[4],'f',3)+"  "+QString::number(newValues[5],'f',3);
    this->ui->raTxtRPY->setText(str);
    str = QString::number(newValues[6],'f',3);
    this->ui->raTxtElbow->setText(str);

    this->raLastRadioButton = currentRb;
    this->raIgnoreValueChanged = false;
}

void MainWindow::torsoPoseChanged(double d)
{
    float goalSpine = this->ui->trsTxtSpine->value();
    float goalWaist = this->ui->trsTxtWaist->value();
    float goalShoulders = this->ui->trsTxtShoulders->value();
    std::cout << "QMainWindow.->Setting new torso pose: " << goalSpine << "  " << goalWaist << "  " << goalShoulders << std::endl;
    JustinaManip::startTorsoGoTo(goalSpine, goalWaist, goalShoulders);
    this->ui->trsLblStatus->setText("Status: Moving to ...");
}

void MainWindow::torsoLocChanged()
{
}

void MainWindow::spgSayChanged()
{
    std::string strToSay = this->ui->spgTxtSay->text().toStdString();
    std::cout << "QMainWindow.->Saying: " << strToSay << std::endl;
    JustinaHRI::say(strToSay);
}

void MainWindow::sprFakeRecognizedChanged()
{
    std::string strToFake = this->ui->sprTxtFakeRecog->text().toStdString();
    std::cout << "QMainWindow.->Faking recog speech: " << strToFake << std::endl;
    JustinaHRI::fakeSpeechRecognized(strToFake);
}

void MainWindow::recSaveVideoChanged()
{
    if(this->recSavingVideo)
    {
        std::cout << "QMainWindow.->Stop saving video." << std::endl;
        JustinaHardware::stopSavingCloud();
        this->ui->recBtnSaveVideo->setText("Start saving video");
        this->ui->recLblStatus->setText("Status: Stand by");
        this->recSavingVideo = false;
    }
    else
    {
        std::string fileName = this->ui->recTxtVideoFile->text().toStdString();
        if(!boost::filesystem::portable_posix_name(fileName))
        {
            std::cout << "QMainWindow.->File name for video is not a valid name :'(" << std::endl;
            this->ui->recLblStatus->setText("Status: Invalid file name...");
            return;
        }
        std::cout << "QMainWindow.->Starting to save video at: " << fileName << std::endl;
        JustinaHardware::startSavingCloud(fileName);
        this->ui->recBtnSaveVideo->setText("Stop saving video");
        this->ui->recLblStatus->setText("Status: saving video...");
        this->recSavingVideo = true;
    }
}

void MainWindow::recSaveImageChanged()
{
}

void MainWindow::sktBtnStartClicked()
{
    if(this->sktRecognizing)
    {
        JustinaVision::stopSkeletonFinding();
        this->sktRecognizing = false;
        this->ui->sktBtnStartRecog->setText("Start Skeletons");
    }
    else
    {
        JustinaVision::startSkeletonFinding();
        this->sktRecognizing = true;
        this->ui->sktBtnStartRecog->setText("Stop Skeletons");
    }
}

void MainWindow::facDBtnStartClicked()
{
    if(this->facDetection)
    {
        JustinaVision::startFaceDetection(false);
        this->facDetection = false;
        this->ui->facBtnStartDetec->setText("Start Detection");
    }
    else
    {
        JustinaVision::startFaceDetection(true);
        this->facDetection = true;
        this->ui->facBtnStartDetec->setText("Stop Detection");
    }
}

void MainWindow::facBtnStartClicked()
{
    if(this->facRecognizing)
    {
        JustinaVision::startFaceRecognition(false);
        this->facRecognizing = false;
        this->ui->facBtnStartRecog->setText("Start Recognition");
    }
    else
    {
        JustinaVision::startFaceRecognition(true);
        this->facRecognizing = true;
        this->ui->facBtnStartRecog->setText("Stop Recognition");
    }
}

void MainWindow::facBtnStartClicked2D()
{
    if(this->facRecognizing2D)
    {
        JustinaVision::startFaceRecognition2D(false);
        this->facRecognizing2D = false;
        this->ui->facBtnStartRecog2D->setText("Start Recognizer 2D");
    }
    else
    {
        JustinaVision::startFaceRecognition2D(true);
        this->facRecognizing2D = true;
        this->ui->facBtnStartRecog2D->setText("Stop Recognizing 2D");
    }
}

void MainWindow::facRecogPressed()
{
    std::string id = this->ui->facTxtRecog->text().toStdString();
    if(id.compare("") == 0)
    {
        //std::cout << "QMainWindow.->Starting recognition without id" << std::endl;
        return;
    }
    if(!boost::filesystem::portable_posix_name(id))
    {
        //std::cout << "QMainWindow.->Invalid ID for face recognition. " << std::endl;
        return;
    }
    JustinaVision::setIdFaceRecognition(id);
}

void MainWindow::facTrainPressed()
{
    std::string str = this->ui->facTxtTrain->text().toStdString();
    std::vector<std::string> parts;
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() < 1)
        return;

    int numOfFrames = -1;
    if(!boost::filesystem::portable_posix_name(parts[0]))
    {
        std::cout << "QMainWindow.->Invalid ID for face training. " << std::endl;
        return;
    }
    if(parts.size() > 1)
    {
        std::stringstream ssValue(parts[1]);
        if(!(ssValue >> numOfFrames) || numOfFrames <= 0)
        {
            std::cout << "QMainWindow.->Invalid number of frames for face training. " << std::endl;
            return;
        }
    }

    if(numOfFrames <= 0)
    {
        std::cout << "QMainWindow.->Sending face training without number of frames. " << std::endl;
        JustinaVision::faceTrain(parts[0], 1);
        return;
    }
    std::cout << "QMainWindow.->Sending face training with " << numOfFrames << " number of frames. " << std::endl;
    JustinaVision::faceTrain(parts[0], numOfFrames);
    return;
}

void MainWindow::facClearPressed()
{
    std::string str = this->ui->facTxtClear->text().toStdString();
    if(str.compare("ALL") == 0)
    {
        std::cout << "QMainWindow.->Clearing all face recognition database" << std::endl;
        JustinaVision::facClearAll();
        return;
    }
    if(!boost::filesystem::portable_posix_name(str))
    {
        std::cout << "QMainWindow.->Invalid ID for clearing face database. " << std::endl;
        return;
    }
    JustinaVision::facClearByID(str);
}

void MainWindow::objRecogObjectChanged()
{
    std::vector<vision_msgs::VisionObject> recoObjList;
    JustinaRepresentation::initKDB("", true, 0);
    if(!JustinaVision::detectObjects(recoObjList))
    {
        std::cout << "MainWindow.->Cannot dectect objects :'( " << std::endl;
        return;
    }
    QString txtResult = "";
    this->ui->objTxtResults->setPlainText(txtResult);
    for(int i=0; i < recoObjList.size(); i++)
    {
        txtResult = "Id: " + QString::fromStdString(recoObjList[i].id);
        this->ui->objTxtResults->appendPlainText(txtResult);
        txtResult = "Centroid:";
        this->ui->objTxtResults->appendPlainText(txtResult);
        txtResult = QString::number(recoObjList[i].pose.position.x, 'f', 3) + "  " + QString::number(recoObjList[i].pose.position.y, 'f', 3) +
            "  " + QString::number(recoObjList[i].pose.position.z, 'f', 3);
        this->ui->objTxtResults->appendPlainText(txtResult);
        txtResult = "";
        this->ui->objTxtResults->appendPlainText(txtResult);
    }
}

void MainWindow::vsnFindLinesClicked()
{
    float x1, y1, z1, x2, y2, z2;
    JustinaVision::findLine(x1, y1, z1, x2, y2, z2);
}

void MainWindow::detectObjYOLOClicked(){
    this->ui->objTableWidgetYOLO->setRowCount(0);

    std::vector<vision_msgs::VisionObject> objRecoYOLO;
    JustinaVision::detectObjectsYOLO(objRecoYOLO);
    std::vector<vision_msgs::VisionObject>::iterator itObjRecoYOLO;
    for(itObjRecoYOLO = objRecoYOLO.begin(); itObjRecoYOLO != objRecoYOLO.end(); itObjRecoYOLO++){
        this->ui->objTableWidgetYOLO->insertRow(this->ui->objTableWidgetYOLO->rowCount());
        float row = this->ui->objTableWidgetYOLO->rowCount() - 1;
        this->ui->objTableWidgetYOLO->setItem(row, ID, new QTableWidgetItem(QString::fromStdString(itObjRecoYOLO->id)));
        this->ui->objTableWidgetYOLO->setItem(row, CONFIDENCE, new QTableWidgetItem(QString::number(itObjRecoYOLO->confidence)));
    }
    this->ui->objTableWidgetYOLO->resizeRowsToContents();
    this->ui->objTableWidgetYOLO->resizeColumnsToContents();
}

void MainWindow::enableObjYOLOClicked(){
    if(enableObjDetectYOLO){
        JustinaVision::enableDetectObjsYOLO(false);
        this->ui->enObjDetectYOLO->setText("Enable");
        enableObjDetectYOLO = false;
    }
    else{
        JustinaVision::enableDetectObjsYOLO(true);
        this->ui->enObjDetectYOLO->setText("Disable");
        enableObjDetectYOLO = true;
    }
}

//HRI
void MainWindow::hriBtnFollowClicked()
{
    if(this->hriFollowing)
    {
        this->ui->hriBtnStartFollow->setText("Start Follow");
        JustinaHRI::stopFollowHuman();
        this->hriFollowing = false;
    }
    else
    {
        this->ui->hriBtnStartFollow->setText("Stop Follow");
        JustinaHRI::startFollowHuman();
        this->hriFollowing = true;
    }
}

void MainWindow::hriBtnLegsClicked()
{
    if(this->hriFindingLegs)
    {
        JustinaHRI::enableLegFinder(false);
        this->ui->hriBtnStartLegs->setText("Start Leg Finder");
        this->hriFindingLegs = false;
    }
    else
    {
        JustinaHRI::enableLegFinder(true);
        this->ui->hriBtnStartLegs->setText("Stop Leg Finder");
        this->hriFindingLegs = true;
    }
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    float rX;
    float rY;
    float rT;
    JustinaNavigation::getRobotPose(rX, rY, rT);
    //std::cout << "MainWindow.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    QString robotTxt = "Robot Pose: "+ QString::number(rX,'f',3) + "  " + QString::number(rY,'f',3) + "  " + QString::number(rT,'f',4);
    this->ui->navLblRobotPose->setText(robotTxt);
    this->robotX = rX;
    this->robotY = rY;
    this->robotTheta = rT;

    float pan;
    float tilt;
    JustinaHardware::getHeadCurrentPose(pan, tilt);
    QString headTxt = QString::number(pan, 'f', 4) + "  " + QString::number(tilt, 'f', 4);
    this->ui->hdLblHeadPose->setText(headTxt);
    this->headPan = pan;
    this->headTilt = tilt;

    if(JustinaNavigation::isGoalReached())
        this->ui->navLblStatus->setText("Base Status: Goal Reached (Y)");
    else
        this->ui->navLblStatus->setText("Base Status: Moving to goal pose...");

    if(JustinaManip::isLaGoalReached())
        this->ui->laLblStatus->setText("LA: Goal Reached (Y)");
    else
        this->ui->laLblStatus->setText("LA: Moving to goal...");

    if(JustinaManip::isRaGoalReached())
        this->ui->raLblStatus->setText("RA: Goal Reached (Y)");
    else
        this->ui->raLblStatus->setText("RA: Moving to goal...");

    if(JustinaManip::isHdGoalReached())
        this->ui->hdLblStatus->setText("Status: Goal Pose reached (Y)");
    else
        this->ui->hdLblStatus->setText("Status: Moving to goal pose...");

    if(JustinaManip::isTorsoGoalReached())
        this->ui->trsLblStatus->setText("Status: Goal Reached!");

    std::string faceId = "";
    float facePosX = 0, facePosY = 0, facePosZ = 0;
    float faceConfidence = -1;
    int faceGender = -1;
    bool faceSmiling = false;
    if(JustinaVision::getMostConfidentFace(faceId, facePosX, facePosY, facePosZ, faceConfidence, faceGender, faceSmiling))
    {
        QString faceIdQ = QString::fromStdString("ResultID: " + faceId);
        this->ui->facLblResultID->setText(faceIdQ);
        QString facePos = "Position: " +QString::number(facePosX,'f',2)+" "+QString::number(facePosY,'f',2)+" "+QString::number(facePosZ,'f',2);
        this->ui->facLblResultPose->setText(facePos);
        if(faceGender == 0)
            this->ui->facLblResultGender->setText("Gender: Female");
        else if(faceGender == 1)
            this->ui->facLblResultGender->setText("Gender: Male");
        else
            this->ui->facLblResultGender->setText("Gender: Unknown");
        if(faceSmiling)
            this->ui->facLblResultSmile->setText("Smiling: Yes");
        else
            this->ui->facLblResultSmile->setText("Smiling: No");
    }

    if(JustinaNavigation::obstacleInFront())
        this->ui->navLblObstacleInFront->setText("Obs In Front: True");
    else
        this->ui->navLblObstacleInFront->setText("Obs In Front: False");

    if(JustinaNavigation::collisionRisk())
        this->ui->navLblRiskOfCollision->setText("Risk of Collision: True");
    else
        this->ui->navLblRiskOfCollision->setText("Risk of Collision: False");

    this->ui->sprLblLastRecog->setText(QString::fromStdString("Recog: " + JustinaHRI::lastRecogSpeech()));

    this->ui->pgbBatt1->setValue((JustinaHardware::leftArmBatteryPerc() + JustinaHardware::rightArmBatteryPerc())/2);
    this->ui->pgbBatt2->setValue((JustinaHardware::headBatteryPerc() + JustinaHardware::baseBatteryPerc())/2);
    QString batt1Txt = QString::number((JustinaHardware::leftArmBattery() + JustinaHardware::rightArmBattery())/2, 'f', 2) + " V";
    QString batt2Txt = QString::number((JustinaHardware::headBattery() + JustinaHardware::baseBattery())/2, 'f', 2) + " V";
    this->ui->lblBatt1Level->setText(batt1Txt);
    this->ui->lblBatt2Level->setText(batt2Txt);

    if(enableObjDetectYOLO){
        this->ui->objTableWidgetYOLO->setRowCount(0);

        std::vector<vision_msgs::VisionObject> objRecoYOLO;
        JustinaVision::getObjectsYOLO(objRecoYOLO);
        std::vector<vision_msgs::VisionObject>::iterator itObjRecoYOLO;
        for(itObjRecoYOLO = objRecoYOLO.begin(); itObjRecoYOLO != objRecoYOLO.end(); itObjRecoYOLO++){
            this->ui->objTableWidgetYOLO->insertRow(this->ui->objTableWidgetYOLO->rowCount());
            float row = this->ui->objTableWidgetYOLO->rowCount() - 1;
            this->ui->objTableWidgetYOLO->setItem(row, ID, new QTableWidgetItem(QString::fromStdString(itObjRecoYOLO->id)));
            this->ui->objTableWidgetYOLO->setItem(row, CONFIDENCE, new QTableWidgetItem(QString::number(itObjRecoYOLO->confidence)));
        }
        this->ui->objTableWidgetYOLO->resizeRowsToContents();
        this->ui->objTableWidgetYOLO->resizeColumnsToContents();
    }

    JustinaKnowledge::getInitKnownLoc(initKnownLoacations);
    if(defInitKnownLoacations || initKnownLoacations){
      std::cout << "QMainWindow.->Init know location" << std::endl;
      std::cout << "QMainWindow.->defInitKnownLoacations:" << defInitKnownLoacations << std::endl;
      std::cout << "QMainWindow.->initKnownLoacations:" << initKnownLoacations << std::endl;
      this->ui->locTableWidget->setRowCount(0);

      std::map<std::string, std::vector<float> > loc;
      JustinaKnowledge::getKnownLocations(loc);

      for(std::map<std::string, std::vector<float> >::iterator it = loc.begin(); it != loc.end(); ++it){
        this->ui->locTableWidget->insertRow(this->ui->locTableWidget->rowCount());
        float row = this->ui->locTableWidget->rowCount() - 1;
        this->ui->locTableWidget->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it->first)));
        this->ui->locTableWidget->setItem(row, X, new QTableWidgetItem(QString::number(it->second[0])));
        this->ui->locTableWidget->setItem(row, Y, new QTableWidgetItem(QString::number(it->second[1])));
        if(it->second.size() > 2)
          this->ui->locTableWidget->setItem(row, A, new QTableWidgetItem(QString::number(it->second[2])));
        else
          this->ui->locTableWidget->setItem(row, A, new QTableWidgetItem(""));
      }
      this->ui->locTableWidget->resizeRowsToContents();
      this->ui->locTableWidget->resizeColumnsToContents();
      this->ui->locTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
      this->ui->locTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
      this->ui->locTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
      defInitKnownLoacations = false;
      initKnownLoacations = false;
    }
    else{
      JustinaKnowledge::getUpdateKnownLoc(updateKnownLoacations);
      //std::cout << "QMainWindow.->updateKnownLoacations:" << updateKnownLoacations << std::endl;
      if(updateKnownLoacations){
        std::map<std::string, std::vector<float> > loc;
        JustinaKnowledge::getKnownLocations(loc);
        //std::cout << "QMainWindow.->loc size:" << loc.size() << std::endl;
        int row = 0;
        for(std::map<std::string, std::vector<float> >::iterator it = loc.begin(); it != loc.end(); ++it){
          this->ui->locTableWidget->item(row, X)->setText(QString::number(it->second[0]));
          this->ui->locTableWidget->item(row, Y)->setText(QString::number(it->second[1]));
          if(it->second.size() > 2)
            this->ui->locTableWidget->item(row, A)->setText(QString::number(it->second[2]));
          else
            this->ui->locTableWidget->item(row, A)->setText("");
          row++;
        }
        this->ui->locTableWidget->resizeRowsToContents();
        this->ui->locTableWidget->resizeColumnsToContents();
        updateKnownLoacations = false;
      }
    }
}

void MainWindow::on_enInteractiveEdit_clicked()
{
  if(!enableInteractiveEdit){
    JustinaKnowledge::enableInteractiveUpdate(true);
    this->ui->enInteractiveEdit->setText("Disable Interactive");
    enableInteractiveEdit = true;
  }
  else{
    JustinaKnowledge::enableInteractiveUpdate(false);
    this->ui->enInteractiveEdit->setText("Enable Interactive");
    enableInteractiveEdit = false;
  }
}

void MainWindow::on_removeLoc_clicked()
{
  std::cout << "QMainWindow.->on_removeLoc_clicked:" << std::endl;
  std::string name = this->ui->addNameLoc->text().toStdString();
  JustinaKnowledge::deleteKnownLoc(name);
}

void MainWindow::on_locTableWidget_itemSelectionChanged()
{
  std::cout << "QMainWindow.->on_locTableWidget_itemSelectionChanged:" << std::endl;

  QModelIndexList indexes = this->ui->locTableWidget->selectionModel()->selectedRows();

  foreach(QModelIndex index, indexes){
    //std::cout << "QMainWindow.->updateKnownLoacations:" << updateKnownLoacations << std::endl;
    std::cout << "QMainWindow.->row selected:" << this->ui->locTableWidget->item(index.row(), NAME)->text().toStdString() << std::endl;
    this->ui->addNameLoc->setText(this->ui->locTableWidget->item(index.row(), NAME)->text());
    this->ui->addXLoc->setText(this->ui->locTableWidget->item(index.row(), X)->text());
    this->ui->addYLoc->setText(this->ui->locTableWidget->item(index.row(), Y)->text());
    this->ui->addALoc->setText(this->ui->locTableWidget->item(index.row(), A)->text());
  }
}

void MainWindow::on_addLoc_clicked()
{
    std::cout << "QMainWindow.->on_addLoc_clicked:" << std::endl;
    std::cout << "QMainWindow.->on_addLoc_clicked:" << this->ui->addALoc->text().toStdString() << std::endl;

    std::string name = this->ui->addNameLoc->text().toStdString();
    std::vector<float> values;
    values.push_back(this->ui->addXLoc->text().toFloat());
    values.push_back(this->ui->addYLoc->text().toFloat());
    if(this->ui->addALoc->text().compare("") != 0)
      values.push_back(this->ui->addALoc->text().toFloat());
    JustinaKnowledge::addUpdateKnownLoc(name, values);
}

void MainWindow::on_GetRobotPose_clicked()
{
    float x, y, theta;
    JustinaNavigation::getRobotPose(x, y, theta);
    this->ui->addXLoc->setText(QString::number(x));
    this->ui->addYLoc->setText(QString::number(y));
    this->ui->addALoc->setText(QString::number(theta));
}

void MainWindow::on_loadFromFile_clicked()
{
  QString pathFile = QFileDialog::getOpenFileName(
        this,
        tr("Open File"),
        QString::fromStdString(this->pathKnownLoc),
        "Text File (*.txt)"
        );
  std::cout << "QMainWindow.->pathFile:" << pathFile.toStdString() << std::endl;
  JustinaKnowledge::loadFromFile(pathFile.toStdString());
}

void MainWindow::on_SaveInFile_clicked()
{
  QString pathFile = QFileDialog::getSaveFileName(
        this,
        tr("Save File"),
        QString::fromStdString(this->pathKnownLoc),
        "Text File (*.txt)"
        );
  std::cout << "QMainWindow.->pathFile:" << pathFile.toStdString() << std::endl;
  JustinaKnowledge::saveInFile(pathFile.toStdString());
}

void MainWindow::quesReqChanged(){
  std::cout << "QMainWindow.->quesReq:" << this->ui->quesReq->text().toStdString() << std::endl;
  std::string answer;
  bool found = JustinaKnowledge::comparePredQuestion(
          this->ui->quesReq->text().toStdString(), answer);
  if(found)
    this->ui->browserAnswerResp->setText(QString::fromStdString(answer));
  else{
    std::string answer;
    std::string question = this->ui->quesReq->text().toStdString();
    JustinaRepresentation::initKDB("", true, 0);
    bool success = JustinaRepresentation::answerQuestionFromKDB(question, answer, 1000);
    if(success)
        this->ui->browserAnswerResp->setText(QString::fromStdString(answer));
    else
        this->ui->browserAnswerResp->setText(QString::fromStdString(""));
  }
  //sb->setValue(sb->maximum());
}

void MainWindow::on_runCLIPS_clicked()
{
    JustinaRepresentation::runCLIPS(true);
}

void MainWindow::on_resetCLIPS_clicked()
{
    JustinaRepresentation::resetCLIPS(true);
}

void MainWindow::on_factsCLIPS_clicked()
{
    JustinaRepresentation::factCLIPS(true);
}

void MainWindow::on_rulesCLIPS_clicked()
{
    JustinaRepresentation::ruleCLIPS(true);
}

void MainWindow::on_agendaCLIPS_clicked()
{
    JustinaRepresentation::agendaCLIPS(true);
}

void MainWindow::enterCommandChanged(){
    std::cout << "QMainWindow.->enterCommand:" << this->ui->enterCommand->text().toStdString() << std::endl;
    JustinaRepresentation::sendCLIPS(this->ui->enterCommand->text().toStdString());
}

void MainWindow::on_openFileCommand_clicked()
{
    std::string path;
    path = ros::package::getPath("knowledge_representation");
    std::cout << path << std::endl;
    QString pathFile = QFileDialog::getOpenFileName(
          this,
          tr("Open File"),
          QString::fromStdString(path),
          "Text File (*.dat)"
          );
    std::cout << "QMainWindow.->pathFile:" << pathFile.toStdString() << std::endl;
    this->ui->loadCommand->setText( pathFile);
}

void MainWindow::setPathKR()
{
    std::string path;
    path = ros::package::getPath("knowledge_representation");
    std::cout << path << std::endl;
    std::stringstream ss;
    ss << path << "/scripts/virbot_gpsr/speechTest.dat";
    this->ui->loadCommand->setText(QString::fromStdString(ss.str()));
}

void MainWindow::loadCommandChanged()
{
    std::cout << "QMainWindow.->loadCommand:" << this->ui->loadCommand->text().toStdString() << std::endl;
    JustinaRepresentation::loadCLIPS(this->ui->loadCommand->text().toStdString());
}

void MainWindow::setlocClips()
{
    std::string path;
    path = ros::package::getPath("knowledge_representation");
    std::cout << path << std::endl;
    std::stringstream ss;
    ss << path << "/scripts/base_data/Locations.txt";

    //this->ui->locCLIPStab->setRowCount(0);
    //obtain the information of the file Locations.txt
    //std::map<std::string, std::vector<std::string> > loc;
    JustinaRepresentation::getLocations(ss.str(), locations);
    //objects = loc;
    //int row = 0;
    //std::map<std::string, std::vector<std::string> >::iterator it = locations.begin();
    this->ui->locCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it = locations.begin(); it != locations.end(); ++it){
      this->ui->locCLIPStab->insertRow(this->ui->locCLIPStab->rowCount());
      float row = this->ui->locCLIPStab->rowCount() - 1;
      this->ui->locCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it->second[0])));
      this->ui->locCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it->first)));
      this->ui->locCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it->second[1])));
      this->ui->locCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it->second[2])));
    }
    this->ui->locCLIPStab->resizeRowsToContents();
    this->ui->locCLIPStab->resizeColumnsToContents();
    this->ui->locCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->locCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->locCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);

    /*---------------------------------------------------------------------------------------*/
    ss.str("");
    ss << path << "/scripts/base_data/Objects.txt";

    JustinaRepresentation::getObjects(ss.str(), objects);
    objects = objects;

    //row=0;
    //std::map<std::string, std::vector<std::string> >::iterator it2 = objects.begin();
    this->ui->objCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it2 = objects.begin(); it2 != objects.end(); it2++){
      this->ui->objCLIPStab->insertRow(this->ui->objCLIPStab->rowCount());
      float row = this->ui->objCLIPStab->rowCount() - 1;
      this->ui->objCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it2->first)));
      this->ui->objCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it2->second[0])));
      this->ui->objCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it2->second[1])));
      this->ui->objCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it2->second[2])));
      this->ui->objCLIPStab->setItem(row, C1, new QTableWidgetItem(QString::fromStdString(it2->second[3])));
      this->ui->objCLIPStab->setItem(row, C2, new QTableWidgetItem(QString::fromStdString(it2->second[4])));
      this->ui->objCLIPStab->setItem(row, C3, new QTableWidgetItem(QString::fromStdString(it2->second[5])));
      this->ui->objCLIPStab->setItem(row, C4, new QTableWidgetItem(QString::fromStdString(it2->second[6])));

      this->ui->objCLIPStab->setItem(row, C5, new QTableWidgetItem(QString::fromStdString(it2->second[7])));
      this->ui->objCLIPStab->setItem(row, C6, new QTableWidgetItem(QString::fromStdString(it2->second[8])));
      this->ui->objCLIPStab->setItem(row, C7, new QTableWidgetItem(QString::fromStdString(it2->second[9])));
      this->ui->objCLIPStab->setItem(row, C8, new QTableWidgetItem(QString::fromStdString(it2->second[10])));
    }
    this->ui->objCLIPStab->resizeRowsToContents();
    this->ui->objCLIPStab->resizeColumnsToContents();
    this->ui->objCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->objCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->objCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);

    /*-----------------------------------------------------------------------------------------------------*/
    ss.str("");
    ss<<path<<"/scripts/base_data/People.txt";
    JustinaRepresentation::getPeoples(ss.str(),peoples);
    peoples=peoples;
    //row=0;
    //std::map<std::string, std::vector<std::string> >::iterator it4 = peoples.begin();
    std::string name;
    std::string age;
    std::string gender;
    
    this->ui->peopleCLIPStab->setRowCount(0);
    for(std::map<std::string , std::vector<std::string> >::iterator itp = peoples.begin() ; itp != peoples.end() ; itp++){
      this->ui->peopleCLIPStab->insertRow(this->ui->peopleCLIPStab->rowCount());
      float row = this->ui->peopleCLIPStab->rowCount()-1;
      name=JustinaRepresentation::covertLetters(itp->first);
      age=JustinaRepresentation::covertLetters(itp->second[0]);
      gender=JustinaRepresentation::covertLetters(itp->second[1]);
      //std::cout<<" Palabra Convertida: "<< name <<std::endl;
      this->ui->peopleCLIPStab->setItem(row,NAME, new QTableWidgetItem(QString::fromStdString(name)));
      this->ui->peopleCLIPStab->setItem(row,X, new QTableWidgetItem(QString::fromStdString(age)));   
      this->ui->peopleCLIPStab->setItem(row,Y, new QTableWidgetItem(QString::fromStdString(gender)));  
    }
    this->ui->peopleCLIPStab->resizeRowsToContents();
    this->ui->peopleCLIPStab->resizeColumnsToContents();
    this->ui->peopleCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->peopleCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->peopleCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);

    /*---------------------------------------------------------------------------------------*/
    ss.str("");
    ss<<path<<"/scripts/base_data/Categorys.txt";
    JustinaRepresentation::getCategorys(ss.str(),categorys);
    categorys=categorys;
    //row=0;
    this->ui->categoryCLIPStab->setRowCount(0);
    for(std::map<std::string , std::vector<std::string> >::iterator itc = categorys.begin() ; itc != categorys.end() ; itc++){
         this->ui->categoryCLIPStab->insertRow(this->ui->categoryCLIPStab->rowCount());
         float row = this->ui->categoryCLIPStab->rowCount()-1;
         this->ui->categoryCLIPStab->setItem( row, NAME , new QTableWidgetItem(QString::fromStdString(itc->first)));
         this->ui->categoryCLIPStab->setItem( row, X , new QTableWidgetItem(QString::fromStdString(itc->second[0])));
         this->ui->categoryCLIPStab->setItem( row, Y , new QTableWidgetItem(QString::fromStdString(itc->second[1])));
         this->ui->categoryCLIPStab->setItem( row, A , new QTableWidgetItem(QString::fromStdString(itc->second[2])));
         this->ui->categoryCLIPStab->setItem( row, C1 , new QTableWidgetItem(QString::fromStdString(itc->second[3])));
         this->ui->categoryCLIPStab->setItem( row, C2 , new QTableWidgetItem(QString::fromStdString(itc->second[4])));
         this->ui->categoryCLIPStab->setItem( row, C3 , new QTableWidgetItem(QString::fromStdString(itc->second[5])));
     }

    this->ui->categoryCLIPStab->resizeRowsToContents();
    this->ui->categoryCLIPStab->resizeColumnsToContents();
    this->ui->categoryCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->categoryCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->categoryCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);


}

void MainWindow::on_addCLIPSloc_clicked()
{
    std::cout << "QMainWindow.->on_addCLIPSLoc_clicked:" << std::endl;
    //std::cout << "QMainWindow.->name:" << this->ui->nameCLIPSloc->text().toStdString() << std::endl;
    //std::cout << "QMainWindow.->type:" << this->ui->typeCLIPSloc->text().toStdString() << std::endl;
    //std::cout << "QMainWindow.->quantity:" << this->ui->quantCLIPSloc->text().toStdString() << std::endl;
    //std::cout << "QMainWindow.->room:" << this->ui->roomCLIPSloc->text().toStdString() << std::endl;

    //for(std::map<std::string, std::vector<std::string> >::iterator it = locations.begin(); it != locations.end(); it++){
    //    std::cout << "first:" << it->first << std::endl;
    //}
    std::string name = this->ui->nameCLIPSloc->text().toStdString();
    std::vector<std::string> values;
    values.push_back(this->ui->typeCLIPSloc->text().toStdString());
    values.push_back(this->ui->quantCLIPSloc->text().toStdString());
    values.push_back(this->ui->roomCLIPSloc->text().toStdString());

    JustinaRepresentation::addLocations(locations,name,values);

    //for(std::map<std::string, std::vector<std::string> >::iterator it2 = locations.begin(); it2 != locations.end(); it2++){
    //    std::cout << "SECOND:" << it2->first << std::endl;
    //}

    this->ui->locCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it = locations.begin(); it != locations.end(); ++it){
      this->ui->locCLIPStab->insertRow(this->ui->locCLIPStab->rowCount());
      float row = this->ui->locCLIPStab->rowCount() - 1;
      this->ui->locCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it->second[0])));
      this->ui->locCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it->first)));
      this->ui->locCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it->second[1])));
      this->ui->locCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it->second[2])));
    }
    this->ui->locCLIPStab->resizeRowsToContents();
    this->ui->locCLIPStab->resizeColumnsToContents();
    this->ui->locCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->locCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->locCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void MainWindow::on_addCLIPSobj_clicked(){
    std::cout << "QMainWindow.->on_addCLIPSLoc_clicked:" << std::endl;
    std::cout << "QMainWindow.->name:" << this->ui->nameCLIPSobj->text().toStdString() << std::endl;
    std::cout << "QMainWindow.->cat:" << this->ui->catCLIPSobj->text().toStdString() << std::endl;
    std::cout << "QMainWindow.->location:" << this->ui->locCLIPSobj->text().toStdString() << std::endl;
    std::cout << "QMainWindow.->room:" << this->ui->roomCLIPSobj->text().toStdString() << std::endl;
    std::cout << "QMainWindow.->weight:" << this->ui->weightCLIPSobj->text().toStdString() << std::endl;
    std::cout << "QMainWindow.->size:" << this->ui->sizeCLIPSobj->text().toStdString() << std::endl;
    std::cout << "QMainWindow.->color:" << this->ui->colorCLIPSobj->text().toStdString() << std::endl;
    std::cout << "QMainWindow.->quant:" << this->ui->graspCLIPSobj->text().toStdString() << std::endl;

    //for(std::map<std::string, std::vector<std::string> >::iterator it = objects.begin(); it != objects.end(); it++){
    //    std::cout << "first:" << it->first << std::endl;
    //}
    std::string name = this->ui->nameCLIPSobj->text().toStdString();
    std::vector<std::string> values;
    values.push_back(this->ui->catCLIPSobj->text().toStdString());
    values.push_back(this->ui->locCLIPSobj->text().toStdString());
    values.push_back(this->ui->roomCLIPSobj->text().toStdString());
    values.push_back(this->ui->weightCLIPSobj->text().toStdString());
    values.push_back(this->ui->sizeCLIPSobj->text().toStdString());
    values.push_back(this->ui->colorCLIPSobj->text().toStdString());
    values.push_back(this->ui->graspCLIPSobj->text().toStdString());

    JustinaRepresentation::addObjects(objects,name,values);

    //for(std::map<std::string, std::vector<std::string> >::iterator it2 = objects.begin(); it2 != objects.end(); it2++){
    //    std::cout << "SECOND:" << it2->first << std::endl;
    //}

    this->ui->objCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it2 = objects.begin(); it2 != objects.end(); it2++){
      this->ui->objCLIPStab->insertRow(this->ui->objCLIPStab->rowCount());
      float row = this->ui->objCLIPStab->rowCount() - 1;
      this->ui->objCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it2->first)));
      this->ui->objCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it2->second[0])));
      this->ui->objCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it2->second[1])));
      this->ui->objCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it2->second[2])));
      this->ui->objCLIPStab->setItem(row, C1, new QTableWidgetItem(QString::fromStdString(it2->second[3])));
      this->ui->objCLIPStab->setItem(row, C2, new QTableWidgetItem(QString::fromStdString(it2->second[4])));
      this->ui->objCLIPStab->setItem(row, C3, new QTableWidgetItem(QString::fromStdString(it2->second[5])));
      this->ui->objCLIPStab->setItem(row, C4, new QTableWidgetItem(QString::fromStdString(it2->second[6])));

      /*this->ui->objCLIPStab->setItem(row, C5, new QTableWidgetItem(QString::fromStdString(it2->second[7])));
      this->ui->objCLIPStab->setItem(row, C6, new QTableWidgetItem(QString::fromStdString(it2->second[8])));
      this->ui->objCLIPStab->setItem(row, C7, new QTableWidgetItem(QString::fromStdString(it2->second[9])));
      this->ui->objCLIPStab->setItem(row, C8, new QTableWidgetItem(QString::fromStdString(it2->second[10])));*/
    }
    this->ui->objCLIPStab->resizeRowsToContents();
    this->ui->objCLIPStab->resizeColumnsToContents();
    this->ui->objCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->objCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->objCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);

}

void MainWindow::on_addCLIPSpeoples_clicked(){

    std::cout<<"\nName"<<this->ui->nameCLIPSpeople->text().toStdString() <<std::endl;
    std::cout<<"\tAge"<<this->ui->AgeCLIPSpeople->text().toStdString() <<std::endl;
    std::cout<<"\tGender"<<this->ui->GenderCLIPSpeople->text().toStdString()<<std::endl;

    std::string name=this->ui->nameCLIPSpeople->text().toStdString();
    std::vector<std::string> values;

    values.push_back(this->ui->AgeCLIPSpeople->text().toStdString());
    values.push_back(this->ui->GenderCLIPSpeople->text().toStdString());

    JustinaRepresentation::addPeoples(peoples,name,values);
   
    this->ui->peopleCLIPStab->setRowCount(0);
    for(std::map<std::string , std::vector<std::string> >::iterator itp = peoples.begin() ; itp != peoples.end() ; itp++){
      this->ui->peopleCLIPStab->insertRow(this->ui->peopleCLIPStab->rowCount());
      float row = this->ui->peopleCLIPStab->rowCount()-1;
     std::string  name=JustinaRepresentation::covertLetters(itp->first);
     std::string  age=JustinaRepresentation::covertLetters(itp->second[0]);
     std::string  gender=JustinaRepresentation::covertLetters(itp->second[1]);
      //std::cout<<" Palabra Convertida: "<< name <<std::endl;
      this->ui->peopleCLIPStab->setItem(row,NAME, new QTableWidgetItem(QString::fromStdString(name)));
      this->ui->peopleCLIPStab->setItem(row,X, new QTableWidgetItem(QString::fromStdString(age)));   
      this->ui->peopleCLIPStab->setItem(row,Y, new QTableWidgetItem(QString::fromStdString(gender)));  
    }
    this->ui->peopleCLIPStab->resizeRowsToContents();
    this->ui->peopleCLIPStab->resizeColumnsToContents();
    this->ui->peopleCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->peopleCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->peopleCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void MainWindow::on_addCLIPScategory_clicked(){
     std::cout<<"\n Name: "<<this->ui->nameCLIPScat->text().toStdString() <<std::endl;
     std::cout<<"\t Zone: "<<this->ui->zoneCLIPScat->text().toStdString() <<std::endl;
     std::cout<<"\t Quantity: "<<this->ui->quantityCLIPScat->text().toStdString() <<std::endl;
     std::cout<<"\t Biggest: "<<this->ui->biggetsCLIPScat->text().toStdString() <<std::endl;
     std::cout<<"\t Smallets: "<<this->ui->smallestCLIPScat->text().toStdString() <<std::endl;
     std::cout<<"\t Heaviets: "<<this->ui->heavietsCLIPScat->text().toStdString() <<std::endl;
     std::cout<<"\t Lightets: "<<this->ui->lightestCLIPScat->text().toStdString() <<std::endl;

     std::string name= this->ui->nameCLIPScat->text().toStdString();
     std::vector<std::string> values;
     values.push_back(this->ui->zoneCLIPScat->text().toStdString());
     values.push_back(this->ui->quantityCLIPScat->text().toStdString());
     values.push_back(this->ui->biggetsCLIPScat->text().toStdString());
     values.push_back(this->ui->smallestCLIPScat->text().toStdString());
     values.push_back(this->ui->heavietsCLIPScat->text().toStdString());
     values.push_back(this->ui->lightestCLIPScat->text().toStdString());

     JustinaRepresentation::addCategorys(categorys , name , values);
     
    this->ui->categoryCLIPStab->setRowCount(0);
    for(std::map<std::string , std::vector<std::string> >::iterator itc = categorys.begin() ; itc != categorys.end() ; itc++){
         this->ui->categoryCLIPStab->insertRow(this->ui->categoryCLIPStab->rowCount());
         float row = this->ui->categoryCLIPStab->rowCount()-1;
         this->ui->categoryCLIPStab->setItem( row, NAME , new QTableWidgetItem(QString::fromStdString(itc->first)));
         this->ui->categoryCLIPStab->setItem( row, X , new QTableWidgetItem(QString::fromStdString(itc->second[0])));
         this->ui->categoryCLIPStab->setItem( row, Y , new QTableWidgetItem(QString::fromStdString(itc->second[1])));
         this->ui->categoryCLIPStab->setItem( row, A , new QTableWidgetItem(QString::fromStdString(itc->second[2])));
         this->ui->categoryCLIPStab->setItem( row, C1 , new QTableWidgetItem(QString::fromStdString(itc->second[3])));
         this->ui->categoryCLIPStab->setItem( row, C2 , new QTableWidgetItem(QString::fromStdString(itc->second[4])));
         this->ui->categoryCLIPStab->setItem( row, C3 , new QTableWidgetItem(QString::fromStdString(itc->second[5])));
     }

    this->ui->categoryCLIPStab->resizeRowsToContents();
    this->ui->categoryCLIPStab->resizeColumnsToContents();
    this->ui->categoryCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->categoryCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->categoryCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
}


void MainWindow::on_saveCLIPSloc_clicked(){
    std::cout<<"\nFunction saved Locations :"<<std::endl;
    std::string path= ros::package::getPath("knowledge_representation");
    std::string directory="/scripts/base_data/Locations_Data_Base/";
    QString extension=".txt";
    //std::cout<<"\n Directory"<< path+directory<< std::endl;
    std::string folderPath=path+directory;
    QString fileName = QFileDialog::getSaveFileName(this ,tr("Saving file of Locations"),
                                 QString::fromStdString(folderPath),tr("Text files(*.txt)"));
    QString concatenation=fileName+extension;
   
    std::string my_path=fileName.toStdString();
    std::string filePath = concatenation.toStdString();

    if(my_path == ""){
       std::cout<<"\n does not contain file \n";
    }else{
        std::cout<<"\n if it contains file \n";
        qDebug()<<"Directory: "<<concatenation;
        std::ofstream file(filePath.c_str());

        if(file.fail()){
            std::cout<<"\n Fail File "<<std::endl;
        }else{
            std::cout<<"File ready to save data "<<std::endl;
        }
        for(std::map<std::string , std::vector<std::string> >::iterator it = locations.begin() ; it != locations.end(); it++ ){
         /* std::cout<<"\nType: "<<it->second[0];
         std::cout<<"\nName: "<<it->first;
         std::cout<<"\nQuantity: "<<it->second[1];
         std::cout<<"\nRoom: "<<it->second[2];*/
         file<<it->second[0]<<" "<<it->first<<" "<<it->second[1]<<" "<<it->second[2]<<"\n";;
        }
        std::cout<<"\n  File Saved "<<std::endl;
        file.close();
     }   
}

void MainWindow::on_saveCLIPSobj_clicked(){
    std::cout<<"\nFunction saved Objects :"<<std::endl;
    std::string path= ros::package::getPath("knowledge_representation");
    std::string directory="/scripts/base_data/Objects_Data_Base/";
    QString extension=".txt";
    //std::cout<<"\n Directory"<< path+directory<< std::endl;
    std::string folderPath=path+directory;
    QString fileName = QFileDialog::getSaveFileName(this ,tr("Saving file of Objects"),
                                 QString::fromStdString(folderPath),tr("Text files(*.txt)"));
    //qDebug()<<"file Name: "<<fileName;//

    QString concatenation=fileName+extension;
    std::string my_path=fileName.toStdString();
    std::string filePath = concatenation.toStdString();
    if(my_path == ""){
       std::cout<<"\n does not contain file \n";
    }else{
            std::cout<<"\n if it contains file \n";
            qDebug()<<"Directory: "<<concatenation;
            std::ofstream file(filePath.c_str());
            if(file.fail()){
                std::cout<<"\n Fail File "<<std::endl;
            }else{
                std::cout<<"File ready to save data "<<std::endl;
            }
            for(std::map<std::string , std::vector<std::string> >::iterator it = objects.begin() ; it != objects.end(); it++ ){
              /*  std::cout<<"\n Name: "<<it->first;
                std::cout<<"\t Category: "<<it->second[0];
                std::cout<<"\t Loaction: "<<it->second[1];
                std::cout<<"\t Room: "<<it->second[2];
                std::cout<<"\t Weight: "<<it->second[3];
                std::cout<<"\t Size: "<<it->second[4];
                std::cout<<"\t Color: "<<it->second[5];
                std::cout<<"\t Quantity: "<<it->second[6];*/
                file<<it->first<<" "<<it->second[0]<<" "<<it->second[1]<<" "<<it->second[2]
                    <<" "<<it->second[3]<<" "<<it->second[4]<<" "<<it->second[5]<<" "<<it->second[6]<<"\n";
            }
            std::cout<<"\n  File Saved "<<std::endl;
            file.close();
     }       
}

void MainWindow::on_saveCLIPSpeoples_clicked(){
    std::cout<<"\nFunction saved Peoples :"<<std::endl;
    //bool action;
    std::string path= ros::package::getPath("knowledge_representation");
    std::string directory="/scripts/base_data/Peoples_Data_Base/";
    QString extension=".txt";
    //std::cout<<"\n Directory:   "<< path+directory<< std::endl;
    std::string folderPath=path+directory;
    QString fileName = QFileDialog::getSaveFileName(this ,tr("Saving file of Peoples"),
                                 QString::fromStdString(folderPath),tr("Text files(*.txt)"));
    QString concatenation=fileName+extension;
    //qDebug()<<" Ruta Obtenida:  "<<fileName;
    //qDebug()<<" Directory : "<<concatenation;
    std::string my_path=fileName.toStdString();
    std::string filePath = concatenation.toStdString();

    ///action=JustinaRepresentation::evaluateRoute(filePath);
    if(my_path == ""){
       std::cout<<"\n does not contain file \n";
    }else{
       std::cout<<"\n if it contains file \n";
       qDebug()<<"Directory: "<<concatenation;
       std::ofstream file(filePath.c_str());
       if(file.fail()){
        std::cout<<"\n Fail File "<<std::endl;
       }else{
        std::cout<<"File ready to save data "<<std::endl;
       }

        for(std::map<std::string , std::vector<std::string> >::iterator it = peoples.begin() ; it != peoples.end(); it++ ){
        /*std::cout<<"\n Name: "<<it->first;
        std::cout<<"\t Age: "<<it->second[0];
        std::cout<<"\t Gender: "<<it->second[1]<< std::endl;*/
            file<<it->first<<" "<<it->second[0]<<" "<<it->second[1]<<"\n";
        }

        std::cout<<"\n  File Saved "<<std::endl;
        file.close();
    }

}

void MainWindow::on_saveCLIPScategory_clicked(){
    std::cout<<"\nFunction saved Categorys :"<<std::endl;
    std::string path= ros::package::getPath("knowledge_representation");
    std::string directory="/scripts/base_data/Categorys_Data_Base/";
    QString extension=".txt";
    //std::cout<<"\n Directory"<< path+directory<< std::endl;
    std::string folderPath=path+directory;
    QString fileName = QFileDialog::getSaveFileName(this ,tr("Saving file of Categorys"),
                                 QString::fromStdString(folderPath),tr("Text files(*.txt)"));
    QString concatenation=fileName+extension;
    //qDebug()<<"Directory: "<<concatenation;
    std::string my_path=fileName.toStdString();
    std::string filePath = concatenation.toStdString();

    if(my_path == ""){
       std::cout<<"\n does not contain file \n";
    }else{
         std::cout<<"\n if it contains file \n";
         qDebug()<<"Directory: "<<concatenation;

        std::ofstream file(filePath.c_str());
            
        if(file.fail()){
            std::cout<<"\n Fail File "<<std::endl;
        }else{
             std::cout<<"File ready to save data "<<std::endl;   
        }
        for(std::map<std::string , std::vector<std::string> >::iterator it = categorys.begin() ; it != categorys.end(); it++ ){
               /* std::cout<<"\n Name: "<<it->first;
                std::cout<<"\t Zone: "<<it->second[0];
                std::cout<<"\t Quantity: "<<it->second[1]<< std::endl;
                std::cout<<"\t Biggets: "<<it->second[2];
                std::cout<<"\t Smallets: "<<it->second[3]<< std::endl;
                std::cout<<"\t Heaviets: "<<it->second[4];
                std::cout<<"\t Lightest: "<<it->second[5]<< std::endl;*/
            file<<it->first<<" "<<it->second[0]<<" "<<it->second[1]<<" "<<it->second[2]<<" "<<it->second[3]<<" "<<it->second[4]<<" "<<it->second[5]<<"\n";
        }

        std::cout<<"\n  File Saved "<<std::endl;
        file.close();
        }       
}

void MainWindow::on_loadCLIPSloc_clicked(){
    std::cout<<"Function Load Locations :"<<std::endl;
    std::string path= ros::package::getPath("knowledge_representation");
   std::string directory="/scripts/base_data/Locations_Data_Base/";
   std::string folderPath=path+directory;
   std::cout<<"\n Directory that contains"<< folderPath<< std::endl;
   QString fileName = QFileDialog::getOpenFileName(this , tr(" Open File "),QString::fromStdString(folderPath),tr("Text files(*.txt)"));
   qDebug()<<"\n Full Route: "<<fileName;


   std::stringstream ss;
   ss << fileName.toStdString();
   JustinaRepresentation::getLocations(ss.str(),locations);
   std::cout<<"\n Get Locations: "<<std::endl;
   locations=locations;
   
   this->ui->locCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it = locations.begin(); it != locations.end(); ++it){
      this->ui->locCLIPStab->insertRow(this->ui->locCLIPStab->rowCount());
      float row = this->ui->locCLIPStab->rowCount() - 1;
      this->ui->locCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it->second[0])));
      this->ui->locCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it->first)));
      this->ui->locCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it->second[1])));
      this->ui->locCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it->second[2])));
    }
    this->ui->locCLIPStab->resizeRowsToContents();
    this->ui->locCLIPStab->resizeColumnsToContents();
    this->ui->locCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->locCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->locCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
}
    
void MainWindow::on_loadCLIPSobj_clicked(){
   std::cout<<"Function Load Objects :"<<std::endl;
   std::cout<<"Function Load Locations :"<<std::endl;
   std::string path= ros::package::getPath("knowledge_representation");
   std::string directory="/scripts/base_data/Objects_Data_Base/";
   std::string folderPath=path+directory;
   std::cout<<"\n Directory that contains"<< folderPath<< std::endl;
   QString fileName = QFileDialog::getOpenFileName(this , tr(" Open File "),QString::fromStdString(folderPath),tr("Text files(*.txt)"));
   qDebug()<<"\n Full Route: "<<fileName;

   std::stringstream ss;
   ss << fileName.toStdString();
   JustinaRepresentation::getObjects(ss.str(),objects);
   std::cout<<"\n Get Objects: "<<std::endl;
   objects=objects;
  
   this->ui->objCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it2 = objects.begin(); it2 != objects.end(); it2++){
      this->ui->objCLIPStab->insertRow(this->ui->objCLIPStab->rowCount());
      float row = this->ui->objCLIPStab->rowCount() - 1;
      this->ui->objCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it2->first)));
      this->ui->objCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it2->second[0])));
      this->ui->objCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it2->second[1])));
      this->ui->objCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it2->second[2])));
      this->ui->objCLIPStab->setItem(row, C1, new QTableWidgetItem(QString::fromStdString(it2->second[3])));
      this->ui->objCLIPStab->setItem(row, C2, new QTableWidgetItem(QString::fromStdString(it2->second[4])));
      this->ui->objCLIPStab->setItem(row, C3, new QTableWidgetItem(QString::fromStdString(it2->second[5])));
      this->ui->objCLIPStab->setItem(row, C4, new QTableWidgetItem(QString::fromStdString(it2->second[6])));

      this->ui->objCLIPStab->setItem(row, C5, new QTableWidgetItem(QString::fromStdString(it2->second[7])));
      this->ui->objCLIPStab->setItem(row, C6, new QTableWidgetItem(QString::fromStdString(it2->second[8])));
      this->ui->objCLIPStab->setItem(row, C7, new QTableWidgetItem(QString::fromStdString(it2->second[9])));
      this->ui->objCLIPStab->setItem(row, C8, new QTableWidgetItem(QString::fromStdString(it2->second[10])));
    }
   this->ui->objCLIPStab->resizeRowsToContents();
   this->ui->objCLIPStab->resizeColumnsToContents();
   this->ui->objCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
   this->ui->objCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
   this->ui->objCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);


}

void MainWindow::on_loadCLIPSpeoples_clicked(){
   std::cout<<"Function Load Peoples :"<<std::endl;
   std::string path= ros::package::getPath("knowledge_representation");
   std::string directory="/scripts/base_data/Peoples_Data_Base/";
   std::string folderPath=path+directory;
   std::cout<<"\n Directory that contains"<< folderPath<< std::endl;
   QString fileName = QFileDialog::getOpenFileName(this , tr(" Open File "),QString::fromStdString(folderPath),tr("Text files(*.txt)"));
   qDebug()<<"\n Full Route: "<<fileName;

   std::stringstream ss;
   ss << fileName.toStdString();
   JustinaRepresentation::getPeoples(ss.str(),peoples);
   std::cout<<"\n Get Peoples:"<<std::endl;
   peoples=peoples;
  
   this->ui->peopleCLIPStab->setRowCount(0);
    for(std::map<std::string , std::vector<std::string> >::iterator itp = peoples.begin() ; itp != peoples.end() ; itp++){
      this->ui->peopleCLIPStab->insertRow(this->ui->peopleCLIPStab->rowCount());
      float row = this->ui->peopleCLIPStab->rowCount()-1;
      std::string  name=JustinaRepresentation::covertLetters(itp->first);
      std::string  age=JustinaRepresentation::covertLetters(itp->second[0]);
      std::string  gender=JustinaRepresentation::covertLetters(itp->second[1]);
      //std::cout<<" Palabra Convertida: "<< name <<std::endl;
      this->ui->peopleCLIPStab->setItem(row,NAME, new QTableWidgetItem(QString::fromStdString(name)));
      this->ui->peopleCLIPStab->setItem(row,X, new QTableWidgetItem(QString::fromStdString(age)));   
      this->ui->peopleCLIPStab->setItem(row,Y, new QTableWidgetItem(QString::fromStdString(gender)));  
    }
    this->ui->peopleCLIPStab->resizeRowsToContents();
    this->ui->peopleCLIPStab->resizeColumnsToContents();
    this->ui->peopleCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->peopleCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->peopleCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);

}

void MainWindow::on_loadCLIPScategory_clicked(){
   std::cout<<"Function Load Categorys :"<<std::endl;
   std::string path= ros::package::getPath("knowledge_representation");
   std::string directory="/scripts/base_data/Categorys_Data_Base/";
   std::string folderPath=path+directory;
   std::cout<<"\n Directory that contains"<< folderPath<< std::endl;
   QString fileName = QFileDialog::getOpenFileName(this , tr(" Open File "),QString::fromStdString(folderPath),tr("Text files(*.txt)"));
   qDebug()<<"\n Full Route: "<<fileName;

   std::stringstream ss;
   ss << fileName.toStdString();
   JustinaRepresentation::getCategorys(ss.str(),categorys);
   std::cout<<"\n Get Categorys:"<<std::endl;
   categorys=categorys;

   this->ui->categoryCLIPStab->setRowCount(0);
    for(std::map<std::string , std::vector<std::string> >::iterator itc = categorys.begin() ; itc != categorys.end() ; itc++){
         this->ui->categoryCLIPStab->insertRow(this->ui->categoryCLIPStab->rowCount());
         float row = this->ui->categoryCLIPStab->rowCount()-1;
         this->ui->categoryCLIPStab->setItem( row, NAME , new QTableWidgetItem(QString::fromStdString(itc->first)));
         this->ui->categoryCLIPStab->setItem( row, X , new QTableWidgetItem(QString::fromStdString(itc->second[0])));
         this->ui->categoryCLIPStab->setItem( row, Y , new QTableWidgetItem(QString::fromStdString(itc->second[1])));
         this->ui->categoryCLIPStab->setItem( row, A , new QTableWidgetItem(QString::fromStdString(itc->second[2])));
         this->ui->categoryCLIPStab->setItem( row, C1 , new QTableWidgetItem(QString::fromStdString(itc->second[3])));
         this->ui->categoryCLIPStab->setItem( row, C2 , new QTableWidgetItem(QString::fromStdString(itc->second[4])));
         this->ui->categoryCLIPStab->setItem( row, C3 , new QTableWidgetItem(QString::fromStdString(itc->second[5])));
     }

    this->ui->categoryCLIPStab->resizeRowsToContents();
    this->ui->categoryCLIPStab->resizeColumnsToContents();
    this->ui->categoryCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->categoryCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->categoryCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
   
}

void MainWindow::on_deleteCLIPSobj_clicked(){
    std::cout<<"Delete object CLIPS :"<<std::endl;
    std::string name =this->ui->nameCLIPSobj->text().toStdString();
    std::cout<<"\n Se eliminara: "<<name<<std::endl;
    std::map<std::string , std::vector<std::string> >::iterator itx=objects.find(name);
    if(itx != objects.end()){
        //std::cout<<"Si esta contenido en el map "<<std::endl;
        JustinaRepresentation::deleteObjects(objects,name);
        objects=objects;
        
      this->ui->objCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it2 = objects.begin(); it2 != objects.end(); it2++){
          this->ui->objCLIPStab->insertRow(this->ui->objCLIPStab->rowCount());
          float row = this->ui->objCLIPStab->rowCount() - 1;
          this->ui->objCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it2->first)));
          this->ui->objCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it2->second[0])));
          this->ui->objCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it2->second[1])));
          this->ui->objCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it2->second[2])));
          this->ui->objCLIPStab->setItem(row, C1, new QTableWidgetItem(QString::fromStdString(it2->second[3])));
          this->ui->objCLIPStab->setItem(row, C2, new QTableWidgetItem(QString::fromStdString(it2->second[4])));
          this->ui->objCLIPStab->setItem(row, C3, new QTableWidgetItem(QString::fromStdString(it2->second[5])));
          this->ui->objCLIPStab->setItem(row, C4, new QTableWidgetItem(QString::fromStdString(it2->second[6])));

      /*this->ui->objCLIPStab->setItem(row, C5, new QTableWidgetItem(QString::fromStdString(it2->second[7])));
      this->ui->objCLIPStab->setItem(row, C6, new QTableWidgetItem(QString::fromStdString(it2->second[8])));
      this->ui->objCLIPStab->setItem(row, C7, new QTableWidgetItem(QString::fromStdString(it2->second[9])));
      this->ui->objCLIPStab->setItem(row, C8, new QTableWidgetItem(QString::fromStdString(it2->second[10])));*/
    }
    this->ui->objCLIPStab->resizeRowsToContents();
    this->ui->objCLIPStab->resizeColumnsToContents();
    this->ui->objCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->objCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->objCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);

    }
}

void MainWindow::on_deleteCLIPSloc_clicked(){
   std::cout<<"Delete location CLIPS :"<<std::endl;
   std::string name =this->ui->nameCLIPSloc->text().toStdString();
    std::cout<<"\n Se eliminara: "<<name<<std::endl;
    std::map<std::string , std::vector<std::string> >::iterator itx=locations.find(name);
    if(itx != locations.end()){
        //std::cout<<"Si esta contenido en el map "<<std::endl;
        JustinaRepresentation::deleteLocations(locations,name);
        locations=locations;
        
    this->ui->locCLIPStab->setRowCount(0);
    for(std::map<std::string, std::vector<std::string> >::iterator it = locations.begin(); it != locations.end(); ++it){
      this->ui->locCLIPStab->insertRow(this->ui->locCLIPStab->rowCount());
      float row = this->ui->locCLIPStab->rowCount() - 1;
      this->ui->locCLIPStab->setItem(row, NAME, new QTableWidgetItem(QString::fromStdString(it->second[0])));
      this->ui->locCLIPStab->setItem(row, X, new QTableWidgetItem(QString::fromStdString(it->first)));
      this->ui->locCLIPStab->setItem(row, Y, new QTableWidgetItem(QString::fromStdString(it->second[1])));
      this->ui->locCLIPStab->setItem(row, A, new QTableWidgetItem(QString::fromStdString(it->second[2])));
    }
    this->ui->locCLIPStab->resizeRowsToContents();
    this->ui->locCLIPStab->resizeColumnsToContents();
    this->ui->locCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->locCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->locCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
    }
}

void MainWindow::on_deleteCLIPSpeoples_clicked(){
    std::cout<<"Delete people CLIPS :"<<std::endl;
    std::string name =this->ui->nameCLIPSpeople->text().toStdString();
    std::cout<<"\n Se eliminara: "<<name<<std::endl;
    std::map<std::string , std::vector<std::string> >::iterator itx=peoples.find(name);
    if(itx != peoples.end()){
        //std::cout<<"Si esta contenido en el map "<<std::endl;
        JustinaRepresentation::deletePeoples(peoples,name);
        peoples=peoples;
    this->ui->peopleCLIPStab->setRowCount(0);
    for(std::map<std::string , std::vector<std::string> >::iterator itp = peoples.begin() ; itp != peoples.end() ; itp++){
      this->ui->peopleCLIPStab->insertRow(this->ui->peopleCLIPStab->rowCount());
      float row = this->ui->peopleCLIPStab->rowCount()-1;
      std::string name=JustinaRepresentation::covertLetters(itp->first);
      std::string  age=JustinaRepresentation::covertLetters(itp->second[0]);
      std::string  gender=JustinaRepresentation::covertLetters(itp->second[1]);
      //std::cout<<" Palabra Convertida: "<< name <<std::endl;
      this->ui->peopleCLIPStab->setItem(row,NAME, new QTableWidgetItem(QString::fromStdString(name)));
      this->ui->peopleCLIPStab->setItem(row,X, new QTableWidgetItem(QString::fromStdString(age)));   
      this->ui->peopleCLIPStab->setItem(row,Y, new QTableWidgetItem(QString::fromStdString(gender)));  
    }
    this->ui->peopleCLIPStab->resizeRowsToContents();
    this->ui->peopleCLIPStab->resizeColumnsToContents();
    this->ui->peopleCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
    this->ui->peopleCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
    this->ui->peopleCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
    
    }
}

void MainWindow::on_deleteCLIPScategory_clicked(){
  std::cout<<"Delete category CLIPS :"<<std::endl;
  std::string name =this->ui->nameCLIPScat->text().toStdString();
    std::cout<<"\n Se eliminara: "<<name<<std::endl;
    std::map<std::string , std::vector<std::string> >::iterator itx=categorys.find(name);
    if(itx != categorys.end()){
       //std::cout<<"Si esta contenido en el map "<<std::endl;
        JustinaRepresentation::deleteCategorys(categorys,name);
        categorys=categorys;
        this->ui->categoryCLIPStab->setRowCount(0);
        for(std::map<std::string , std::vector<std::string> >::iterator itc = categorys.begin() ; itc != categorys.end() ; itc++){
             this->ui->categoryCLIPStab->insertRow(this->ui->categoryCLIPStab->rowCount());
             float row = this->ui->categoryCLIPStab->rowCount()-1;
             this->ui->categoryCLIPStab->setItem( row, NAME , new QTableWidgetItem(QString::fromStdString(itc->first)));
             this->ui->categoryCLIPStab->setItem( row, X , new QTableWidgetItem(QString::fromStdString(itc->second[0])));
             this->ui->categoryCLIPStab->setItem( row, Y , new QTableWidgetItem(QString::fromStdString(itc->second[1])));
             this->ui->categoryCLIPStab->setItem( row, A , new QTableWidgetItem(QString::fromStdString(itc->second[2])));
             this->ui->categoryCLIPStab->setItem( row, C1 , new QTableWidgetItem(QString::fromStdString(itc->second[3])));
             this->ui->categoryCLIPStab->setItem( row, C2 , new QTableWidgetItem(QString::fromStdString(itc->second[4])));
             this->ui->categoryCLIPStab->setItem( row, C3 , new QTableWidgetItem(QString::fromStdString(itc->second[5])));
         }

        this->ui->categoryCLIPStab->resizeRowsToContents();
        this->ui->categoryCLIPStab->resizeColumnsToContents();
        this->ui->categoryCLIPStab->setSelectionMode(QAbstractItemView::SingleSelection);
        this->ui->categoryCLIPStab->setSelectionBehavior(QAbstractItemView::SelectRows);
        this->ui->categoryCLIPStab->setEditTriggers(QAbstractItemView::NoEditTriggers);
    }
}

void MainWindow::on_createCLIPSfile_clicked(){
    std::cout<<"Generate file CLIPS :"<<std::endl;

    std::string path= ros::package::getPath("knowledge_representation");
    std::string directory="/scripts/base_data/";
    
    std::string nameCLIPS="/scripts/virbot_gpsr/virbot_initial_state_v2.clp";
    std::string nameONTOLOGY="stuff_ontology.txt";
    std::string pathFILECLIPS=path+nameCLIPS;
    std::string pathFILEONTOLOGY=path+directory+nameONTOLOGY;
    std::cout<<"\n Directory:   "<<pathFILECLIPS<< std::endl;
    std::cout<<"\n Directory:   "<<pathFILEONTOLOGY<< std::endl;
        
    
    std::ofstream CLIPSfile(pathFILECLIPS.c_str());
    std::ofstream ONTOLOGYfile(pathFILEONTOLOGY.c_str());

    if(CLIPSfile.fail()){
        std::cout<<"\n Fail File  CLIPS "<<std::endl;
    }else{
        std::cout<<"File CLIPS ready to save data "<<std::endl;
    }
    
   if(ONTOLOGYfile.fail()){
        std::cout<<"\n Fail File ONTOLOGY"<<std::endl;
    }else{
        std::cout<<"File ONTOLOGY ready to save data "<<std::endl;
    }


   CLIPSfile<<"\n\n(deffacts Initial-state-objects-rooms-zones-actors";

   std::cout<<" OBJECTS :"<<std::endl;
   CLIPSfile<<"\n\n\n";
   CLIPSfile<<";;;;;;;;;;OBJECTS";
   CLIPSfile<<"\n\n\n";
   ONTOLOGYfile<<"\n\n\n";
   ONTOLOGYfile<<"#SNACKS";
   ONTOLOGYfile<<"\n";
   for(std::map<std::string , std::vector<std::string> >::iterator it2 = objects.begin() ; it2 != objects.end(); it2++ ){
       std::string name=JustinaRepresentation::covertLetters(it2->first);
       std::string category=JustinaRepresentation::covertLetters(it2->second[0]);
       std::string location=JustinaRepresentation::covertLetters(it2->second[1]);
       std::string room=JustinaRepresentation::covertLetters(it2->second[2]);
       std::string weight=JustinaRepresentation::covertLetters(it2->second[3]);
       std::string size=JustinaRepresentation::covertLetters(it2->second[4]);
       std::string color=JustinaRepresentation::covertLetters(it2->second[5]);
       std::string quantity=JustinaRepresentation::covertLetters(it2->second[6]);
       /*std::cout<<"\n Name: "<<it2->first<<std::endl;
       std::cout<<"\t Category: "<<it2->second[0]<<std::endl;
       std::cout<<"\t Location: "<<it2->second[1]<< std::endl;
       std::cout<<"\n Room: "<<it2->second[2]<<std::endl;
       std::cout<<"\t Weight: "<<it2->second[3]<<std::endl;
       std::cout<<"\t Size: "<<it2->second[4]<< std::endl;
       std::cout<<"\t Color: "<<it2->second[5]<< std::endl;
       std::cout<<"\t Quantity: "<<it2->second[6]<< std::endl;*/  
       CLIPSfile<<"\n(item (type Objects) (name "<<it2->first<<") (zone "<<it2->second[1]<<") (image cereal) (attributes pick) (pose 0.0 0.0 0.0) (category "<<it2->second[0]<<") (room "<<it2->second[2]<<")(grasp " << it2->second[6] <<")(weight "<<it2->second[3]<<")(size "<<it2->second[4]<<")(height 1)(wide 10)(color "<<it2->second[5]<<")(biggest " << it2->second[7] <<  ")(smallest "<< it2->second[8] <<") (heaviest " << it2->second[9]<<") (lightest " << it2->second[10] << "))\n";  
       ONTOLOGYfile<<"\n("<<name<<"\tis_kind_of      item)";
   }
   std::cout<<" CATEGORYS :"<<std::endl;
   CLIPSfile<<"\n\n\n";
   CLIPSfile<<";;;;;;;;;;CATEGORIES";
   CLIPSfile<<"\n\n\n";
   ONTOLOGYfile<<"\n\n\n";
   ONTOLOGYfile<<"#CATEGORIES";
   ONTOLOGYfile<<"\n";
   for(std::map<std::string , std::vector<std::string> >::iterator it4 = categorys.begin() ; it4 != categorys.end(); it4++ ){
        std::string name=JustinaRepresentation::covertLetters(it4->first);
        std::string zone=JustinaRepresentation::covertLetters(it4->second[0]);
        std::string quantity=JustinaRepresentation::covertLetters(it4->second[1]);
        std::string biggets=JustinaRepresentation::covertLetters(it4->second[2]);
        std::string smallest=JustinaRepresentation::covertLetters(it4->second[3]);
        std::string heaviest=JustinaRepresentation::covertLetters(it4->second[4]);
        std::string lightest=JustinaRepresentation::covertLetters(it4->second[5]);
        
        //std::cout<<"\n Name: "<<it4->first;
        //std::cout<<"\t Zone: "<<it4->second[0];
        //std::cout<<"\t Quantity: "<<it4->second[1]<< std::endl;
        //std::cout<<"\t Biggets: "<<it4->second[2];
        //std::cout<<"\t Smallets: "<<it4->second[3]<< std::endl;
        //std::cout<<"\t Heaviets: "<<it4->second[4];
        //std::cout<<"\t Lightest: "<<it4->second[5]<< std::endl; 
        CLIPSfile<<"\n(item (type Category) (name "<<it4->first<<") (zone "<<it4->second[0]<<")(quantity "<<3<<")(biggest "<<it4->second[2]<<")(smallest "<<it4->second[3]<<") (heaviest "<<it4->second[4]<<") (lightest "<<it4->second[5]<<"))  \n";  
        ONTOLOGYfile<<"\n("<<name<<"\tis_kind_of      category)";
   }
   std::cout<<" PEOPLE :"<<std::endl;
   CLIPSfile<<"\n\n\n";
   CLIPSfile<<";;;;;;;;;;PEOPLE";
   CLIPSfile<<"\n\n\n";
   ONTOLOGYfile<<"\n\n\n";
   ONTOLOGYfile<<"#PERSON";
   ONTOLOGYfile<<"\n";
   for(std::map<std::string , std::vector<std::string> >::iterator it3 = peoples.begin() ; it3 != peoples.end(); it3++ ){
       std::string name=JustinaRepresentation::covertLetters(it3->first);
       std::string age=JustinaRepresentation::covertLetters(it3->second[0]);
       std::string gender=JustinaRepresentation::covertLetters(it3->second[0]);
        //std::cout<<"\n Name: "<<it3->first;
       //std::cout<<"\t Age: "<<it3->second[0];
       //std::cout<<"\t Gender: "<<it3->second[1]<< std::endl;
        CLIPSfile<<"\n(item (type Objects) (name "<<it3->first<<")(zone living_room)"<<"(image "<<it3->first<<")"<<"(attributes pick)(pose -1.87 8.64 0.0))\n";
        ONTOLOGYfile<<"\n("<<name<<"\tis_kind_of      person)";
   }
   std::cout<<" LOCATIONS :"<<std::endl;
   CLIPSfile<<"\n\n\n";
   CLIPSfile<<";;;;;;;; LOCATIONS";
   CLIPSfile<<"\n\n\n";
   CLIPSfile<<"\n\n\n";
   CLIPSfile<<";;;;;;;; ROOMS";
   CLIPSfile<<"\n\n\n";
   ONTOLOGYfile<<"\n\n\n";
   ONTOLOGYfile<<"#ROOMS";
   ONTOLOGYfile<<"\n";
   
   for(std::map<std::string , std::vector<std::string> >::iterator itl1 = locations.begin() ; itl1 != locations.end(); itl1++ ){
        std::string type=JustinaRepresentation::covertLetters(itl1->second[0]);
        std::string name=JustinaRepresentation::covertLetters(itl1->first);
        std::string quantity=JustinaRepresentation::covertLetters(itl1->second[1]);
        std::string room=JustinaRepresentation::covertLetters(itl1->second[2]);
        //std::cout<<"\n ===> Locations Type: "<<itl1->second[0]<<std::endl;
        //std::cout<<"\nName: "<<it->first<<std::endl;
        //std::cout<<"\nQuantity: "<<it->second[1]<<std::endl;
        //std::cout<<"\nRoom: "<<it->second[2]<<std::endl;
        //CLIPSfile<<"\n(item (type "<<itl1->second[0]<<") (name "<<itl1->first<<") (pose -3.55 -3.0 0.0)(quantity "<<itl1->second[1]<<") (quantitys "<<itl1->second[1]<<"))\n";
        if(type == "room"){
         CLIPSfile<<"\n(item (type "<<itl1->second[0]<<") (name "<<itl1->first<<") (pose -3.55 -3.0 0.0)(quantity "<<itl1->second[1]<<") (quantitys "<<itl1->second[1]<<"))\n";   
         ONTOLOGYfile<<"\n("<<name<<"\tis_kind_of      room)";
        } 
         //ONTOLOGYfile<<"\n(bed              is_kind_of      place)";
    }

   CLIPSfile<<"\n\n\n";
   CLIPSfile<<";;;;;;;; PLACEMENT";
   CLIPSfile<<"\n\n\n"; 
   ONTOLOGYfile<<"\n\n\n";
   ONTOLOGYfile<<"#PLACEMENT";
   ONTOLOGYfile<<"\n";
   for(std::map<std::string , std::vector<std::string> >::iterator itl2 = locations.begin() ; itl2 != locations.end(); itl2++ ){
        std::string type=JustinaRepresentation::covertLetters(itl2->second[0]);
        std::string name=JustinaRepresentation::covertLetters(itl2->first);
        std::string quantity=JustinaRepresentation::covertLetters(itl2->second[1]);
        std::string room=JustinaRepresentation::covertLetters(itl2->second[2]);
       // std::cout<<"\n ===> Locations Type: "<<itl2->second[0]<<std::endl;
        //std::cout<<"\nName: "<<it->first<<std::endl;
        //std::cout<<"\nQuantity: "<<it->second[1]<<std::endl;
        //std::cout<<"\nRoom: "<<it->second[2]<<std::endl;

        if(type == "furniture"){
          CLIPSfile<<"\n(item (type "<<itl2->second[0]<<") (name "<<itl2->first<<") (pose -3.55 -3.0 0.0)(quantity "<<itl2->second[1]<<") (quantitys "<<itl2->second[1]<<") (possession " << itl2->second[2] << ") (room " << itl2->second[2] << ") (attributes no_visited))\n";  
          ONTOLOGYfile<<"\n("<<name<<"\tis_kind_of      place)";
        }
   }

   CLIPSfile<<"\n) \n;;;;;;";
   ONTOLOGYfile<<"\n\n";
   std::cout<<"\n  File CLIPS and ONTOLOGY Saved "<<std::endl;

   CLIPSfile.close();
   ONTOLOGYfile.close();
}


void MainWindow::on_locCLIPStab_itemSelectionChanged(){
    std::cout << "QMainWindow.->on_locTableWidgetCLIPS_itemSelectionChanged:" << std::endl;

    QModelIndexList indexes = this->ui->locCLIPStab->selectionModel()->selectedRows();

    foreach(QModelIndex index, indexes){
      //std::cout << "QMainWindow.->updateKnownLoacations:" << updateKnownLoacations << std::endl;
      std::cout << "QMainWindow.->row selected:" << this->ui->locCLIPStab->item(index.row(), NAME)->text().toStdString() << std::endl;
      this->ui->typeCLIPSloc->setText(this->ui->locCLIPStab->item(index.row(), NAME)->text());
      this->ui->nameCLIPSloc->setText(this->ui->locCLIPStab->item(index.row(), X)->text());
      this->ui->quantCLIPSloc->setText(this->ui->locCLIPStab->item(index.row(), Y)->text());
      this->ui->roomCLIPSloc->setText(this->ui->locCLIPStab->item(index.row(), A)->text());
    }
}

void MainWindow::on_objCLIPStab_itemSelectionChanged()
{
    std::cout << "QMainWindow.->on_locTableWidgetCLIPS_itemSelectionChanged:" << std::endl;
    QModelIndexList indexes = this->ui->objCLIPStab->selectionModel()->selectedRows();
    foreach(QModelIndex index, indexes){
      //std::cout << "QMainWindow.->updateKnownLoacations:" << updateKnownLoacations << std::endl;
      std::cout << "QMainWindow.->row selected:" << this->ui->objCLIPStab->item(index.row(), NAME)->text().toStdString() << std::endl;
      this->ui->nameCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), NAME)->text());
      this->ui->catCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), X)->text());
      this->ui->locCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), Y)->text());
      this->ui->roomCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), A)->text());
      this->ui->weightCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C1)->text());
      this->ui->sizeCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C2)->text());
      this->ui->colorCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C3)->text());
      this->ui->graspCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C4)->text());
      this->ui->biggestCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C5)->text());
      this->ui->smallestCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C6)->text());
      this->ui->heaviestCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C7)->text());
      this->ui->lightestCLIPSobj->setText(this->ui->objCLIPStab->item(index.row(), C8)->text());
    }
}


void MainWindow::on_peopleCLIPStab_itemSelectionChanged(){
    std::cout << "QMainWindow.->on_peopleTableWidgetCLIPS_itemSelectionChanged:" << std::endl;
    QModelIndexList indexes = this->ui->peopleCLIPStab->selectionModel()->selectedRows();
    foreach(QModelIndex index, indexes){
      std::cout << "QMainWindow.->row selected:" << this->ui->peopleCLIPStab->item(index.row(), NAME)->text().toStdString() << std::endl;
      this->ui->nameCLIPSpeople->setText(this->ui->peopleCLIPStab->item(index.row(), NAME)->text());
      this->ui->AgeCLIPSpeople->setText(this->ui->peopleCLIPStab->item(index.row(), X)->text());
      this->ui->GenderCLIPSpeople->setText(this->ui->peopleCLIPStab->item(index.row(), Y)->text());
    }

}

void MainWindow::on_categoryCLIPStab_itemSelectionChanged(){
  std::cout << "QMainWindow.->on_categoryTableWidgetCLIPS_itemSelectionChanged:" << std::endl;
   QModelIndexList indexes = this->ui->categoryCLIPStab->selectionModel()->selectedRows();
   foreach(QModelIndex index, indexes){
      std::cout << "QMainWindow.->row selected:" << this->ui->categoryCLIPStab->item(index.row(), NAME)->text().toStdString() << std::endl;
      this->ui->nameCLIPScat->setText(this->ui->categoryCLIPStab->item(index.row(),NAME)->text());
      this->ui->zoneCLIPScat->setText(this->ui->categoryCLIPStab->item(index.row(),X)->text());
      this->ui->quantityCLIPScat->setText(this->ui->categoryCLIPStab->item(index.row(),Y)->text());
      this->ui->biggetsCLIPScat->setText(this->ui->categoryCLIPStab->item(index.row(),A)->text());
      this->ui->smallestCLIPScat->setText(this->ui->categoryCLIPStab->item(index.row(),C1)->text());
      this->ui->heavietsCLIPScat->setText(this->ui->categoryCLIPStab->item(index.row(),C2)->text());
      this->ui->lightestCLIPScat->setText(this->ui->categoryCLIPStab->item(index.row(),C3)->text());
   }

}

void MainWindow::on_rotateButton_clicked()
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << "rotate" << std::endl;
    msg.data = ss.str();
    JustinaVision::moveBaseTrainVision(msg);
}

void MainWindow::on_trainObjButton_clicked()
{
    std::vector<vision_msgs::VisionObject> recoObjList;
    if(false) //!JustinaVision::detectObjects(recoObjList))
    {
        std::cout << "MainWindow.->Cannot dectect objects :'( " << std::endl;
    }else if (false){ //(recoObjList.size() > 1){
        std::cout << "MainWindow.->Too many objects :'( " << std::endl;
    }else{
        std::cout << "MainWindow.->One object detected. Ready to Train" << std::endl;
        std::string obj_name = this->ui->objTxtGoalObject_2->text().toStdString();
        JustinaVision::trainObjectByHeight(obj_name);
    }

}

void MainWindow::on_pushButtonDownTorso_clicked()
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << "moveDown" << std::endl;
    msg.data = ss.str();
    JustinaManip::moveTorsoDown(msg);
}

void MainWindow::on_pushButtonUpTorso_clicked()
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << "moveUp" << std::endl;
    msg.data = ss.str();
    JustinaManip::moveTorsoUp(msg);
}

void MainWindow::on_typeView_currentIndexChanged(const QString &arg1)
{
    if(arg1.toStdString().compare("Navigation") == 0){
        manager_->load(configNav.mapGetChild("Visualization Manager"));
    }
    else if(arg1.toStdString().compare("Visualization") == 0){
        manager_->load(configViz.mapGetChild("Visualization Manager"));
    }
}

void MainWindow::on_actBtnExecRobocup_pressed()
{
    if(this->ui->actCmbRobocup->currentIndex() == 0)
    {
        system("rosrun act_pln storing_groseries_test &");
    }
}





