#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->ui->laRbArticular->setChecked(true);
    this->ui->raRbArticular->setChecked(true);
    this->laLastRadioButton = 0;
    this->raLastRadioButton = 0;
    this->recSavingVideo = false;

    QObject::connect(ui->btnStop, SIGNAL(clicked()), this, SLOT(stopRobot()));
    //Navigation
    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));
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
    QObject::connect(ui->laTxtGripper, SIGNAL(valueChanged(double)), this, SLOT(laGripperChanged(double)));
    QObject::connect(ui->raTxtAngles0, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtGripper, SIGNAL(valueChanged(double)), this, SLOT(raGripperChanged(double)));
    QObject::connect(ui->laRbCartesian, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbCartesianRobot, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbArticular, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->raRbCartesian, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbCartesianRobot, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbArticular, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->laTxtGoTo, SIGNAL(returnPressed()), this, SLOT(laLocationChanged()));
    QObject::connect(ui->raTxtGoTo, SIGNAL(returnPressed()), this, SLOT(raLocationChanged()));
    //Speech synthesis and recog
    QObject::connect(ui->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgSayChanged()));
    QObject::connect(ui->sprTxtFakeRecog, SIGNAL(returnPressed()), this, SLOT(sprFakeRecognizedChanged()));
    //Vision
    QObject::connect(ui->recBtnSaveVideo, SIGNAL(clicked()), this, SLOT(recSaveVideoChanged()));
    QObject::connect(ui->recTxtImgFile, SIGNAL(returnPressed()), this, SLOT(recSaveImageChanged()));
    QObject::connect(ui->recBtnSaveImg, SIGNAL(clicked()), this, SLOT(recSaveImageChanged()));
    QObject::connect(ui->sktBtnStartRecog, SIGNAL(clicked()), this, SLOT(sktBtnStartClicked()));
    QObject::connect(ui->facBtnStartRecog, SIGNAL(clicked()), this, SLOT(facBtnStartClicked()));
    QObject::connect(ui->objTxtGoalObject, SIGNAL(returnPressed()), this, SLOT(objRecogObjectChanged()));

    this->robotX = 0;
    this->robotY = 0;
    this->robotTheta = 0;
    this->laIgnoreValueChanged = false;
    this->raIgnoreValueChanged = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
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

    bool success = true;
    if(this->ui->laRbCartesianRobot->isChecked())
        JustinaManip::startLaGoToCartesianWrtRobot(goalAngles);
    else if(this->ui->laRbCartesian->isChecked())
        JustinaManip::startLaGoToCartesian(goalAngles);
    else if(this->ui->laRbArticular->isChecked())
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

    bool success = true;
    if(this->ui->raRbCartesianRobot->isChecked())
        JustinaManip::startRaGoToCartesianWrtRobot(goalAngles);
    else if(this->ui->raRbCartesian->isChecked())
        JustinaManip::startRaGoToCartesian(goalAngles);
    else if(this->ui->raRbArticular->isChecked())
        JustinaManip::startRaGoToArticular(goalAngles);
}

void MainWindow::laGripperChanged(double d)
{
    JustinaManip::startLaOpenGripper((float)d);
}

void MainWindow::raGripperChanged(double d)
{
    JustinaManip::startRaOpenGripper((float)d);
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

    std::vector<float> oldValues;
    std::vector<float> newValues;
    bool success;
    oldValues.push_back(this->ui->laTxtAngles0->value());
    oldValues.push_back(this->ui->laTxtAngles1->value());
    oldValues.push_back(this->ui->laTxtAngles2->value());
    oldValues.push_back(this->ui->laTxtAngles3->value());
    oldValues.push_back(this->ui->laTxtAngles4->value());
    oldValues.push_back(this->ui->laTxtAngles5->value());
    oldValues.push_back(this->ui->laTxtAngles6->value());
    if(this->ui->laRbArticular->isChecked())
    {
        this->ui->laLblAngles0->setText("Th 0:");
        this->ui->laLblAngles1->setText("Th 1:");
        this->ui->laLblAngles2->setText("Th 2:");
        this->ui->laLblAngles3->setText("Th 3:");
        this->ui->laLblAngles4->setText("Th 4:");
        this->ui->laLblAngles5->setText("Th 5:");
        this->ui->laLblAngles6->setText("Th 6:");
        if(this->laLastRadioButton == 2)
            JustinaTools::transformPose("base_link", oldValues, "left_arm_link0", oldValues);  
        success = JustinaManip::inverseKinematics(oldValues, newValues);
    }
    else if(this->ui->laRbCartesian->isChecked())
    {
        this->ui->laLblAngles0->setText("X:");
        this->ui->laLblAngles1->setText("Y:");
        this->ui->laLblAngles2->setText("Z:");
        this->ui->laLblAngles3->setText("Roll:");
        this->ui->laLblAngles4->setText("Pitch:");
        this->ui->laLblAngles5->setText("Yaw:");
        this->ui->laLblAngles6->setText("Elbow:");
        if(this->laLastRadioButton == 0)
            success = JustinaManip::directKinematics(newValues, oldValues);
        else
            success = JustinaTools::transformPose("base_link", oldValues, "left_arm_link0", newValues);
    }
    else
    {
        this->ui->laLblAngles0->setText("X:");
        this->ui->laLblAngles1->setText("Y:");
        this->ui->laLblAngles2->setText("Z:");
        this->ui->laLblAngles3->setText("Roll:");
        this->ui->laLblAngles4->setText("Pitch:");
        this->ui->laLblAngles5->setText("Yaw:");
        this->ui->laLblAngles6->setText("Elbow:");
        if(this->laLastRadioButton == 0)
            success = JustinaManip::directKinematics(oldValues, oldValues);
        
        success = JustinaTools::transformPose("left_arm_link0", oldValues, "base_link", newValues);
    }
    if(!success)
    {
        this->laIgnoreValueChanged = false;
        if(this->laLastRadioButton == 0) this->ui->laRbArticular->setChecked(true);
        if(this->laLastRadioButton == 1) this->ui->laRbCartesian->setChecked(true);
        if(this->laLastRadioButton == 2) this->ui->laRbCartesianRobot->setChecked(true);
        return;
    }
    this->ui->laTxtAngles0->setValue(newValues[0]);
    this->ui->laTxtAngles1->setValue(newValues[1]);
    this->ui->laTxtAngles2->setValue(newValues[2]);
    this->ui->laTxtAngles3->setValue(newValues[3]);
    this->ui->laTxtAngles4->setValue(newValues[4]);
    this->ui->laTxtAngles5->setValue(newValues[5]);
    this->ui->laTxtAngles6->setValue(newValues[6]);
    
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

    std::vector<float> oldValues;
    std::vector<float> newValues;
    bool success;
    oldValues.push_back(this->ui->raTxtAngles0->value());
    oldValues.push_back(this->ui->raTxtAngles1->value());
    oldValues.push_back(this->ui->raTxtAngles2->value());
    oldValues.push_back(this->ui->raTxtAngles3->value());
    oldValues.push_back(this->ui->raTxtAngles4->value());
    oldValues.push_back(this->ui->raTxtAngles5->value());
    oldValues.push_back(this->ui->raTxtAngles6->value());
    if(this->ui->raRbArticular->isChecked())
    {
        this->ui->raLblAngles0->setText("Th 0:");
        this->ui->raLblAngles1->setText("Th 1:");
        this->ui->raLblAngles2->setText("Th 2:");
        this->ui->raLblAngles3->setText("Th 3:");
        this->ui->raLblAngles4->setText("Th 4:");
        this->ui->raLblAngles5->setText("Th 5:");
        this->ui->raLblAngles6->setText("Th 6:");
        if(this->raLastRadioButton == 2)
            JustinaTools::transformPose("base_link", oldValues, "right_arm_link0", oldValues);  
        success = JustinaManip::inverseKinematics(oldValues, newValues);
    }
    else if(this->ui->raRbCartesian->isChecked())
    {
        this->ui->raLblAngles0->setText("X:");
        this->ui->raLblAngles1->setText("Y:");
        this->ui->raLblAngles2->setText("Z:");
        this->ui->raLblAngles3->setText("Roll:");
        this->ui->raLblAngles4->setText("Pitch:");
        this->ui->raLblAngles5->setText("Yaw:");
        this->ui->raLblAngles6->setText("Elbow:");
        if(this->raLastRadioButton == 0)
            success = JustinaManip::directKinematics(newValues, oldValues);
        else
            success = JustinaTools::transformPose("base_link", oldValues, "right_arm_link0", newValues);
    }
    else
    {
        this->ui->raLblAngles0->setText("X:");
        this->ui->raLblAngles1->setText("Y:");
        this->ui->raLblAngles2->setText("Z:");
        this->ui->raLblAngles3->setText("Roll:");
        this->ui->raLblAngles4->setText("Pitch:");
        this->ui->raLblAngles5->setText("Yaw:");
        this->ui->raLblAngles6->setText("Elbow:");
        if(this->raLastRadioButton == 0)
            success = JustinaManip::directKinematics(oldValues, oldValues);
        
        success = JustinaTools::transformPose("right_arm_link0", oldValues, "base_link", newValues);
    }
    if(!success)
    {
        this->raIgnoreValueChanged = false;
        if(this->raLastRadioButton == 0) this->ui->raRbArticular->setChecked(true);
        if(this->raLastRadioButton == 1) this->ui->raRbCartesian->setChecked(true);
        if(this->raLastRadioButton == 2) this->ui->raRbCartesianRobot->setChecked(true);
        return;
    }
    this->ui->raTxtAngles0->setValue(newValues[0]);
    this->ui->raTxtAngles1->setValue(newValues[1]);
    this->ui->raTxtAngles2->setValue(newValues[2]);
    this->ui->raTxtAngles3->setValue(newValues[3]);
    this->ui->raTxtAngles4->setValue(newValues[4]);
    this->ui->raTxtAngles5->setValue(newValues[5]);
    this->ui->raTxtAngles6->setValue(newValues[6]);
    
    this->raLastRadioButton = currentRb;
    this->raIgnoreValueChanged = false;
}

void MainWindow::laLocationChanged()
{
    std::string strLaLocation = this->ui->laTxtGoTo->text().toStdString();
    JustinaManip::startLaGoTo(strLaLocation);
}

void MainWindow::raLocationChanged()
{
    std::string strRaLocation = this->ui->raTxtGoTo->text().toStdString();
    JustinaManip::startRaGoTo(strRaLocation);
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
        this->ui->sktBtnStartRecog->setText("Start Recognizer");
    }
    else
    {
        JustinaVision::startSkeletonFinding();
        this->sktRecognizing = true;
        this->ui->sktBtnStartRecog->setText("Stop Recognizing");
    }
}

void MainWindow::facBtnStartClicked()
{
    if(this->facRecognizing)
    {
        JustinaVision::stopFaceRecognition();
        this->facRecognizing = false;
        this->ui->facBtnStartRecog->setText("Start Recognizer");
    }
    else
    {
        JustinaVision::startFaceRecognition();
        this->facRecognizing = true;
        this->ui->facBtnStartRecog->setText("Stop Recognizing");
    }
}

void MainWindow::objRecogObjectChanged()
{
    std::vector<vision_msgs::VisionObject> recoObjList;
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

    this->ui->pgbBatt1->setValue((JustinaHardware::leftArmBatteryPerc() + JustinaHardware::rightArmBatteryPerc())/2);
    this->ui->pgbBatt2->setValue((JustinaHardware::headBatteryPerc() + JustinaHardware::baseBatteryPerc())/2);
    QString batt1Txt = QString::number((JustinaHardware::leftArmBattery() + JustinaHardware::rightArmBattery())/2, 'f', 2) + " V";
    QString batt2Txt = QString::number((JustinaHardware::headBattery() + JustinaHardware::baseBattery())/2, 'f', 2) + " V";
    this->ui->lblBatt1Level->setText(batt1Txt);
    this->ui->lblBatt2Level->setText(batt2Txt);
}
