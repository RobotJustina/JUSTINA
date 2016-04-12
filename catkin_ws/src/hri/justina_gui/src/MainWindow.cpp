#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent):
    QWidget(parent)
{
    this->tabWidget = new QTabWidget(this);
    this->tabWidget->setFixedSize(this->size());

    this->tabGeneral = new QWidget();
    this->tabPlanning = new QWidget();
    this->tabManipulation = new QWidget();
    this->tabWidget->addTab(tabGeneral, "General");
    this->tabWidget->addTab(tabPlanning, "Planning");
    this->tabWidget->addTab(tabManipulation, "Manipulation");
    this->tabWidget->setCurrentIndex(0);

    this->navTxtGoalPose = new QLineEdit(tabGeneral);
    this->navTxtStartPose = new QLineEdit(tabGeneral);
    this->navBtnCalcPath = new QPushButton("Calc Path", tabGeneral);
    this->navBtnExecPath = new QPushButton("Exec Path", tabGeneral);
    this->navLblGoalPose = new QLabel("Goal Pose:", tabGeneral);
    this->navLblStartPose = new QLabel("Start Pose:", tabGeneral);
    this->navLblRobotPose = new QLabel("Robot Pose: ", tabGeneral);
    this->navLblStatus = new QLabel("Base Status: ", tabGeneral);
    this->navLblStartPose->setGeometry(2,2, 75, 25);
    this->navTxtStartPose->setGeometry(80, 2, 165, 25);
    this->navBtnCalcPath->setGeometry(250, 2, 80, 25);
    this->navLblGoalPose->setGeometry(2, 27, 75, 25);
    this->navTxtGoalPose->setGeometry(80, 27, 165, 25);
    this->navBtnExecPath->setGeometry(250, 27, 80, 25);
    this->navLblRobotPose->setGeometry(2, 52, 230, 25);
    this->navLblStatus->setGeometry(2, 77, 230, 25);

    this->hdTxtPan = new QLineEdit(tabGeneral);
    this->hdTxtTilt = new QLineEdit(tabGeneral);
    this->hdLblPan = new QLabel("Pan:", tabGeneral);
    this->hdLblTilt = new QLabel("Tilt:", tabGeneral);
    this->hdBtnPanLeft = new QPushButton("L", tabGeneral);
    this->hdBtnPanRight = new QPushButton("R", tabGeneral);
    this->hdBtnTiltUp = new QPushButton("U", tabGeneral);
    this->hdBtnTiltDown = new QPushButton("D", tabGeneral);
    this->hdLblHeadPose = new QLabel("Head Pose: ", tabGeneral);
    this->hdLblStatus = new QLabel("Head Status: ", tabGeneral);
    this->hdLblPan->setGeometry(380, 2, 35, 25);
    this->hdLblTilt->setGeometry(380, 27, 35, 25);
    this->hdTxtPan->setGeometry(415, 2, 150, 25);
    this->hdTxtTilt->setGeometry(415, 27, 150, 25);
    this->hdBtnPanLeft->setGeometry(565, 2, 30, 25);
    this->hdBtnPanRight->setGeometry(595, 2, 30, 25);
    this->hdBtnTiltUp->setGeometry(565, 27, 30, 25);
    this->hdBtnTiltDown->setGeometry(595, 27, 30, 25);
    this->hdLblHeadPose->setGeometry(380, 52, 200, 25);
    this->hdLblStatus->setGeometry(380, 77, 200, 25);

    this->laLabel = new QLabel("Left Arm Angles: ", tabGeneral);
    this->raLabel = new QLabel("Right Arm Angles: ", tabGeneral);
    this->laLblStatus = new QLabel("LA Status: ", tabGeneral);
    this->raLblStatus = new QLabel("RA Status: ", tabGeneral);
    this->laLabel->setGeometry(2, 115, 150, 25);
    this->raLabel->setGeometry(190, 115, 150, 25);
    this->laLblStatus->setGeometry(2, 350, 170, 25);
    this->raLblStatus->setGeometry(190, 350, 170, 25);
    for(int i=0; i< 8; i++)
    {
        QString str = "Theta" + QString::number(i) + ":";
        this->laLblAngles.push_back(new QLabel(str, tabGeneral));
        this->laTxtAngles.push_back(new QLineEdit("0", tabGeneral));
        this->raLblAngles.push_back(new QLabel(str, tabGeneral));
        this->raTxtAngles.push_back(new QLineEdit("0", tabGeneral));
    }
    for(int i=0; i< this->laLblAngles.size(); i++)
    {
        this->laLblAngles[i]->setGeometry(2, 145+i*25, 50, 25);
        this->laTxtAngles[i]->setGeometry(60, 145+i*25, 110, 25);
        this->raLblAngles[i]->setGeometry(180, 145+i*25, 50, 25);
        this->raTxtAngles[i]->setGeometry(240, 145+i*25, 110, 25);
    }

    this->spgLabel = new QLabel("Say:", tabGeneral);
    this->spgTxtSay = new QLineEdit(tabGeneral);
    this->spgLabel->setGeometry(380, 115, 50, 25);
    this->spgTxtSay->setGeometry(380, 140, 250, 25);

    this->sprLabel = new QLabel("Fake Recognizer:", tabGeneral);
    this->sprTxtRecognized = new QLineEdit(tabGeneral);
    this->sprLabel->setGeometry(380, 170, 200, 25);
    this->sprTxtRecognized->setGeometry(380, 195, 250, 25);

    //Connect signals from MainWindow to QtRosNode
    //For example, for publishing a msg when a button is pressed
    QObject::connect(this->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(this->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(this->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(this->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));
    QObject::connect(this->hdTxtPan, SIGNAL(returnPressed()), this, SLOT(hdPanTiltChanged()));
    QObject::connect(this->hdTxtTilt, SIGNAL(returnPressed()), this, SLOT(hdPanTiltChanged()));
    QObject::connect(this->hdBtnPanLeft, SIGNAL(clicked()), this, SLOT(hdBtnPanLeft_pressed()));
    QObject::connect(this->hdBtnPanRight, SIGNAL(clicked()), this, SLOT(hdBtnPanRight_pressed()));
    QObject::connect(this->hdBtnTiltUp, SIGNAL(clicked()), this, SLOT(hdBtnTiltUp_pressed()));
    QObject::connect(this->hdBtnTiltDown, SIGNAL(clicked()), this, SLOT(hdBtnTiltDown_pressed()));
    for(int i=0; i < 7; i++)
        QObject::connect(this->laTxtAngles[i], SIGNAL(returnPressed()), this, SLOT(laAnglesChanged()));
    for(int i=0; i < 7; i++)
        QObject::connect(this->raTxtAngles[i], SIGNAL(returnPressed()), this, SLOT(raAnglesChanged()));
    QObject::connect(this->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgSayChanged()));
    QObject::connect(this->sprTxtRecognized, SIGNAL(returnPressed()), this, SLOT(sprRecognizedChanged()));
    
    this->robotX = 0;
    this->robotY = 0;
    this->robotTheta = 0;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;
    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(onCurrentRobotPoseReceived(float, float, float)), this, SLOT(currentRobotPoseReceived(float, float, float)));
    QObject::connect(qtRosNode, SIGNAL(onCurrentHeadPoseReceived(float, float)), this, SLOT(currentHeadPoseReceived(float, float)));
    QObject::connect(qtRosNode, SIGNAL(onCurrentLaPoseReceived(std::vector<float>)), this, SLOT(currentLeftArmPoseReceived(std::vector<float>)));
    QObject::connect(qtRosNode, SIGNAL(onCurrentRaPoseReceived(std::vector<float>)), this, SLOT(currentRightArmPoseReceived(std::vector<float>)));
    QObject::connect(qtRosNode, SIGNAL(onNavigationGoalReached(bool)), this, SLOT(navigGoalReachedReceived(bool)));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    event->accept();
}

void MainWindow::navBtnCalcPath_pressed()
{
    float startX, startY, startTheta, goalX, goalY, goalTheta;
    std::vector<std::string> parts;
    
    std::string str = this->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    {
        this->navTxtStartPose->setText("Robot");
        startX = this->robotX;
        startY = this->robotY;
        startTheta = this->robotTheta;
    }
    else
    {
        boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
        if(parts.size() < 2)
        {
            this->navTxtStartPose->setText("Invalid format");
            return;
        }
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
		if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
   
    str = this->navTxtGoalPose->text().toStdString();
    if(str.compare("livingroom") == 0) //TODO: Subscribe to predefined locations
    {
        goalX = 0.0;
        goalY = 1.0;
        goalTheta = 0;
    }
    else
    {
        boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
        if(parts.size() < 2)
        {
            this->navTxtGoalPose->setText("Invalid format");
            return;
        }
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
		if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    //this->qtRosNode->call_PathCalculator_WaveFront(startX, startY, 0, goalX, goalY, 0);
    this->qtRosNode->call_PathCalculator_AStar(startX, startY, 0, goalX, goalY, 0, this->calculatedPath);
}

void MainWindow::navBtnExecPath_pressed()
{
    this->navBtnCalcPath_pressed();
    this->navLblStatus->setText("Base Status: Moving to goal point...");
    this->qtRosNode->publish_SimpleMove_GoalPath(this->calculatedPath);
}

void MainWindow::hdBtnPanLeft_pressed()
{
    float goalPan = this->headPan + 0.1;
    if(goalPan > 1.5708)
        goalPan = 1.5708;
    QString txt = QString::number(goalPan, 'f', 4);
    this->hdTxtPan->setText(txt);
    this->hdPanTiltChanged();
}

void MainWindow::hdBtnPanRight_pressed()
{
    float goalPan = this->headPan - 0.1;
    if(goalPan < -1.5708)
        goalPan = -1.5708;
    QString txt = QString::number(goalPan, 'f', 4);
    this->hdTxtPan->setText(txt);
    this->hdPanTiltChanged();
}

void MainWindow::hdBtnTiltUp_pressed()
{
    float goalTilt = this->headTilt + 0.1;
    if(goalTilt > 1.0)
        goalTilt = 1.0;
    QString txt = QString::number(goalTilt, 'f', 4);
    this->hdTxtTilt->setText(txt);
    this->hdPanTiltChanged();
}

void MainWindow::hdBtnTiltDown_pressed()
{
    float goalTilt = this->headTilt - 0.1;
    if(goalTilt < -1.0)
        goalTilt = -1.0;
    QString txt = QString::number(goalTilt, 'f', 4);
    this->hdTxtTilt->setText(txt);
    this->hdPanTiltChanged();
}

void MainWindow::hdPanTiltChanged()
{
    float goalPan, goalTilt;
    std::vector<std::string> parts;
    
    std::string strPan = this->hdTxtPan->text().toStdString();
    std::string strTilt = this->hdTxtTilt->text().toStdString();
    std::stringstream ssPan(strPan);
    std::stringstream ssTilt(strTilt);
    if(!(ssPan >> goalPan))
    {
        this->hdTxtPan->setText("Invalid format");
        return;
    }
    if(!(ssTilt >> goalTilt))
    {
        this->hdTxtTilt->setText("Invalid format");
        return;
    }
    if(goalPan > 1.5708)
    {
        goalPan = 1.5708;
        QString txt = QString::number(goalPan, 'f', 4);
        this->hdTxtPan->setText(txt);
    }
    if(goalPan < -1.5708)
    {
        goalPan = -1.5708;
        QString txt = QString::number(goalPan, 'f', 4);
        this->hdTxtPan->setText(txt);
    }
    if(goalTilt > 1.0)
    {
        goalTilt = 1.0;
        QString txt = QString::number(goalTilt, 'f', 4);
        this->hdTxtTilt->setText(txt);
    }
    if(goalTilt < -1.0)
    {
        goalTilt = -1.0;
        QString txt = QString::number(goalTilt, 'f', 4);
        this->hdTxtTilt->setText(txt);
    }
    this->hdLblStatus->setText("Head Status: Moving to goal point...");
    this->qtRosNode->publish_Head_GoalPose(goalPan, goalTilt);
}

void MainWindow::laAnglesChanged()
{
    std::vector<float> goalAngles;
    std::vector<std::string> parts;
    std::vector<std::string> strGoalAngles;
    for(int i=0; i< 7; i++)
    {
        goalAngles.push_back(0);
        strGoalAngles.push_back(this->laTxtAngles[i]->text().toStdString());
    }
    for(int i=0; i < 7; i++)
    {
        std::stringstream ssAngle(strGoalAngles[i]);
        if(!(ssAngle >> goalAngles[i]))
        {
            this->laTxtAngles[i]->setText("Invalid format");
            return;
        }
    }
    this->laLblStatus->setText("LA Status: Moving to goal point...");
    this->qtRosNode->publish_La_GoalPose(goalAngles);
}

void MainWindow::raAnglesChanged()
{
    std::vector<float> goalAngles;
    std::vector<std::string> parts;
    std::vector<std::string> strGoalAngles;
    for(int i=0; i< 7; i++)
    {
        goalAngles.push_back(0);
        strGoalAngles.push_back(this->raTxtAngles[i]->text().toStdString());
    }
    for(int i=0; i < 7; i++)
    {
        std::stringstream ssAngle(strGoalAngles[i]);
        if(!(ssAngle >> goalAngles[i]))
        {
            this->raTxtAngles[i]->setText("Invalid format");
            return;
        }
    }
    this->raLblStatus->setText("RA Status: Moving to goal point...");
    this->qtRosNode->publish_Ra_GoalPose(goalAngles);
}

void MainWindow::spgSayChanged()
{
    std::string strToSay = this->spgTxtSay->text().toStdString();
    this->qtRosNode->publish_Spg_Say(strToSay);
}

void MainWindow::sprRecognizedChanged()
{
    std::string strToFake = this->sprTxtRecognized->text().toStdString();
    this->qtRosNode->publish_Spr_Recognized(strToFake);
}

void MainWindow::currentRobotPoseReceived(float currentX, float currentY, float currentTheta)
{
    //std::cout << "MainWindow.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    QString txt = "Robot Pose: " + QString::number(currentX,'f',3) + "  " + QString::number(currentY,'f',3) + "  " + QString::number(currentTheta,'f',4);
    this->navLblRobotPose->setText(txt);
    this->robotX = currentX;
    this->robotY = currentY;
    this->robotTheta = currentTheta;
}

void MainWindow::robotGoalPoseReached(bool success)
{
}

void MainWindow::currentHeadPoseReceived(float pan, float tilt)
{
    QString txt = "Head Pose: " + QString::number(pan, 'f', 4) + "  " + QString::number(tilt, 'f', 4);
    this->hdLblHeadPose->setText(txt);
    this->headPan = pan;
    this->headTilt = tilt;
}

void MainWindow::headGoalPoseReached(bool success)
{
}

void MainWindow::currentLeftArmPoseReceived(std::vector<float> angles)
{
    
}

void MainWindow::leftArmGoalPoseReached(bool success)
{
}

void MainWindow::currentRightArmPoseReceived(std::vector<float> angles)
{
}

void MainWindow::rightArmGoalPoseReached(bool success)
{
}

void MainWindow:: navigGoalReachedReceived(bool success)
{
    this->navLblStatus->setText("Base Status: Goal Reached (Y)");
}
