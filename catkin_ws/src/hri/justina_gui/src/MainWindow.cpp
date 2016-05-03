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

    QObject::connect(ui->btnStop, SIGNAL(clicked()), this, SLOT(stopRobot()));
    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));
    QObject::connect(ui->hdTxtPan, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    QObject::connect(ui->hdTxtTilt, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    QObject::connect(ui->laTxtAngles0, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->laTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(laAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles0, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->raTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(raAnglesChanged(double)));
    QObject::connect(ui->laRbCartesian, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbCartesianRobot, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbArticular, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->raRbCartesian, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbCartesianRobot, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbArticular, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgSayChanged()));
    QObject::connect(ui->sprTxtFakeRecog, SIGNAL(returnPressed()), this, SLOT(sprFakeRecognizedChanged()));

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
    float startX, startY, startTheta, goalX, goalY, goalTheta;
    std::vector<std::string> parts;

    std::string str = this->ui->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    {
        this->ui->navTxtStartPose->setText("Robot");
        startX = this->robotX;
        startY = this->robotY;
        startTheta = this->robotTheta;
    }
    else
    {
        boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
        if(parts.size() < 2)
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
        if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }

    str = this->ui->navTxtGoalPose->text().toStdString();
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
            this->ui->navTxtGoalPose->setText("Invalid format");
            return;
        }
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    JustinaNavigation::calcPathFromAllAStar(startX, startY, goalX, goalY, this->calculatedPath);
}

void MainWindow::navBtnExecPath_pressed()
{
    this->navBtnCalcPath_pressed();
    this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
    JustinaNavigation::startMovePath(this->calculatedPath);
}

void MainWindow::hdPanTiltChanged(double)
{
    float goalPan = this->ui->hdTxtPan->value();
    float goalTilt = this->ui->hdTxtTilt->value();
    std::cout << "QMainWindow.->Setting new head goal pose: " << goalPan << "  " << goalTilt  << std::endl;
    this->ui->hdLblStatus->setText("Head Status: Moving to goal point...");
    JustinaHardware::setHeadGoalPose(goalPan, goalTilt);
}

void MainWindow::laAnglesChanged(double d)
{
    if(this->laIgnoreValueChanged)
        return;
    
    std::vector<float> goalAngles;
    std::cout << "QMainWindow.->Setting new left arm goal pose..." << std::endl;
    goalAngles.push_back(this->ui->laTxtAngles0->value());
    goalAngles.push_back(this->ui->laTxtAngles1->value());
    goalAngles.push_back(this->ui->laTxtAngles2->value());
    goalAngles.push_back(this->ui->laTxtAngles3->value());
    goalAngles.push_back(this->ui->laTxtAngles4->value());
    goalAngles.push_back(this->ui->laTxtAngles5->value());
    goalAngles.push_back(this->ui->laTxtAngles6->value());
    
    this->ui->laLblStatus->setText("LA: Moving to goal...");
    JustinaHardware::setLeftArmGoalPose(goalAngles);
}

void MainWindow::raAnglesChanged(double d)
{
    if(this->raIgnoreValueChanged)
        return;
    
    std::vector<float> goalAngles;
    std::cout << "QMainWindow.->Setting new right arm goal pose..." << std::endl;
    goalAngles.push_back(this->ui->raTxtAngles0->value());
    goalAngles.push_back(this->ui->raTxtAngles1->value());
    goalAngles.push_back(this->ui->raTxtAngles2->value());
    goalAngles.push_back(this->ui->raTxtAngles3->value());
    goalAngles.push_back(this->ui->raTxtAngles4->value());
    goalAngles.push_back(this->ui->raTxtAngles5->value());
    goalAngles.push_back(this->ui->raTxtAngles6->value());
    
    this->ui->raLblStatus->setText("RA: Moving to goal...");
    JustinaHardware::setRightArmGoalPose(goalAngles);
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
    this->laLastRadioButton = currentRb;
    
    std::cout << "Rb changed" << std::endl;
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
        success = JustinaManip::inverseKinematics(oldValues, newValues);
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

        success = JustinaManip::directKinematics(newValues, oldValues);
    }
    if(!success)
    {
        this->laIgnoreValueChanged = false;
        return;
    }
    this->ui->laTxtAngles0->setValue(newValues[0]);
    this->ui->laTxtAngles1->setValue(newValues[1]);
    this->ui->laTxtAngles2->setValue(newValues[2]);
    this->ui->laTxtAngles3->setValue(newValues[3]);
    this->ui->laTxtAngles4->setValue(newValues[4]);
    this->ui->laTxtAngles5->setValue(newValues[5]);
    this->ui->laTxtAngles6->setValue(newValues[6]);
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

    this->raLastRadioButton = currentRb;
    
    this->raIgnoreValueChanged = true;
    if(this->ui->raRbArticular->isChecked())
    {
        this->ui->raLblAngles0->setText("Th 0:");
        this->ui->raLblAngles1->setText("Th 1:");
        this->ui->raLblAngles2->setText("Th 2:");
        this->ui->raLblAngles3->setText("Th 3:");
        this->ui->raLblAngles4->setText("Th 4:");
        this->ui->raLblAngles5->setText("Th 5:");
        this->ui->raLblAngles6->setText("Th 6:");

        this->ui->raTxtAngles0->setValue(0.0);
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

        this->ui->raTxtAngles0->setValue(1.0);
    }
    this->raIgnoreValueChanged = false;
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
    JustinaHRI::fakeSpokenSentence(strToFake);
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    float rX = JustinaNavigation::currentRobotX;
    float rY = JustinaNavigation::currentRobotY;
    float rT = JustinaNavigation::currentRobotTheta;
    //std::cout << "MainWindow.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    QString robotTxt = "Robot Pose: "+ QString::number(rX,'f',3) + "  " + QString::number(rY,'f',3) + "  " + QString::number(rT,'f',4);
    this->ui->navLblRobotPose->setText(robotTxt);
    this->robotX = rX;
    this->robotY = rY;
    this->robotTheta = rT;

    float pan = JustinaHardware::headPan;
    float tilt = JustinaHardware::headTilt;
    QString headTxt = QString::number(pan, 'f', 4) + "  " + QString::number(tilt, 'f', 4);
    this->ui->hdLblHeadPose->setText(headTxt);
    this->headPan = pan;
    this->headTilt = tilt;

    if(JustinaNavigation::isGoalReached)
        this->ui->navLblStatus->setText("Base Status: Goal Reached (Y)");

    this->ui->pgbBatt1->setValue((JustinaHardware::leftArmBatteryPerc + JustinaHardware::rightArmBatteryPerc)/2);
    this->ui->pgbBatt2->setValue((JustinaHardware::headBatteryPerc + JustinaHardware::baseBatteryPerc)/2);
    QString batt1Txt = QString::number((JustinaHardware::leftArmBattery + JustinaHardware::rightArmBattery)/2, 'f', 2) + " V";
    QString batt2Txt = QString::number((JustinaHardware::headBattery + JustinaHardware::baseBattery)/2, 'f', 2) + " V";
    this->ui->lblBatt1Level->setText(batt1Txt);
    this->ui->lblBatt2Level->setText(batt2Txt);
}
