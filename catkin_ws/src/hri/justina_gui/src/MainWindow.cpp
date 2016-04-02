#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent):
    QWidget(parent)
{
    this->tabWidget = new QTabWidget(this);
    this->tabWidget->setFixedSize(this->size());

    this->tabPlanning = new QWidget();
    this->tabNavigation = new QWidget();
    this->tabManipulation = new QWidget();
    this->tabWidget->addTab(tabPlanning, "Planning");
    this->tabWidget->addTab(tabNavigation, "Navigation");
    this->tabWidget->addTab(tabManipulation, "Manipulation");
    this->tabWidget->setCurrentIndex(1);

    this->navTxtGoalPose = new QLineEdit(tabNavigation);
    this->navTxtStartPose = new QLineEdit(tabNavigation);
    this->navBtnCalcPath = new QPushButton("Calc Path", tabNavigation);
    this->navBtnExecPath = new QPushButton("Exec Path", tabNavigation);
    this->navLblGoalPose = new QLabel("Goal Pose:", tabNavigation);
    this->navLblStartPose = new QLabel("Start Pose:", tabNavigation);
    this->navLblRobotPose = new QLabel("Robot Pose: ", tabNavigation);
    this->navLblStartPose->setGeometry(2,2, 75, 25);
    this->navTxtStartPose->setGeometry(80, 2, 165, 25);
    this->navBtnCalcPath->setGeometry(250, 2, 80, 25);
    this->navLblGoalPose->setGeometry(2, 27, 75, 25);
    this->navTxtGoalPose->setGeometry(80, 27, 165, 25);
    this->navBtnExecPath->setGeometry(250, 27, 80, 25);
    this->navLblRobotPose->setGeometry(2, 52, 200, 25);

    QObject::connect(this->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(this->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(this->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
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
    QObject::connect(qtRosNode, SIGNAL(onCurrentPoseReceived(float, float, float)), this, SLOT(currentPoseReceived(float, float, float)));

    //Connect signals from MainWindow to QtRosNode
    //For example, for publishing a msg when a button is pressed
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
        goalX = this->0.0;
        goalY = this->1.0;
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
		if(!(ssGoalX >> goalX) || !(ssStartY >> startY))
        {
            this->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    
    this->qtRosNode->publish_PathCalculator_WaveFront(0,0,0, 1, 0, 0);
}

void MainWindow::currentPoseReceived(float currentX, float currentY, float currentTheta)
{
    //std::cout << "MainWindow.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    QString txt = "Robot Pose: " + QString::number(currentX,'f',3) + "  " + QString::number(currentY,'f',3) + "  " + QString::number(currentTheta,'f',4);
    this->navLblRobotPose->setText(txt);
    this->robotX = currentX;
    this->robotY = currentY;
    this->robotTheta = currentTheta;
}
