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
    std::cout << "Button calc path pressed" << std::endl;
    //this->qtRosNode->publish_SimpleMove_GoalDist(1.0);
    this->qtRosNode->publish_PathCalculator_WaveFront(0,0,0, 1, 0, 0);
}

void MainWindow::currentPoseReceived(float currentX, float currentY, float currentTheta)
{
    //std::cout << "MainWindow.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    QString txt = "Robot Pose: " + QString::number(currentX,'f',3) + "  " + QString::number(currentY,'f',3) + "  " + QString::number(currentTheta,'f',4);
    this->navLblRobotPose->setText(txt);
}
