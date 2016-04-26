#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));
    QObject::connect(ui->hdTxtPan, SIGNAL(returnPressed()), this, SLOT(hdPanTiltChanged()));
    QObject::connect(ui->hdTxtTilt, SIGNAL(returnPressed()), this, SLOT(hdPanTiltChanged()));
    //QObject::connect(ui->hdBtnPanLeft, SIGNAL(clicked()), this, SLOT(hdBtnPanLeft_pressed()));
    //QObject::connect(ui->hdBtnPanRight, SIGNAL(clicked()), this, SLOT(hdBtnPanRight_pressed()));
    //QObject::connect(ui->hdBtnTiltUp, SIGNAL(clicked()), this, SLOT(hdBtnTiltUp_pressed()));
    //QObject::connect(ui->hdBtnTiltDown, SIGNAL(clicked()), this, SLOT(hdBtnTiltDown_pressed()));
    //for(int i=0; i < 7; i++)
    //    QObject::connect(this->laTxtAngles[i], SIGNAL(returnPressed()), this, SLOT(laAnglesChanged()));
    //for(int i=0; i < 7; i++)
    //    QObject::connect(this->raTxtAngles[i], SIGNAL(returnPressed()), this, SLOT(raAnglesChanged()));
    QObject::connect(ui->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgSayChanged()));
    QObject::connect(ui->sprTxtRecognized, SIGNAL(returnPressed()), this, SLOT(sprRecognizedChanged()));

    this->robotX = 0;
    this->robotY = 0;
    this->robotTheta = 0;
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

void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
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
    //this->qtRosNode->call_PathCalculator_WaveFront(startX, startY, 0, goalX, goalY, 0);
    //this->qtRosNode->call_PathCalculator_AStar(startX, startY, 0, goalX, goalY, 0, this->calculatedPath);
    JustinaNavigation::calcPathFromAllAStar(startX, startY, goalX, goalY, this->calculatedPath);
}

void MainWindow::navBtnExecPath_pressed()
{
    this->navBtnCalcPath_pressed();
    this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
    //this->qtRosNode->publish_SimpleMove_GoalPath(this->calculatedPath);
    JustinaNavigation::startMovePath(this->calculatedPath);
}

void MainWindow::hdBtnPanLeft_pressed()
{
    float goalPan = this->headPan + 0.1;
    if(goalPan > 1.5708)
        goalPan = 1.5708;
    QString txt = QString::number(goalPan, 'f', 4);
    this->ui->hdTxtPan->setText(txt);
    this->hdPanTiltChanged();
}

void MainWindow::hdBtnPanRight_pressed()
{
    float goalPan = this->headPan - 0.1;
    if(goalPan < -1.5708)
        goalPan = -1.5708;
    QString txt = QString::number(goalPan, 'f', 4);
    this->ui->hdTxtPan->setText(txt);
    this->hdPanTiltChanged();
}
