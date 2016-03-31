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
    this->navLblStartPose->setGeometry(2,2, 75, 25);
    this->navTxtStartPose->setGeometry(80, 2, 165, 25);
    this->navBtnCalcPath->setGeometry(250, 2, 80, 25);
    this->navLblGoalPose->setGeometry(2, 26, 75, 25);
    this->navTxtGoalPose->setGeometry(80, 26, 165, 25);
    this->navBtnExecPath->setGeometry(250, 26, 80, 25);

    QObject::connect(this->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
}

void MainWindow::navBtnCalcPath_pressed()
{
    std::cout << "Button calc path pressed" << std::endl;
}
