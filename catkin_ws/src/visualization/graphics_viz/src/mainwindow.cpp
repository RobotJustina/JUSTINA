#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setQtRosNode(QtRosNode *qtRosNode){
    this->qtRosNode = qtRosNode;
    connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsSlot()));
    connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    connect(qtRosNode, SIGNAL(updateGraphics()), this->ui->glwidget, SLOT(updateGL()));
    this->ui->groupBoxPosition->hide();
    this->ui->groupBoxColor->hide();
    this->ui->groupBoxEdit->hide();
}

void MainWindow::closeEvent(QCloseEvent *event){
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
}

void MainWindow::updateGraphicsSlot(){
}


void MainWindow::on_add_pressed()
{

}

void MainWindow::on_delete_2_pressed()
{

}

void MainWindow::on_copy_pressed()
{

}

void MainWindow::on_rename_pressed()
{

}
