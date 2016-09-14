#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QObject::connect(ui->vs1, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs2, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs3, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs4, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs5, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs6, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs7, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs8, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs9, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
    QObject::connect(ui->vs10, SIGNAL(valueChanged(int)), this, SLOT(slidersValuedChanged(int)));
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

//
//Slots for Audio
//

void MainWindow::slidersValuedChanged(int)
{
    this->qtRosNode->sendEqualizer(ui->vs1->value()/100.0, ui->vs2->value()/100.0, ui->vs3->value()/100.0,
                                   ui->vs4->value()/100.0, ui->vs5->value()/100.0, ui->vs6->value()/100.0,
                                   ui->vs7->value()/100.0, ui->vs8->value()/100.0, ui->vs9->value()/100.0,
                                   ui->vs10->value()/100.0);
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
}
