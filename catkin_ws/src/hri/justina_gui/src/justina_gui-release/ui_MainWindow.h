/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget;
    QWidget *tabGeneral;
    QGroupBox *gbNavig;
    QPushButton *navBtnExecPath;
    QLineEdit *navTxtStartPose;
    QLineEdit *lineEdit;
    QLabel *navLblStartPose;
    QPushButton *navBtnCalcPath;
    QLabel *navLblGoalPose;
    QLabel *navLblRobotPose;
    QLabel *navLblStatus;
    QGroupBox *gbManip;
    QLabel *laLabel;
    QLabel *raLabel;
    QLabel *laLblAngles0;
    QLabel *raLblAngles0;
    QLabel *laLblAngles1;
    QLabel *raLblAngles1;
    QLabel *laLblAngles2;
    QLabel *raLblAngles2;
    QLabel *laLblAngles3;
    QLabel *raLblAngles3;
    QLabel *raLblAngles6;
    QLabel *laLblAngles6;
    QLabel *laLblAngles4;
    QLabel *raLblAngles5;
    QLabel *raLblGripper;
    QLabel *raLblAngles4;
    QLabel *laLblGripper;
    QLabel *laLblAngles5;
    QRadioButton *laRbCartesian;
    QRadioButton *laRbArticular;
    QRadioButton *laRbCartesian_2;
    QRadioButton *laRbArticular_2;
    QDoubleSpinBox *laTxtAngles0;
    QDoubleSpinBox *raTxtAngles0;
    QDoubleSpinBox *laTxtAngles1;
    QDoubleSpinBox *raTxtAngles1;
    QDoubleSpinBox *laTxtAngles2;
    QDoubleSpinBox *raTxtAngles2;
    QDoubleSpinBox *laTxtAngles3;
    QDoubleSpinBox *raTxtAngles3;
    QDoubleSpinBox *laTxtAngles4;
    QDoubleSpinBox *raTxtAngles4;
    QDoubleSpinBox *laTxtAngles5;
    QDoubleSpinBox *raTxtAngles5;
    QDoubleSpinBox *laTxtAngles6;
    QDoubleSpinBox *raTxtAngles6;
    QDoubleSpinBox *laTxtGripper;
    QDoubleSpinBox *raTxtGripper;
    QComboBox *laBtnGoTo;
    QComboBox *raBtnGoTo;
    QWidget *tabNavigation;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QButtonGroup *btnGrpLeftArm;
    QButtonGroup *btnGrpRightArm;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(640, 530);
        MainWindow->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 9px;\n"
"    margin-top: 0.5em;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 10px;\n"
"    padding: 0 3px 0 3px;\n"
"}"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 640, 492));
        tabGeneral = new QWidget();
        tabGeneral->setObjectName(QString::fromUtf8("tabGeneral"));
        gbNavig = new QGroupBox(tabGeneral);
        gbNavig->setObjectName(QString::fromUtf8("gbNavig"));
        gbNavig->setGeometry(QRect(5, 0, 321, 131));
        gbNavig->setStyleSheet(QString::fromUtf8(""));
        navBtnExecPath = new QPushButton(gbNavig);
        navBtnExecPath->setObjectName(QString::fromUtf8("navBtnExecPath"));
        navBtnExecPath->setGeometry(QRect(230, 45, 81, 25));
        navTxtStartPose = new QLineEdit(gbNavig);
        navTxtStartPose->setObjectName(QString::fromUtf8("navTxtStartPose"));
        navTxtStartPose->setGeometry(QRect(90, 20, 140, 25));
        lineEdit = new QLineEdit(gbNavig);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setGeometry(QRect(90, 45, 140, 25));
        navLblStartPose = new QLabel(gbNavig);
        navLblStartPose->setObjectName(QString::fromUtf8("navLblStartPose"));
        navLblStartPose->setGeometry(QRect(10, 20, 81, 17));
        navBtnCalcPath = new QPushButton(gbNavig);
        navBtnCalcPath->setObjectName(QString::fromUtf8("navBtnCalcPath"));
        navBtnCalcPath->setGeometry(QRect(230, 20, 81, 25));
        navLblGoalPose = new QLabel(gbNavig);
        navLblGoalPose->setObjectName(QString::fromUtf8("navLblGoalPose"));
        navLblGoalPose->setGeometry(QRect(10, 45, 81, 17));
        navLblRobotPose = new QLabel(gbNavig);
        navLblRobotPose->setObjectName(QString::fromUtf8("navLblRobotPose"));
        navLblRobotPose->setGeometry(QRect(10, 75, 231, 17));
        navLblStatus = new QLabel(gbNavig);
        navLblStatus->setObjectName(QString::fromUtf8("navLblStatus"));
        navLblStatus->setGeometry(QRect(10, 100, 271, 17));
        gbManip = new QGroupBox(tabGeneral);
        gbManip->setObjectName(QString::fromUtf8("gbManip"));
        gbManip->setGeometry(QRect(5, 140, 321, 311));
        laLabel = new QLabel(gbManip);
        laLabel->setObjectName(QString::fromUtf8("laLabel"));
        laLabel->setGeometry(QRect(10, 20, 67, 17));
        raLabel = new QLabel(gbManip);
        raLabel->setObjectName(QString::fromUtf8("raLabel"));
        raLabel->setGeometry(QRect(165, 20, 81, 17));
        laLblAngles0 = new QLabel(gbManip);
        laLblAngles0->setObjectName(QString::fromUtf8("laLblAngles0"));
        laLblAngles0->setGeometry(QRect(10, 65, 41, 17));
        raLblAngles0 = new QLabel(gbManip);
        raLblAngles0->setObjectName(QString::fromUtf8("raLblAngles0"));
        raLblAngles0->setGeometry(QRect(165, 65, 41, 17));
        laLblAngles1 = new QLabel(gbManip);
        laLblAngles1->setObjectName(QString::fromUtf8("laLblAngles1"));
        laLblAngles1->setGeometry(QRect(10, 90, 41, 17));
        raLblAngles1 = new QLabel(gbManip);
        raLblAngles1->setObjectName(QString::fromUtf8("raLblAngles1"));
        raLblAngles1->setGeometry(QRect(165, 90, 41, 17));
        laLblAngles2 = new QLabel(gbManip);
        laLblAngles2->setObjectName(QString::fromUtf8("laLblAngles2"));
        laLblAngles2->setGeometry(QRect(10, 115, 41, 17));
        raLblAngles2 = new QLabel(gbManip);
        raLblAngles2->setObjectName(QString::fromUtf8("raLblAngles2"));
        raLblAngles2->setGeometry(QRect(165, 115, 41, 17));
        laLblAngles3 = new QLabel(gbManip);
        laLblAngles3->setObjectName(QString::fromUtf8("laLblAngles3"));
        laLblAngles3->setGeometry(QRect(10, 140, 41, 17));
        raLblAngles3 = new QLabel(gbManip);
        raLblAngles3->setObjectName(QString::fromUtf8("raLblAngles3"));
        raLblAngles3->setGeometry(QRect(165, 140, 41, 17));
        raLblAngles6 = new QLabel(gbManip);
        raLblAngles6->setObjectName(QString::fromUtf8("raLblAngles6"));
        raLblAngles6->setGeometry(QRect(165, 215, 41, 17));
        laLblAngles6 = new QLabel(gbManip);
        laLblAngles6->setObjectName(QString::fromUtf8("laLblAngles6"));
        laLblAngles6->setGeometry(QRect(10, 215, 41, 17));
        laLblAngles4 = new QLabel(gbManip);
        laLblAngles4->setObjectName(QString::fromUtf8("laLblAngles4"));
        laLblAngles4->setGeometry(QRect(10, 165, 41, 17));
        raLblAngles5 = new QLabel(gbManip);
        raLblAngles5->setObjectName(QString::fromUtf8("raLblAngles5"));
        raLblAngles5->setGeometry(QRect(165, 190, 41, 17));
        raLblGripper = new QLabel(gbManip);
        raLblGripper->setObjectName(QString::fromUtf8("raLblGripper"));
        raLblGripper->setGeometry(QRect(165, 240, 41, 17));
        raLblAngles4 = new QLabel(gbManip);
        raLblAngles4->setObjectName(QString::fromUtf8("raLblAngles4"));
        raLblAngles4->setGeometry(QRect(165, 165, 41, 17));
        laLblGripper = new QLabel(gbManip);
        laLblGripper->setObjectName(QString::fromUtf8("laLblGripper"));
        laLblGripper->setGeometry(QRect(10, 240, 41, 17));
        laLblAngles5 = new QLabel(gbManip);
        laLblAngles5->setObjectName(QString::fromUtf8("laLblAngles5"));
        laLblAngles5->setGeometry(QRect(10, 190, 41, 17));
        laRbCartesian = new QRadioButton(gbManip);
        btnGrpLeftArm = new QButtonGroup(MainWindow);
        btnGrpLeftArm->setObjectName(QString::fromUtf8("btnGrpLeftArm"));
        btnGrpLeftArm->addButton(laRbCartesian);
        laRbCartesian->setObjectName(QString::fromUtf8("laRbCartesian"));
        laRbCartesian->setGeometry(QRect(20, 40, 61, 22));
        laRbArticular = new QRadioButton(gbManip);
        btnGrpLeftArm->addButton(laRbArticular);
        laRbArticular->setObjectName(QString::fromUtf8("laRbArticular"));
        laRbArticular->setGeometry(QRect(80, 40, 61, 22));
        laRbCartesian_2 = new QRadioButton(gbManip);
        btnGrpRightArm = new QButtonGroup(MainWindow);
        btnGrpRightArm->setObjectName(QString::fromUtf8("btnGrpRightArm"));
        btnGrpRightArm->addButton(laRbCartesian_2);
        laRbCartesian_2->setObjectName(QString::fromUtf8("laRbCartesian_2"));
        laRbCartesian_2->setGeometry(QRect(175, 40, 61, 22));
        laRbArticular_2 = new QRadioButton(gbManip);
        btnGrpRightArm->addButton(laRbArticular_2);
        laRbArticular_2->setObjectName(QString::fromUtf8("laRbArticular_2"));
        laRbArticular_2->setGeometry(QRect(240, 40, 61, 22));
        laTxtAngles0 = new QDoubleSpinBox(gbManip);
        laTxtAngles0->setObjectName(QString::fromUtf8("laTxtAngles0"));
        laTxtAngles0->setGeometry(QRect(50, 65, 105, 25));
        raTxtAngles0 = new QDoubleSpinBox(gbManip);
        raTxtAngles0->setObjectName(QString::fromUtf8("raTxtAngles0"));
        raTxtAngles0->setGeometry(QRect(205, 65, 105, 25));
        laTxtAngles1 = new QDoubleSpinBox(gbManip);
        laTxtAngles1->setObjectName(QString::fromUtf8("laTxtAngles1"));
        laTxtAngles1->setGeometry(QRect(50, 90, 105, 25));
        raTxtAngles1 = new QDoubleSpinBox(gbManip);
        raTxtAngles1->setObjectName(QString::fromUtf8("raTxtAngles1"));
        raTxtAngles1->setGeometry(QRect(205, 90, 105, 25));
        laTxtAngles2 = new QDoubleSpinBox(gbManip);
        laTxtAngles2->setObjectName(QString::fromUtf8("laTxtAngles2"));
        laTxtAngles2->setGeometry(QRect(50, 115, 105, 25));
        raTxtAngles2 = new QDoubleSpinBox(gbManip);
        raTxtAngles2->setObjectName(QString::fromUtf8("raTxtAngles2"));
        raTxtAngles2->setGeometry(QRect(205, 115, 105, 25));
        laTxtAngles3 = new QDoubleSpinBox(gbManip);
        laTxtAngles3->setObjectName(QString::fromUtf8("laTxtAngles3"));
        laTxtAngles3->setGeometry(QRect(50, 140, 105, 25));
        raTxtAngles3 = new QDoubleSpinBox(gbManip);
        raTxtAngles3->setObjectName(QString::fromUtf8("raTxtAngles3"));
        raTxtAngles3->setGeometry(QRect(205, 140, 105, 25));
        laTxtAngles4 = new QDoubleSpinBox(gbManip);
        laTxtAngles4->setObjectName(QString::fromUtf8("laTxtAngles4"));
        laTxtAngles4->setGeometry(QRect(50, 165, 105, 25));
        raTxtAngles4 = new QDoubleSpinBox(gbManip);
        raTxtAngles4->setObjectName(QString::fromUtf8("raTxtAngles4"));
        raTxtAngles4->setGeometry(QRect(205, 165, 105, 25));
        laTxtAngles5 = new QDoubleSpinBox(gbManip);
        laTxtAngles5->setObjectName(QString::fromUtf8("laTxtAngles5"));
        laTxtAngles5->setGeometry(QRect(50, 190, 105, 25));
        raTxtAngles5 = new QDoubleSpinBox(gbManip);
        raTxtAngles5->setObjectName(QString::fromUtf8("raTxtAngles5"));
        raTxtAngles5->setGeometry(QRect(205, 190, 105, 25));
        laTxtAngles6 = new QDoubleSpinBox(gbManip);
        laTxtAngles6->setObjectName(QString::fromUtf8("laTxtAngles6"));
        laTxtAngles6->setGeometry(QRect(50, 215, 105, 25));
        raTxtAngles6 = new QDoubleSpinBox(gbManip);
        raTxtAngles6->setObjectName(QString::fromUtf8("raTxtAngles6"));
        raTxtAngles6->setGeometry(QRect(205, 215, 105, 25));
        laTxtGripper = new QDoubleSpinBox(gbManip);
        laTxtGripper->setObjectName(QString::fromUtf8("laTxtGripper"));
        laTxtGripper->setGeometry(QRect(50, 240, 105, 25));
        raTxtGripper = new QDoubleSpinBox(gbManip);
        raTxtGripper->setObjectName(QString::fromUtf8("raTxtGripper"));
        raTxtGripper->setGeometry(QRect(205, 240, 105, 25));
        laBtnGoTo = new QComboBox(gbManip);
        laBtnGoTo->setObjectName(QString::fromUtf8("laBtnGoTo"));
        laBtnGoTo->setGeometry(QRect(14, 270, 141, 27));
        raBtnGoTo = new QComboBox(gbManip);
        raBtnGoTo->setObjectName(QString::fromUtf8("raBtnGoTo"));
        raBtnGoTo->setGeometry(QRect(170, 270, 141, 27));
        tabWidget->addTab(tabGeneral, QString());
        tabNavigation = new QWidget();
        tabNavigation->setObjectName(QString::fromUtf8("tabNavigation"));
        tabWidget->addTab(tabNavigation, QString());
        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "JUSTINA GUI - By Marcosoft", 0, QApplication::UnicodeUTF8));
        gbNavig->setTitle(QApplication::translate("MainWindow", "Mobile base and navigation", 0, QApplication::UnicodeUTF8));
        navBtnExecPath->setText(QApplication::translate("MainWindow", "Exec Path", 0, QApplication::UnicodeUTF8));
        navLblStartPose->setText(QApplication::translate("MainWindow", "Start Pose:", 0, QApplication::UnicodeUTF8));
        navBtnCalcPath->setText(QApplication::translate("MainWindow", "Calc Path", 0, QApplication::UnicodeUTF8));
        navLblGoalPose->setText(QApplication::translate("MainWindow", "Goal Pose:", 0, QApplication::UnicodeUTF8));
        navLblRobotPose->setText(QApplication::translate("MainWindow", "Robot Pose: 0.000 0.000 0.0000", 0, QApplication::UnicodeUTF8));
        navLblStatus->setText(QApplication::translate("MainWindow", "Robot Status: Moving to goal pose ...", 0, QApplication::UnicodeUTF8));
        gbManip->setTitle(QApplication::translate("MainWindow", "Arms and Manipulation", 0, QApplication::UnicodeUTF8));
        laLabel->setText(QApplication::translate("MainWindow", "Left Arm:", 0, QApplication::UnicodeUTF8));
        raLabel->setText(QApplication::translate("MainWindow", "Right Arm:", 0, QApplication::UnicodeUTF8));
        laLblAngles0->setText(QApplication::translate("MainWindow", "Th 0:", 0, QApplication::UnicodeUTF8));
        raLblAngles0->setText(QApplication::translate("MainWindow", "X:", 0, QApplication::UnicodeUTF8));
        laLblAngles1->setText(QApplication::translate("MainWindow", "Th 1:", 0, QApplication::UnicodeUTF8));
        raLblAngles1->setText(QApplication::translate("MainWindow", "Y:", 0, QApplication::UnicodeUTF8));
        laLblAngles2->setText(QApplication::translate("MainWindow", "Th 2:", 0, QApplication::UnicodeUTF8));
        raLblAngles2->setText(QApplication::translate("MainWindow", "Z:", 0, QApplication::UnicodeUTF8));
        laLblAngles3->setText(QApplication::translate("MainWindow", "Th 3:", 0, QApplication::UnicodeUTF8));
        raLblAngles3->setText(QApplication::translate("MainWindow", "Roll:", 0, QApplication::UnicodeUTF8));
        raLblAngles6->setText(QApplication::translate("MainWindow", "Elbo:", 0, QApplication::UnicodeUTF8));
        laLblAngles6->setText(QApplication::translate("MainWindow", "Th 6:", 0, QApplication::UnicodeUTF8));
        laLblAngles4->setText(QApplication::translate("MainWindow", "Th 4:", 0, QApplication::UnicodeUTF8));
        raLblAngles5->setText(QApplication::translate("MainWindow", "Yaw:", 0, QApplication::UnicodeUTF8));
        raLblGripper->setText(QApplication::translate("MainWindow", "Grip:", 0, QApplication::UnicodeUTF8));
        raLblAngles4->setText(QApplication::translate("MainWindow", "Pitch:", 0, QApplication::UnicodeUTF8));
        laLblGripper->setText(QApplication::translate("MainWindow", "Grip:", 0, QApplication::UnicodeUTF8));
        laLblAngles5->setText(QApplication::translate("MainWindow", "Th 5:", 0, QApplication::UnicodeUTF8));
        laRbCartesian->setText(QApplication::translate("MainWindow", "XYZ", 0, QApplication::UnicodeUTF8));
        laRbArticular->setText(QApplication::translate("MainWindow", "Art", 0, QApplication::UnicodeUTF8));
        laRbCartesian_2->setText(QApplication::translate("MainWindow", "XYZ", 0, QApplication::UnicodeUTF8));
        laRbArticular_2->setText(QApplication::translate("MainWindow", "Art", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabGeneral), QApplication::translate("MainWindow", "General", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabNavigation), QApplication::translate("MainWindow", "Navigation", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
