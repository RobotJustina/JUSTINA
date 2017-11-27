#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "QtRosNode.h"
#include <QMainWindow>
#include <QMenu>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setQtRosNode(QtRosNode *qtRosNode);

private slots:
    void on_delete_2_pressed();
    void on_add_pressed();
    void on_copy_pressed();
    void on_rename_pressed();
    void updateGraphicsSlot();
    void closeEvent(QCloseEvent *event);
private:
    Ui::MainWindow *ui;
    QtRosNode *qtRosNode;
};

#endif // MAINWINDOW_H
