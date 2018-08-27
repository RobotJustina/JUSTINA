#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "QtRosNode.h"
#include "graphics_viz/treemodel.h"
#include "graphics_viz/abstractmodel.h"
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
private:
    int getLevelTreeView(QModelIndex index);
public slots:
    //void updateActions();
private slots:
    void on_delete_2_pressed();
    void on_add_pressed();
    void on_copy_pressed();
    void on_rename_pressed();
    void updateGraphicsSlot();
    void closeEvent(QCloseEvent *event);
    void on_composeTreeView_clicked(const QModelIndex &index);
    void on_colorRed_valueChanged(int arg1);
    void on_positionX_valueChanged(double arg1);

    void on_scaleX_valueChanged(double arg1);

    void on_orientationX_valueChanged(double arg1);

    void on_orientationY_valueChanged(double arg1);

    void on_orientationZ_valueChanged(double arg1);

    void on_scaleY_valueChanged(double arg1);

    void on_scaleZ_valueChanged(double arg1);

    void on_positionY_valueChanged(double arg1);

    void on_positionZ_valueChanged(double arg1);

    void on_colorGreen_valueChanged(int arg1);

    void on_colorBlue_valueChanged(int arg1);

    void on_colorAlpha_valueChanged(int arg1);

    void on_shapeType_currentIndexChanged(int index);

    void addNewWall(QModelIndex &indexWall);

signals:
    void addNewModel(QModelIndex index);

private:
    Ui::MainWindow *ui;
    QtRosNode *qtRosNode;
    QModelIndex currIndex;
};

#endif // MAINWINDOW_H
