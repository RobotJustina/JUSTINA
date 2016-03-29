#pragma once
#include <QWidget>

namespace rviz
{
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

class MyViz: public QWidget
{
public:
    MyViz(QWidget* parent=0);
    virtual ~MyViz();

private:
    void setThickness
};
