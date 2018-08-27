#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <ros/package.h>
#include "Shader.h"
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <QtOpenGL/QGLWidget>
#include <QModelIndex>
#include <QTime>

#include "graphics_viz/box.h"
#include "graphics_viz/sphere.h"
#include "graphics_viz/triangle.h"
#include "graphics_viz/quad.h"
#include "graphics_viz/lines.h"
#include "graphics_viz/cylinder.h"
#include "graphics_viz/grid.h"
#include "graphics_viz/compositeModell.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    GLWidget(QWidget* parent = 0 );
public slots:
    void addNewModel(QModelIndex index);
    std::map<int, std::shared_ptr<CompositeModel>> getContainer(){
        return container;
    }
signals:
    void addNewWall(QModelIndex &indexWall);
    void updateTreeView(QModelIndex indexWall);

protected:
    virtual void initializeGL();
    virtual void resizeGL( int w, int h );
    virtual void paintGL();
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);
    virtual void keyPressEvent(QKeyEvent* e);
    virtual void wheelEvent(QWheelEvent* event);

private:
    bool leftMouseStatus;
    bool rightMouseStatus;
    bool middleMouseStatus;
    int lastx, lasty;
    std::string path;

    QTime timer;
    QTime timer2;
    int dt;

    glm::vec3 initRay;
    glm::vec3 endRay;
    glm::vec3 prevInitRay;
    glm::vec3 prevEndRay;
    int numClicks = 0;
    bool generateRay = false;
    std::map<int, std::shared_ptr<CompositeModel>> container;
};

#endif // GLWIDGET_H
