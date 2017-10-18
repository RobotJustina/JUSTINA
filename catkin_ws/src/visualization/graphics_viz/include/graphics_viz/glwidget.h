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
#include <QTime>

class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    GLWidget(QWidget* parent = 0 );

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
};

#endif // GLWIDGET_H
