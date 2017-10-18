#include <GL/glew.h>

#include "graphics_viz/glwidget.h"

#include "graphics_viz/box.h"
#include "graphics_viz/sphere.h"
#include "graphics_viz/triangle.h"
#include "graphics_viz/quad.h"
#include "graphics_viz/lines.h"
#include "graphics_viz/cylinder.h"
#include "graphics_viz/grid.h"

#include "graphics_viz/Texture.h"

#include "graphics_viz/firstpersoncamera.h"
#include "graphics_viz/orbitcamera.h"
#include "graphics_viz/thirdpersoncamera.h"

#include <QCoreApplication>
#include <QKeyEvent>
#include <QResource>

Shader * shadersC;
Shader * shadersLmc;
Shader * shadersSc;
Shader * shadersT;
Shader * shadersUc;
std::shared_ptr<Texture> texture1;
std::shared_ptr<Texture> texture2;
glm::mat4 projection;
glm::mat4 globalMatrix;

std::shared_ptr<Camera> camera(new OrbitCamera());

Box box1;
Sphere sphere1(20, 20);
Sphere sphere2(20, 20);
Triangle triangle1(glm::vec3(-0.5, -0.5, 0.0), glm::vec3(0.5, -0.5, 0.0), glm::vec3(0.0, 0.5, 0.0));
Quad quad1(glm::vec3(-0.5, -0.5, 0.0), glm::vec3(-0.5, 0.5, 0.0), glm::vec3(0.5, 0.5, 0.0), glm::vec3(0.5, -0.5, 0.0));
Lines line1;
Lines line2;
Cylinder cylinder(8, 8, 1.0, 1.0);
Grid grid(5, 1.0);
Cylinder cylinderCamera(10, 10, 0.2, 0.2);

glm::vec3 lightPos(0.0f, 0.0f, 6.0f);

GLWidget::GLWidget(QWidget* parent ): leftMouseStatus(false), rightMouseStatus(false), middleMouseStatus(false){
    QGLFormat format;
    format.setVersion( 3, 3 );
    format.setProfile( QGLFormat::CoreProfile ); // Requires >=Qt-4.8.0
    format.setSampleBuffers( true );
    format.setSwapInterval(1);
    QGLWidget( format, parent );
    Q_INIT_RESOURCE(resource);
    path = ros::package::getPath("graphics_viz");
}

void GLWidget::initializeGL()
{
    rightMouseStatus = false;
    leftMouseStatus = false;
    middleMouseStatus = false;
    // Init glew
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err) {
        std::cerr << "Failed to initialize glew" << std::endl;
        return;
    }
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);

    shadersC = new Shader();
    shadersC->initialize(path + "/Shaders/color.vert", path + "/Shaders/color.frag");
    shadersLmc = new Shader();
    shadersLmc->initialize(path + "/Shaders/lightingMatColor.vert", path + "/Shaders/lightingMatColor.frag");
    shadersSc = new Shader();
    shadersSc->initialize(path + "/Shaders/simpleColor.vert", path + "/Shaders/simpleColor.frag");
    shadersT = new Shader();
    shadersT->initialize(path + "/Shaders/texture.vert", path + "/Shaders/texture.frag");
    shadersUc = new Shader();
    shadersUc->initialize(path + "/Shaders/uniformColor.vert", path + "/Shaders/uniformColor.frag");

    texture1 = std::shared_ptr<Texture>(new Texture(GL_TEXTURE_2D, path + "/Textures/bioroboanexo4.pgm"));
    texture1->load();

    projection = glm::perspective(45.0f,
                                  (GLfloat) this->width()
                                  / (GLfloat) this->height(), 0.1f, 1000.0f);
    //projection = glm::ortho(-1.0f, 1.0f,-1.0f, 1.0f, 0.1f, 100.0f);

    //camera->setCameraPos(glm::vec3(0.0, 0.0, 3.0));

    globalMatrix = glm::rotate(globalMatrix, 1.5708f, glm::vec3(1.0f, 0.0f, 0.0f));

    grid.init();
    grid.setShader(shadersC);
    grid.setColor(glm::vec4(1.0, 0.0, 1.0, 1.0));

    box1.init();
    box1.setShader(shadersLmc);

    sphere2.init();
    sphere2.setShader(shadersLmc);

    sphere1.init();
    sphere1.setShader(shadersC);

    triangle1.init();
    triangle1.setShader(shadersLmc);
    triangle1.setColor(glm::vec4(1.0, 0.0, 1.0, 1.0));

    quad1.init();
    quad1.setTexture(texture1);
    quad1.setUVTexture(glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 1.0f), glm::vec2(0.0f, 1.0f));
    quad1.setShader(shadersT);

    line1.init(glm::vec3(-0.5, -0.5, 0.0), glm::vec3(0.5, -0.5, 0.0));
    line1.setShader(shadersC);

    std::vector<glm::vec3> vertex;
    vertex.push_back(glm::vec3(-0.5, -0.5, 0.0));
    vertex.push_back(glm::vec3(-0.5, 0.5, 0.0));
    vertex.push_back(glm::vec3(0.5, 0.5, 0.0));
    line2.init(vertex);
    line2.setShader(shadersC);

    cylinder.init();
    cylinder.setShader(shadersLmc);
    //cylinder.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));

    cylinderCamera.init();
    cylinderCamera.setShader(shadersLmc);
    cylinderCamera.setColor(glm::vec4(0.0, 1.0, 1.0, 1.0));

    timer.start();
    timer2.start();
}

void GLWidget::resizeGL( int w, int h )
{
    // Set the viewport to window dimensions
    glViewport( 0, 0, w, qMax( h, 1 ) );
    projection = glm::perspective(45.0f,
                                  (GLfloat) this->width()
                                  / (GLfloat) this->height(), 0.1f, 1000.0f);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    dt = timer.elapsed();

    //glm::mat4 viewMatrix = glm::translate(glm::mat4(), glm::vec3(0.0, 0.0, -1.0));
    glm::mat4 viewMatrix = camera->getViewMatrix();
    glm::mat4 box1Transform;

    lightPos.y = -6.0 * glm::cos(glm::radians((float)timer2.elapsed()) * 0.01);
    lightPos.z = -6.0 * glm::sin(glm::radians((float)timer2.elapsed()) * 0.01);

    shadersLmc->turnOn();
    GLint viewPosLoc = shadersLmc->getUniformLocation("viewPos");
    glUniform3f(viewPosLoc, camera->getCameraPos().x, camera->getCameraPos().y, camera->getCameraPos().z);

    GLint matAmbientLoc = shadersLmc->getUniformLocation(
                "material.ambient");
    GLint matDiffuseLoc = shadersLmc->getUniformLocation(
                "material.diffuse");
    GLint matSpecularLoc = shadersLmc->getUniformLocation(
                "material.specular");
    GLint matShineLoc = shadersLmc->getUniformLocation(
                "material.shininess");
    glUniform3f(matAmbientLoc, 0.3f, 0.3f, 0.3f);
    glUniform3f(matDiffuseLoc, 0.7f, 0.7f, 0.7f);
    glUniform3f(matSpecularLoc, 0.3f, 0.3f, 0.3f);
    glUniform1f(matShineLoc, 16.0f);

    GLint lightAmbientLoc = shadersLmc->getUniformLocation(
                "light.ambient");
    GLint lightDiffuseLoc = shadersLmc->getUniformLocation(
                "light.diffuse");
    GLint lightSpecularLoc = shadersLmc->getUniformLocation(
                "light.specular");
    GLint lightPosLoc = shadersLmc->getUniformLocation("light.position");

    // Set lights properties
    glm::vec3 lightColor;
    lightColor.x = 1.0;
    lightColor.y = 1.0;
    lightColor.z = 1.0;

    glm::vec3 diffuseColor = lightColor * glm::vec3(1.0f); // Decrease the influence
    glm::vec3 ambientColor = diffuseColor * glm::vec3(1.0f); // Low influence

    glUniform3f(lightPosLoc, lightPos.x, lightPos.y, lightPos.z);
    glUniform3f(lightAmbientLoc, ambientColor.x, ambientColor.y,
                ambientColor.z);
    glUniform3f(lightDiffuseLoc, diffuseColor.x, diffuseColor.y,
                diffuseColor.z); // Let's darken the light a bit to fit the scene
    glUniform3f(lightSpecularLoc, 1.0, 1.0, 1.0);

    box1.setProjectionMatrix(projection);
    box1.setViewMatrix(viewMatrix);
    box1.setModelMatrix(box1Transform);
    //box1.enableWireMode();
    box1.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));
    //box1.render();

    sphere2.setProjectionMatrix(projection);
    sphere2.setViewMatrix(viewMatrix);
    sphere2.setModelMatrix(glm::mat4());
    //sphere2.enableWireMode();
    sphere2.setColor(glm::vec4(1.0, 0.0, 0.0, 1.0));
    //sphere2.render();

    sphere1.setProjectionMatrix(projection);
    sphere1.setViewMatrix(viewMatrix);
    sphere1.setModelMatrix(glm::scale(glm::translate(glm::mat4(), lightPos), glm::vec3(0.05, 0.05, 0.05)));
    sphere1.setColor(glm::vec4(lightColor, 1.0));
    sphere1.render();

    triangle1.setProjectionMatrix(projection);
    triangle1.setViewMatrix(viewMatrix);
    //triangle1.render();

    int w = texture1->getWidth() * 0.05;
    int h = texture1->getHeight() * 0.05;
    box1Transform = glm::scale(box1Transform, glm::vec3(w, h, 1.0));
    quad1.setProjectionMatrix(projection);
    quad1.setViewMatrix(viewMatrix);
    quad1.setModelMatrix(globalMatrix * box1Transform);
    quad1.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));
    //quad1.render();

    grid.setProjectionMatrix(projection);
    grid.setViewMatrix(viewMatrix);
    grid.setModelMatrix(globalMatrix);
    grid.render();

    line2.setProjectionMatrix(projection);
    line2.setViewMatrix(viewMatrix);
    line2.setModelMatrix(box1Transform);
    //line2.render(Lines::LINE_STRIP);

    cylinder.setProjectionMatrix(projection);
    cylinder.setViewMatrix(viewMatrix);
    cylinder.setModelMatrix(glm::scale(glm::vec3(1.0, 2.0, 1.0)));
    //cylinder.enableWireMode();
    cylinder.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));
    cylinder.render();

    glm::vec3 cameraTarget = std::static_pointer_cast<OrbitCamera>(camera)->getCameraTarget();
    cylinderCamera.setProjectionMatrix(projection);
    cylinderCamera.setViewMatrix(viewMatrix);
    cylinderCamera.setModelMatrix(glm::translate(glm::scale(glm::vec3(1.0, 0.01, 1.0)), cameraTarget));
    cylinderCamera.render();

    timer.restart();
}

void GLWidget::keyPressEvent(QKeyEvent* e){
    switch ( e->key() ){
    case Qt::Key_Escape:
        QCoreApplication::instance()->quit();
        break;
    case Qt::Key_Up:
    case Qt::Key_W:
        std::static_pointer_cast<FirstPersonCamera>(camera)->moveFrontCamera(true, dt);
        break;
    case Qt::Key_Down:
    case Qt::Key_S:
        std::static_pointer_cast<FirstPersonCamera>(camera)->moveFrontCamera(false, dt);
        break;
    case Qt::Key_Right:
    case Qt::Key_D:
        std::static_pointer_cast<FirstPersonCamera>(camera)->moveRightCamera(true, dt);
        break;
    case Qt::Key_Left:
    case Qt::Key_A:
        std::static_pointer_cast<FirstPersonCamera>(camera)->moveRightCamera(false, dt);
        break;
    default:
        QGLWidget::keyPressEvent(e);
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent* e){
    float factor = 0.1;
    if(leftMouseStatus){
        float dx = lastx - e->x();
        float dy = lasty - e->y();
        camera->mouseMoveCamera(dx, dy, dt);
    }
    else if(middleMouseStatus){
        float dx = lastx - e->x();
        float dy = lasty - e->y();
        glm::mat4 oriViewMatrix = std::static_pointer_cast<OrbitCamera>(camera)->getViewMatrix();
        glm::vec3 cameraTarget = std::static_pointer_cast<OrbitCamera>(camera)->getCameraTarget();
        glm::vec4 newCameraTarget;
        float sensitivity = std::static_pointer_cast<OrbitCamera>(camera)->getSensitivity();
        newCameraTarget.x = dy * dt * sensitivity;
        newCameraTarget.z = dx * dt * sensitivity;
        newCameraTarget = oriViewMatrix * newCameraTarget;
        cameraTarget.x += newCameraTarget.x;
        cameraTarget.z += newCameraTarget.z;
        std::static_pointer_cast<OrbitCamera>(camera)->setCameraTarget(cameraTarget);
        camera->updateCamera();
    }
    lastx = e->x();
    lasty = e->y();
}

void GLWidget::mousePressEvent(QMouseEvent* e){
    lastx = e->x();
    lasty = e->y();
    if(e->button() == Qt::MouseButton::LeftButton)
        leftMouseStatus = true;
    else if(e->button() == Qt::MouseButton::RightButton)
        rightMouseStatus = true;
    else if(e->button() == Qt::MouseButton::MiddleButton)
        middleMouseStatus = true;
}

void GLWidget::mouseReleaseEvent(QMouseEvent* e){
    if(e->button() == Qt::MouseButton::LeftButton)
        leftMouseStatus = false;
    else if(e->button() == Qt::MouseButton::RightButton)
        rightMouseStatus = false;
    else if(e->button() == Qt::MouseButton::MiddleButton)
        middleMouseStatus = false;
}

void GLWidget::wheelEvent(QWheelEvent* event){
    if(!middleMouseStatus)
        std::static_pointer_cast<OrbitCamera>(camera)->scrollMoveCamera(event->delta(), dt);
}
