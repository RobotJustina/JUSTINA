#include <GL/glew.h>

#include "graphics_viz/glwidget.h"

#include "graphics_viz/box.h"
#include "graphics_viz/sphere.h"
#include "graphics_viz/triangle.h"
#include "graphics_viz/quad.h"
#include "graphics_viz/lines.h"
#include "graphics_viz/cylinder.h"
#include "graphics_viz/Texture.h"

#include "graphics_viz/firstpersoncamera.h"
#include "graphics_viz/orbitcamera.h"
#include "graphics_viz/thirdpersoncamera.h"

#include <QCoreApplication>
#include <QKeyEvent>
#include <QResource>

Shader * shader;
std::shared_ptr<Texture> texture1;
std::shared_ptr<Texture> texture2;
glm::mat4 projection;

std::shared_ptr<Camera> camera(new OrbitCamera());

Box box1;
Sphere sphere1(20, 20);
Triangle triangle1(glm::vec3(-0.5, -0.5, 0.0), glm::vec3(0.5, -0.5, 0.0), glm::vec3(0.0, 0.5, 0.0));
Quad quad1(glm::vec3(-0.5, -0.5, 0.0), glm::vec3(0.5, -0.5, 0.0), glm::vec3(0.5, 0.5, 0.0), glm::vec3(-0.5, 0.5, 0.0));
Lines line1;
Lines line2;
Cylinder cylinder(4, 4, 0.0, 1.0);


GLWidget::GLWidget(QWidget* parent ): leftMouseStatus(false), rightMouseStatus(false), middleMouseStatus(false){
    QGLFormat format;
    format.setVersion( 3, 3 );
    format.setProfile( QGLFormat::CoreProfile ); // Requires >=Qt-4.8.0
    format.setSampleBuffers( true );
    QGLWidget( format, parent );
    Q_INIT_RESOURCE(resource);
    path = ros::package::getPath("graphics_viz");
}

void GLWidget::initializeGL()
{

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

    shader = new Shader();
    shader->initialize(path + "/Shaders/texture.vert", path + "/Shaders/texture.frag");

    texture1 = std::shared_ptr<Texture>(new Texture(GL_TEXTURE_2D, path + "/Textures/bioroboanexo4.pgm"));
    texture1->load();

    projection = glm::perspective(45.0f,
                                  (GLfloat) this->width()
                                  / (GLfloat) this->height(), 0.1f, 100.0f);
    //projection = glm::ortho(-1.0f, 1.0f,-1.0f, 1.0f, 0.1f, 100.0f);

    //camera->setCameraPos(glm::vec3(0.0, 0.0, 3.0));

    box1.init();
    box1.setShader(shader);

    sphere1.init();
    sphere1.setShader(shader);

    triangle1.init();
    triangle1.setShader(shader);
    triangle1.setColor(glm::vec4(1.0, 0.0, 1.0, 1.0));

    quad1.init();
    quad1.setTexture(texture1);
    quad1.setUVTexture(glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 1.0f), glm::vec2(0.0f, 1.0f));
    quad1.setShader(shader);

    line1.init(glm::vec3(-0.5, -0.5, 0.0), glm::vec3(0.5, -0.5, 0.0));
    line1.setShader(shader);

    std::vector<glm::vec3> vertex;
    vertex.push_back(glm::vec3(-0.5, -0.5, 0.0));
    vertex.push_back(glm::vec3(-0.5, 0.5, 0.0));
    vertex.push_back(glm::vec3(0.5, 0.5, 0.0));
    line2.init(vertex);
    line2.setShader(shader);

    cylinder.init();
    cylinder.setShader(shader);
    //cylinder.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));

    timer.start();
}

void GLWidget::resizeGL( int w, int h )
{
    // Set the viewport to window dimensions
    glViewport( 0, 0, w, qMax( h, 1 ) );
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    dt = timer.elapsed();

    glm::mat4 globalRotation = glm::toMat4(rotation);

    glm::mat4 box1Transform = glm::scale(globalRotation, glm::vec3(0.5, 0.7, 0.5));
    //glm::mat4 viewMatrix = glm::translate(glm::mat4(), glm::vec3(0.0, 0.0, -1.0));
    glm::mat4 viewMatrix = camera->getViewMatrix();

    shader->turnOn();
    glm::vec3 lightPos(0.0f, 0.0f, -1.0f);
    GLint objectColorLoc = shader->getUniformLocation("objectColor");
    GLint viewPosLoc = shader->getUniformLocation("viewPos");
    glUniform3f(objectColorLoc, 1.0f, 0.5f, 0.31f);
    glUniform3f(viewPosLoc, 0.0, 0.0, -1.0);

    GLint matAmbientLoc = shader->getUniformLocation(
                "material.ambient");
    GLint matDiffuseLoc = shader->getUniformLocation(
                "material.diffuse");
    GLint matSpecularLoc = shader->getUniformLocation(
                "material.specular");
    GLint matShineLoc = shader->getUniformLocation(
                "material.shininess");
    glUniform3f(matAmbientLoc, 0.3f, 0.3f, 0.3f);
    glUniform3f(matDiffuseLoc, 0.7f, 0.7f, 0.7f);
    glUniform3f(matSpecularLoc, 0.3f, 0.3f, 0.3f);
    glUniform1f(matShineLoc, 16.0f);

    GLint lightAmbientLoc = shader->getUniformLocation(
                "light.ambient");
    GLint lightDiffuseLoc = shader->getUniformLocation(
                "light.diffuse");
    GLint lightSpecularLoc = shader->getUniformLocation(
                "light.specular");
    GLint lightPosLoc = shader->getUniformLocation("light.position");

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
    box1.setColor(glm::vec4(1.0, 0.0, 0.0, 1.0));
    //box1.render();

    sphere1.setProjectionMatrix(projection);
    sphere1.setViewMatrix(viewMatrix);
    sphere1.setModelMatrix(box1Transform);
    sphere1.setColor(glm::vec4(0.0, 0.0, 1.0, 1.0));
    //sphere1.render();

    triangle1.setProjectionMatrix(projection);
    triangle1.setViewMatrix(viewMatrix);
    triangle1.setModelMatrix(box1Transform);
    //triangle1.render();

    quad1.setProjectionMatrix(projection);
    quad1.setViewMatrix(viewMatrix);
    quad1.setModelMatrix(box1Transform);
    quad1.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));
    quad1.render();

    line2.setProjectionMatrix(projection);
    line2.setViewMatrix(viewMatrix);
    line2.setModelMatrix(box1Transform);
    //line2.render(Lines::LINE_STRIP);

    cylinder.setProjectionMatrix(projection);
    cylinder.setViewMatrix(viewMatrix);
    cylinder.setModelMatrix(box1Transform);
    cylinder.enableWireMode();
    cylinder.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));
    //cylinder.render();
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
    else if(e->button() == Qt::MouseButton::MidButton)
        middleMouseStatus = true;
}

void GLWidget::mouseReleaseEvent(QMouseEvent* e){
    if(e->button() == Qt::MouseButton::LeftButton)
        leftMouseStatus = false;
    else if(e->button() == Qt::MouseButton::RightButton)
        rightMouseStatus = false;
    else if(e->button() == Qt::MouseButton::MidButton)
        middleMouseStatus = false;
}

void GLWidget::wheelEvent(QWheelEvent* event){
    std::static_pointer_cast<OrbitCamera>(camera)->scrollMoveCamera(event->delta(), dt);
}
