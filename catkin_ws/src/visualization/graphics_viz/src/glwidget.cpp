#include <GL/glew.h>

#include "graphics_viz/glwidget.h"

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
std::shared_ptr<Texture> map;
glm::mat4 projection;

std::shared_ptr<Camera> camera(new OrbitCamera());

Sphere lightSphere(20, 20);
Quad quadMap(glm::vec3(-0.5, -0.5, 0.0), glm::vec3(-0.5, 0.5, 0.0), glm::vec3(0.5, 0.5, 0.0), glm::vec3(0.5, -0.5, 0.0));
Grid grid(5, 1.0);
Cylinder cylinderCamera(10, 10, 0.2, 0.2);
std::vector<std::shared_ptr<Lines>> rays;

glm::vec3 lightPos(0.0f, 0.0f, 20.0f);

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

    map = std::shared_ptr<Texture>(new Texture(GL_TEXTURE_2D, path + "/Textures/bioroboanexo4.pgm"));
    map->load();

    projection = glm::perspective(45.0f,
                                  (GLfloat) this->width()
                                  / (GLfloat) this->height(), 0.1f, 100.0f);

//    float n = 0.1f;
//    float f = 100.0f;
//    float fov = 45.0f;
//    float aspectRatio = (GLfloat) this->width() / (GLfloat) this->height();
//    float width = n * atan(fov/2) * aspectRatio;
//    float hight = n * atan(width) / aspectRatio;
//    projection[0][0] =  2 * n / (width + width);
//    projection[1][1] =  2 * n / (hight + hight);
//    projection[2][2] =  -(f + n) / (f - n);
//    projection[2][3] = -1;
//    projection[3][2] = -(2 * f * n) / (f - n);


    //projection = glm::ortho(-1.0f, 1.0f,-1.0f, 1.0f, 0.1f, 100.0f);

    //camera->setCameraPos(glm::vec3(0.0, 0.0, 3.0));

    int w = map->getWidth();
    int h = map->getHeight();
    grid = Grid( w >= h ? w * 0.05 / 2.0 : h * 0.05 / 2.0, 1.0);
    grid.init();
    grid.setShader(shadersC);
    grid.setColor(glm::vec4(0.0, 0.0, 1.0, 1.0));

    lightSphere.init();
    lightSphere.setShader(shadersC);


    quadMap.init();
    quadMap.setTexture(map);
    quadMap.setUVTexture(glm::vec2(0.0f, 1.0f), glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 1.0f));
    quadMap.setShader(shadersT);

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

    glm::vec4 viewport = glm::vec4(0.0f, 0.0f, this->width(), this->height());

    glm::mat4 viewMatrix = camera->getViewMatrix();
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

    lightSphere.setProjectionMatrix(projection);
    lightSphere.setViewMatrix(viewMatrix);
    lightSphere.setPosition(lightPos);
    lightSphere.setScale(glm::vec3(0.1, 0.1, 0.1));
    lightSphere.setColor(glm::vec4(lightColor, 1.0));
    lightSphere.render();

    int w = map->getWidth() * 0.05;
    int h = map->getHeight() * 0.05;
    quadMap.setProjectionMatrix(projection);
    quadMap.setViewMatrix(viewMatrix);
    quadMap.setScale(glm::vec3(w, h, 1.0));
    quadMap.setColor(glm::vec4(1.0, 1.0, 0.0, 1.0));
    quadMap.render();

    grid.setProjectionMatrix(projection);
    grid.setViewMatrix(viewMatrix);
    grid.render();

    glm::vec3 cameraTarget = std::static_pointer_cast<OrbitCamera>(camera)->getCameraTarget();
    cylinderCamera.setProjectionMatrix(projection);
    cylinderCamera.setViewMatrix(viewMatrix);
    cylinderCamera.setPosition(cameraTarget);
    cylinderCamera.setScale(glm::vec3(1.0, 1.0, 0.01));
    cylinderCamera.render();

    for(std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.begin(); it != container.end(); it++){
        it->second->setProjectionMatrix(projection);
        it->second->setViewMatrix(viewMatrix);
        it->second->render();
    }


    if(leftMouseStatus){
        prevInitRay = initRay;
        prevEndRay = endRay;
        initRay = glm::unProject(glm::vec3(lastx, this->height()- lasty, 0.0), camera->getViewMatrix(), projection, viewport);
        endRay = glm::unProject(glm::vec3(lastx, this->height()- lasty, 1.0), camera->getViewMatrix(), projection, viewport);

        if(numClicks == 2){
            initRay = glm::unProject(glm::vec3(lastx, this->height()- lasty, 0.0), camera->getViewMatrix(), projection, viewport);
            endRay = glm::unProject(glm::vec3(lastx, this->height()- lasty, 1.0), camera->getViewMatrix(), projection, viewport);
            numClicks = 0;
            glm::vec3 pick1;
            bool picking1 = false;
            glm::vec3 pick2;
            bool picking2 = false;
            picking1 = quadMap.rayPicking(prevInitRay, prevEndRay, pick1);
            picking2 = quadMap.rayPicking(initRay, endRay, pick2);
            std::cout << "pick1:" << "(" << pick1.x << "," << pick1.y << "," << pick1.z << ")" << std::endl;
            std::cout << "pick2:" << "(" << pick2.x << "," << pick2.y << "," << pick2.z << ")" << std::endl;
            if(picking1 && picking2){
                QModelIndex index;
                emit addNewWall(index);
                if(index.internalId() != 0){
                    std::shared_ptr<AbstractModel> model1(new Box());
                    float l = glm::distance(pick1, pick2);
                    float h = 2.0f;
                    glm::vec3 position = (pick1 + pick2) / 2.0f;
                    position.z = h / 2.0f;
                    glm::vec3 p = glm::normalize(pick2 - pick1);
                    float angle;
                    if(fabs(p.y) < 0.1)
                        angle = 0;
                    else
                        angle = atan2(p.y, p.x);
                    /*if(angle < 0)
                        angle += 2 * M_PI;*/
                    /*float r = sqrt(pow(p.x, 2) + pow(p.y, 2));
                    float angle = acos(p.x / r);*/
                    angle = angle * 180 / M_PI;
                    std::cout << "Angle:" << angle << std::endl;
                    model1->setOrientation(glm::vec3(0.0, 0.0, angle));
                    model1->setPosition(position);
                    model1->setScale(glm::vec3(l, 0.03f, h));
                    model1->setShader(shadersLmc);
                    model1->init();
                    container[index.parent().internalId()]->addSubModel(index.internalId(), "", model1);
                    emit updateTreeView(index);
                }
            }
        }
    }

    /*if(generateRay){
        std::shared_ptr<Lines> ray(new Lines());
        ray->init(initRay, endRay);
        ray->setShader(shadersC);
        rays.push_back(ray);
        generateRay = false;
    }

    for(int i = 0; i < rays.size(); i++){
        rays[i]->setProjectionMatrix(projection);
        rays[i]->setViewMatrix(viewMatrix);
        rays[i]->render();
    }*/

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
        float dx = e->x() - lastx;
        float dy = lasty - e->y();

        glm::vec3 rightAxis = glm::normalize(glm::cross(camera->getCameraUp(), camera->getCameraFront()));
        glm::vec3 front = glm::normalize(glm::cross(camera->getCameraUp(), rightAxis));

        glm::vec3 cameraTarget = std::static_pointer_cast<OrbitCamera>(camera)->getCameraTarget();
        float sensitivity = std::static_pointer_cast<OrbitCamera>(camera)->getSensitivity();
        cameraTarget += rightAxis * dx * (float) dt * sensitivity;
        cameraTarget += front * dy * (float) dt * sensitivity;

        std::static_pointer_cast<OrbitCamera>(camera)->setCameraTarget(cameraTarget);
        camera->updateCamera();
    }
    lastx = e->x();
    lasty = e->y();
}

void GLWidget::mousePressEvent(QMouseEvent* e){
    lastx = e->x();
    lasty = e->y();
    if(e->button() == Qt::MouseButton::LeftButton){
        leftMouseStatus = true;
        generateRay = true;
    }else if(e->button() == Qt::MouseButton::RightButton)
        rightMouseStatus = true;
    else if(e->button() == Qt::MouseButton::MiddleButton)
        middleMouseStatus = true;
}

void GLWidget::mouseReleaseEvent(QMouseEvent* e){
    if(e->button() == Qt::MouseButton::LeftButton){
        leftMouseStatus = false;
        generateRay = false;
        numClicks++;
    }else if(e->button() == Qt::MouseButton::RightButton)
        rightMouseStatus = false;
    else if(e->button() == Qt::MouseButton::MiddleButton)
        middleMouseStatus = false;
}

void GLWidget::wheelEvent(QWheelEvent* event){
    if(!middleMouseStatus)
        std::static_pointer_cast<OrbitCamera>(camera)->scrollMoveCamera(event->delta(), dt);
}

void GLWidget::addNewModel(QModelIndex index){
    QModelIndex parentIndex = index.parent();
    if(parentIndex.internalPointer() != nullptr){
        if(parentIndex.parent().internalPointer() != nullptr){
            std::shared_ptr<CompositeModel> compositeModel;
            std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(parentIndex.internalId());
            if(it != container.end()){
                compositeModel = it->second;
                std::shared_ptr<AbstractModel> model1(new Sphere(20, 20, 0.5));
                model1->setShader(shadersLmc);
                model1->init();
                compositeModel->addSubModel(index.internalId(), "", model1);
            }
        }
        else{
            std::shared_ptr<CompositeModel> compositeModel(new CompositeModel());
            container[index.internalId()] = compositeModel;
        }
    }
}
