#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QStringList headers;
    headers << tr("Name");
    TreeModel *model = new TreeModel(headers, "");
    this->ui->composeTreeView->setModel(model);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setQtRosNode(QtRosNode *qtRosNode){
    this->qtRosNode = qtRosNode;
    connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsSlot()));
    connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    connect(qtRosNode, SIGNAL(updateGraphics()), this->ui->glwidget, SLOT(updateGL()));
    connect(this, &MainWindow::addNewModel, this->ui->glwidget, &GLWidget::addNewModel);
    connect(this->ui->glwidget, &GLWidget::addNewWall, this, &MainWindow::addNewWall);
    connect(this->ui->glwidget, &GLWidget::updateTreeView, this, &MainWindow::on_composeTreeView_clicked);
//    this->ui->groupBoxPosition->hide();
//    this->ui->groupBoxColor->hide();
//    this->ui->groupBoxScale->hide();
    this->ui->groupBoxEdit->hide();

    for(int i = 0; i < AbstractModel::TypeModel::NUM_MODELS; i++){
        std::string value;
        switch (i) {
        case AbstractModel::TypeModel::TRIANGLE:
            value = "Triangle";
            break;
        case AbstractModel::TypeModel::SPHERE:
            value = "Sphere";
            break;
        case AbstractModel::TypeModel::QUAD:
            value = "Quad";
            break;
        case AbstractModel::TypeModel::BOX:
            value = "Box";
            break;
        case AbstractModel::TypeModel::CYLINDER:
            value = "Cylinder";
            break;
        default:
            break;
        }
        this->ui->shapeType->insertItem(i, QString(value.data()));
    }

}

void MainWindow::closeEvent(QCloseEvent *event){
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
}

void MainWindow::updateGraphicsSlot(){
}

void MainWindow::on_delete_2_pressed()
{

}

void MainWindow::on_copy_pressed()
{

}

void MainWindow::on_rename_pressed()
{

}

void MainWindow::on_add_pressed()
{
    QModelIndex index = this->ui->composeTreeView->selectionModel()->currentIndex();

    int depth = getLevelTreeView(index);

    if(depth <= 2){
        QAbstractItemModel *model = this->ui->composeTreeView->model();

        if (model->columnCount(index) == 0) {
            if (!model->insertColumn(0, index))
                return;
        }

        if (!model->insertRow(0, index))
            return;

        for (int column = 0; column < model->columnCount(index); ++column) {
            QModelIndex child = model->index(0, column, index);
            model->setData(child, QVariant("Model 1"), Qt::EditRole);
        }
        this->ui->composeTreeView->selectionModel()->setCurrentIndex(model->index(0, 0, index),
                                                                     QItemSelectionModel::ClearAndSelect);
    }
    currIndex = this->ui->composeTreeView->selectionModel()->currentIndex();
    depth = getLevelTreeView(currIndex);
    if(depth == 1){
        this->ui->groupBoxEdit->hide();
    }
    else if(depth == 2 || depth == 3){
        this->ui->groupBoxEdit->show();
        if(depth == 2){
            this->ui->shapeType->setEnabled(false);
            this->ui->groupBoxColor->hide();
        }else{
            this->ui->shapeType->setEnabled(true);
            this->ui->groupBoxColor->show();
        }
        emit addNewModel(currIndex);
        this->ui->shapeType->setCurrentIndex(AbstractModel::TypeModel::SPHERE);
    }
}

int MainWindow::getLevelTreeView(QModelIndex index){
    QModelIndex currNode = index;
    int depth = 0;
    do{
        currNode = currNode.parent();
        depth++;
    }while(currNode.internalPointer() != nullptr);
    return depth;
}

void MainWindow::on_composeTreeView_clicked(const QModelIndex &index)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;

    int depth = getLevelTreeView(index);
    currIndex = index;
    if(depth == 1)
        this->ui->groupBoxEdit->hide();
    else if(depth == 2 || depth == 3){
        this->ui->groupBoxEdit->show();
        if(depth == 2){
            this->ui->groupBoxColor->hide();
            std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(index.internalId());
            if(it != container.end()){
                compositeModel = it->second;
                /*this->ui->colorRed->setValue(compositeModel->getColor().x * 255);
                this->ui->colorGreen->setValue(compositeModel->getColor().y * 255);
                this->ui->colorBlue->setValue(compositeModel->getColor().z * 255);*/
                this->ui->positionX->setValue(compositeModel->getPosition().x);
                this->ui->positionY->setValue(compositeModel->getPosition().y);
                this->ui->positionZ->setValue(compositeModel->getPosition().z);
                this->ui->orientationX->setValue(compositeModel->getOrientation().x);
                this->ui->orientationY->setValue(compositeModel->getOrientation().y);
                this->ui->orientationZ->setValue(compositeModel->getOrientation().z);
                this->ui->scaleX->setValue(compositeModel->getScale().x);
                this->ui->scaleY->setValue(compositeModel->getScale().y);
                this->ui->scaleZ->setValue(compositeModel->getScale().z);
            }
            this->ui->shapeType->setEnabled(false);
        }else{
            this->ui->groupBoxColor->show();
            std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(index.parent().internalId());
            if(it != container.end()){
                compositeModel = it->second;
                std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
                std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
                if(it2 != subModels.end()){
                    model = it2->second;
                    this->ui->colorRed->setValue(model->getColor().x * 255);
                    this->ui->colorGreen->setValue(model->getColor().y * 255);
                    this->ui->colorBlue->setValue(model->getColor().z * 255);
                    this->ui->colorAlpha->setValue(model->getColor().w * 255);
                    this->ui->positionX->setValue(model->getPosition().x);
                    this->ui->positionY->setValue(model->getPosition().y);
                    this->ui->positionZ->setValue(model->getPosition().z);
                    this->ui->orientationX->setValue(model->getOrientation().x);
                    this->ui->orientationY->setValue(model->getOrientation().y);
                    this->ui->orientationZ->setValue(model->getOrientation().z);
                    this->ui->scaleX->setValue(model->getScale().x);
                    this->ui->scaleY->setValue(model->getScale().y);
                    this->ui->scaleZ->setValue(model->getScale().z);
                    this->ui->shapeType->setCurrentIndex(model->getTypeModel());
                }
            }
            this->ui->shapeType->setEnabled(true);
        }
    }
}

void MainWindow::on_colorRed_valueChanged(int arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
    if(it != container.end()){
        compositeModel = it->second;
        std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
        std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
        if(it2 != subModels.end()){
            model = it2->second;
            glm::vec4 color = model->getColor();
            model->setColor(glm::vec4((float) (arg1 / 255.0f), color.y, color.z, color.w));
        }
    }
}

void MainWindow::on_positionX_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 position = compositeModel->getPosition();
            compositeModel->setPosition(glm::vec3((float) arg1, position.y, position.z));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 position = model->getPosition();
                model->setPosition(glm::vec3((float) arg1, position.y, position.z));
            }
        }
    }
}

void MainWindow::on_scaleX_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 scale = compositeModel->getScale();
            compositeModel->setScale(glm::vec3((float) arg1, scale.y, scale.z));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 scale = model->getScale();
                model->setScale(glm::vec3((float) arg1, scale.y, scale.z));
            }
        }
    }
}

void MainWindow::on_orientationX_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 ori = compositeModel->getOrientation();
            compositeModel->setOrientation(glm::vec3((float) arg1, ori.y, ori.z));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 ori = model->getOrientation();
                model->setOrientation(glm::vec3((float) arg1, ori.y, ori.z));
            }
        }
    }
}

void MainWindow::on_orientationY_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 ori = compositeModel->getOrientation();
            compositeModel->setOrientation(glm::vec3(ori.x, (float) arg1, ori.z));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 ori = model->getOrientation();
                model->setOrientation(glm::vec3(ori.x, (float) arg1, ori.z));
            }
        }
    }
}

void MainWindow::on_orientationZ_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 ori = compositeModel->getOrientation();
            compositeModel->setOrientation(glm::vec3(ori.x, ori.y, (float) arg1));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 ori = model->getOrientation();
                model->setOrientation(glm::vec3(ori.x, ori.y, (float) arg1));
            }
        }
    }
}

void MainWindow::on_scaleY_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 scale = compositeModel->getScale();
            compositeModel->setScale(glm::vec3(scale.x, (float) arg1, scale.z));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 scale = model->getScale();
                model->setScale(glm::vec3(scale.x, (float) arg1, scale.z));
            }
        }
    }
}


void MainWindow::on_scaleZ_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 scale = compositeModel->getScale();
            compositeModel->setScale(glm::vec3(scale.x, scale.y, (float) arg1));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 scale = model->getScale();
                model->setScale(glm::vec3(scale.x, scale.y, (float) arg1));
            }
        }
    }
}

void MainWindow::on_positionY_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 position = compositeModel->getPosition();
            compositeModel->setPosition(glm::vec3(position.x, (float) arg1, position.z));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 position = model->getPosition();
                model->setPosition(glm::vec3(position.x, (float) arg1, position.z));
            }
        }
    }
}

void MainWindow::on_positionZ_valueChanged(double arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    int depth = getLevelTreeView(currIndex);
    if(depth == 2){
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.internalId());
        if(it != container.end()){
            compositeModel = it->second;
            glm::vec3 position = compositeModel->getPosition();
            compositeModel->setPosition(glm::vec3(position.x, position.y, (float) arg1));
        }
    }else{
        std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
        if(it != container.end()){
            compositeModel = it->second;
            std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
            std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
            if(it2 != subModels.end()){
                model = it2->second;
                glm::vec3 position = model->getPosition();
                model->setPosition(glm::vec3(position.x, position.y, (float) arg1));
            }
        }
    }
}

void MainWindow::on_colorGreen_valueChanged(int arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
    if(it != container.end()){
        compositeModel = it->second;
        std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
        std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
        if(it2 != subModels.end()){
            model = it2->second;
            glm::vec4 color = model->getColor();
            model->setColor(glm::vec4(color.x, (float) (arg1 / 255.0f), color.z, color.w));
        }
    }
}

void MainWindow::on_colorBlue_valueChanged(int arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
    if(it != container.end()){
        compositeModel = it->second;
        std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
        std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
        if(it2 != subModels.end()){
            model = it2->second;
            glm::vec4 color = model->getColor();
            model->setColor(glm::vec4(color.x, color.y, (float) (arg1 / 255.0f), color.w));
        }
    }
}

void MainWindow::on_colorAlpha_valueChanged(int arg1)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> model;
    std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
    if(it != container.end()){
        compositeModel = it->second;
        std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
        std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
        if(it2 != subModels.end()){
            model = it2->second;
            glm::vec4 color = model->getColor();
            model->setColor(glm::vec4(color.x, color.y, color.z, (float) (arg1 / 255.0f)));
        }
    }
}

void MainWindow::on_shapeType_currentIndexChanged(int index)
{
    std::map<int, std::shared_ptr<CompositeModel>> container = this->ui->glwidget->getContainer();
    std::shared_ptr<CompositeModel> compositeModel;
    std::shared_ptr<AbstractModel> oldModel;
    std::map<int, std::shared_ptr<CompositeModel>>::iterator it = container.find(currIndex.parent().internalId());
    if(it != container.end()){
        compositeModel = it->second;
        std::map<int, std::shared_ptr<AbstractModel>> subModels = compositeModel->getSubModels();
        std::map<int, std::shared_ptr<AbstractModel>>::iterator it2 = subModels.find(currIndex.internalId());
        if(it2 != subModels.end()){
            std::shared_ptr<AbstractModel> newModel;
            oldModel = it2->second;
            if(oldModel->getTypeModel() != index){
                switch (index) {
                case AbstractModel::TypeModel::TRIANGLE:
                    newModel = std::shared_ptr<AbstractModel>(new Triangle(glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.5f, -0.5f, 0.0f), glm::vec3(0.0f, 0.5f, 0.0f)));
                    break;
                case AbstractModel::TypeModel::SPHERE:
                    newModel = std::shared_ptr<AbstractModel>(new Sphere(20.0f, 20.0f, 0.5f));
                    break;
                case AbstractModel::TypeModel::QUAD:
                    newModel = std::shared_ptr<AbstractModel>(new Quad(glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.5f, -0.5f, 0.0f), glm::vec3(0.5f, 0.5f, 0.0f), glm::vec3(-0.5f, 0.5f, 0.0f)));
                    break;
                case AbstractModel::TypeModel::BOX:
                    newModel = std::shared_ptr<AbstractModel>(new Box());
                    break;
                case AbstractModel::TypeModel::CYLINDER:
                    newModel = std::shared_ptr<AbstractModel>(new Cylinder(20, 20, 0.5, 0.5, 1.0));
                    break;
                default:
                    break;
                }
                newModel->setShader(oldModel->getShader());
                newModel->init();
                newModel->setColor(oldModel->getColor());
                newModel->setPosition(oldModel->getPosition());
                newModel->setOrientation(oldModel->getOrientation());
                newModel->setScale(oldModel->getScale());
                compositeModel->updateSubModel(currIndex.internalId(), "", newModel);
            }
        }
    }
}

void MainWindow::addNewWall(QModelIndex &indexWall){
    QModelIndex index = this->ui->composeTreeView->selectionModel()->currentIndex();

    int depth = getLevelTreeView(index);

    if(depth == 2){
        QAbstractItemModel *model = this->ui->composeTreeView->model();

        if (model->columnCount(index) == 0) {
            if (!model->insertColumn(0, index))
                return;
        }

        if (!model->insertRow(0, index))
            return;

        for (int column = 0; column < model->columnCount(index); ++column) {
            QModelIndex child = model->index(0, column, index);
            model->setData(child, QVariant("Model 1"), Qt::EditRole);
        }
        this->ui->groupBoxEdit->show();
        this->ui->shapeType->setEnabled(true);
        this->ui->groupBoxColor->show();
        indexWall = model->index(0, 0, index);
        //this->ui->composeTreeView->selectionModel()->setCurrentIndex(indexWall,
                                                                     //QItemSelectionModel::ClearAndSelect);
        //on_composeTreeView_clicked(indexWall);
    }
}
