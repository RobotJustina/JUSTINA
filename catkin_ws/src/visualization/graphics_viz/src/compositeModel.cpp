#include "graphics_viz/compositeModell.h"

CompositeModel::CompositeModel(){
}

void CompositeModel::addSubModel(int id, std::string name, std::shared_ptr<AbstractModel> subModel){
    subModels[id] = subModel;
}

void CompositeModel::updateSubModel(int id, std::string name, std::shared_ptr<AbstractModel> subModel){
    subModels.erase(id);
    addSubModel(id, name, subModel);
}

void CompositeModel::render(){
    glm::mat4 scale = glm::scale(this->scale);
    glm::mat4 translate = glm::translate(this->position);
    glm::quat oX = glm::angleAxis<float>(glm::radians(orientation.x), glm::vec3(1.0, 0.0, 0.0));
    glm::quat oY = glm::angleAxis<float>(glm::radians(orientation.y), glm::vec3(0.0, 1.0, 0.0));
    glm::quat oZ = glm::angleAxis<float>(glm::radians(orientation.z), glm::vec3(0.0, 0.0, 1.0));
    glm::quat ori = oZ * oY * oX;
    glm::mat4 modelMatrix = translate * glm::toMat4(ori) * scale;
    for(std::map<int, std::shared_ptr<AbstractModel>>::iterator it = subModels.begin(); it != subModels.end(); it++){
        it->second->setViewMatrix(viewMatrix);
        it->second->setProjectionMatrix(projectionMatrix);
        it->second->render(modelMatrix);
    }
}

void CompositeModel::destroy(){
    for(std::map<int, std::shared_ptr<AbstractModel>>::iterator it = subModels.begin(); it != subModels.end(); it++)
        it->second->destroy();
}

bool CompositeModel::rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection){

}
