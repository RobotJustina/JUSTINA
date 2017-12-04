#ifndef COMPOSITEMODEL_H
#define COMPOSITEMODEL_H

#include <map>
#include "abstractmodel.h"

class CompositeModel: public AbstractModel
{
public:
    CompositeModel();
    void addSubModel(int id, std::string name, std::shared_ptr<AbstractModel> subModel);
    void updateSubModel(int id, std::string name, std::shared_ptr<AbstractModel> subModel);
    void render();
    void destroy();
    void setName(std::string name){
        this->name = name;
    }
    std::string getName(){
        return this->name;
    }
    void setId(int id){
        this->id = id;
    }
    int getId(){
        return this->id;
    }
    std::map<int, std::shared_ptr<AbstractModel>> getSubModels(){
        return this->subModels;
    }
    virtual bool rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection);

protected:
    std::map<int, std::shared_ptr<AbstractModel>> subModels;
    std::string name;
    int id;
};

#endif // SCENEMODEL_H
