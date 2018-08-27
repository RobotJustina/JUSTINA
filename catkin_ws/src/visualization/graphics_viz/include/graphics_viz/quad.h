#ifndef QUAD_H
#define QUAD_H
#include <memory>
#include "abstractmodel.h"
#include "graphics_viz/Texture.h"

class Quad: public AbstractModel
{
public:
    typedef AbstractModel super;
    Quad(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 v4);
    ~Quad();
    void setUVTexture(glm::vec2 uv1, glm::vec2 uv2, glm::vec2 uv3, glm::vec2 uv4);
    void render(glm::mat4 parentTrans = glm::mat4());
    void setTexture(std::shared_ptr<Texture> texture_ptr){
           this->texture_ptr = texture_ptr;
    }
    virtual bool rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection);

private:
    std::shared_ptr<Texture> texture_ptr;
protected:
    glm::vec3 normalPlane;
};

#endif // QUAD_H
