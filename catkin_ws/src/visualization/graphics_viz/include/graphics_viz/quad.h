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
    void render();
    void setTexture(std::shared_ptr<Texture> texture_ptr){
           this->texture_ptr = texture_ptr;
    }

private:
    std::shared_ptr<Texture> texture_ptr;
};

#endif // QUAD_H
