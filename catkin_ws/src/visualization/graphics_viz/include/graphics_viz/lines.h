#ifndef LINE_H
#define LINE_H
#include "graphics_viz/abstractmodel.h"

class Lines: public AbstractModel
{
public:
    enum LINES_MODE{
        LINES, LINE_STRIP
    };

    Lines();
    ~Lines();

    void init(glm::vec3 v1, glm::vec3 v2);
    void init(std::vector<glm::vec3> lines);
    void render(LINES_MODE linesMode = LINES, glm::mat4 parentTrans = glm::mat4());
    void update();
    virtual bool rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection);
public:
    std::vector<VertexColor> vertexArray;
};

#endif // LINE_H
