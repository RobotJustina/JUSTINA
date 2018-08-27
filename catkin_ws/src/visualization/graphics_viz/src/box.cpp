#include "graphics_viz/box.h"

Box::Box()
{
    vertexArray.push_back(Vertex(glm::vec3(0.5, 0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));
    vertexArray.push_back(Vertex(glm::vec3(0.5, -0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));

    vertexArray.push_back(Vertex(glm::vec3(0.5, 0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(1.0, 0.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(0.5, -0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(1.0, 0.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(0.5, -0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(1.0, 0.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(0.5, 0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(1.0, 0.0, 0.0)));

    vertexArray.push_back(Vertex(glm::vec3(0.5, 0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, -1.0)));
    vertexArray.push_back(Vertex(glm::vec3(0.5, -0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, -1.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, -1.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, -1.0)));

    vertexArray.push_back(Vertex(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(-1.0, 0.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(-1.0, 0.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(-1.0, 0.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(-1.0, 0.0, 0.0)));

    vertexArray.push_back(Vertex(glm::vec3(0.5, 0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 1.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(0.5, 0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 1.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 1.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, 1.0, 0.0)));

    vertexArray.push_back(Vertex(glm::vec3(0.5, -0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, -1.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, -1.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, -1.0, 0.0)));
    vertexArray.push_back(Vertex(glm::vec3(0.5, -0.5, -0.5), glm::vec4(), glm::vec2(), glm::vec3(0.0, -1.0, 0.0)));

    GLuint indexArray[36] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11, 12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23};
    index.insert(index.begin(), indexArray, indexArray + sizeof(indexArray) / sizeof(GLuint));

    aabb = AABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f));
    typeModel = TypeModel::BOX;
}

Box::~Box(){
}

bool Box::rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection){
    glm::quat oX = glm::angleAxis<float>(glm::radians(orientation.x), glm::vec3(1.0, 0.0, 0.0));
    glm::quat oY = glm::angleAxis<float>(glm::radians(orientation.y), glm::vec3(0.0, 1.0, 0.0));
    glm::quat oZ = glm::angleAxis<float>(glm::radians(orientation.z), glm::vec3(0.0, 0.0, 1.0));
    glm::quat ori = oZ * oY * oX;
    glm::mat4 t = glm::translate(position) * glm::toMat4(ori) * glm::scale(scale);
    glm::mat4 tinv = glm::inverse(t);

    glm::vec3 initT = glm::vec3(tinv * glm::vec4(init, 1.0f));
    glm::vec3 endT = glm::vec3(tinv * glm::vec4(end, 1.0f));
    glm::vec3 dirT = endT - initT;

    float tmin = -FLT_MAX, tmax = FLT_MAX;

    for(int i = 0; i < 3; i++){
        if (fabs(dirT[i]) < 0.01){
            if(!(initT[i] >= aabb.mins[i] && initT[i] <= aabb.maxs[i]))
                return false;
        }
        else{
            float ood = 1.0f / dirT[i];
            float t1 = (aabb.mins[i] - initT[i]) * ood;
            float t2 = (aabb.maxs[i] - initT[i]) * ood;

            if (t1 > t2) {
                float aux = t1;
                t1 = t2;
                t2 = aux;
            }

            if (t1 > tmin)
                tmin = t1;
            if (t2 < tmax)
                tmax = t2;

            if (tmin > tmax)
                return false;
        }
    }

    glm::vec3 qT = initT + dirT * tmin;
    glm::vec3 q = glm::vec3(t * glm::vec4(qT, 1.0f));

    intersection = q;

    return true;
}
