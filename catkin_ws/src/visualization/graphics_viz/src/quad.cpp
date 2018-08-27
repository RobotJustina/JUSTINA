#include "graphics_viz/quad.h"

Quad::Quad(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 v4){
    vertexArray.push_back(Vertex(v1, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v2 - v1, v3 - v1))));
    vertexArray.push_back(Vertex(v2, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v3 - v2, v1 - v2))));
    vertexArray.push_back(Vertex(v3, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v4 - v3, v2 - v3))));
    vertexArray.push_back(Vertex(v4, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v1 - v4, v3 - v4))));
    GLuint indexArray[6] = {0, 1, 2, 0, 2, 3};
    index.insert(index.begin(), indexArray, indexArray + sizeof(indexArray) / sizeof(GLuint));
    normalPlane = glm::normalize(glm::cross(v2 - v1, v3 - v1));
    typeModel = TypeModel::QUAD;
}

Quad::~Quad(){
}

void Quad::setUVTexture(glm::vec2 uv1, glm::vec2 uv2, glm::vec2 uv3, glm::vec2 uv4){
    vertexArray[0].m_tex = uv1;
    vertexArray[1].m_tex = uv2;
    vertexArray[2].m_tex = uv3;
    vertexArray[3].m_tex = uv4;
    super::update();
}

void Quad::render(glm::mat4 parentTrans){
    shader_ptr->turnOn();
    if(texture_ptr)
        texture_ptr->bind(GL_TEXTURE0);
    super::render(parentTrans);
    shader_ptr->turnOff();
}

bool Quad::rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection){
    glm::vec3 dir = end - init;
    glm::quat oX = glm::angleAxis<float>(glm::radians(orientation.x), glm::vec3(1.0, 0.0, 0.0));
    glm::quat oY = glm::angleAxis<float>(glm::radians(orientation.y), glm::vec3(0.0, 1.0, 0.0));
    glm::quat oZ = glm::angleAxis<float>(glm::radians(orientation.z), glm::vec3(0.0, 0.0, 1.0));
    glm::quat ori = oZ * oY * oX;
    glm::mat4 t = glm::translate(position) * glm::toMat4(ori) * glm::scale(scale);
    glm::mat4 tinv = glm::inverse(t);
    glm::vec3 n = glm::normalize(glm::vec3(t * glm::vec4(normalPlane, 1.0)));
    //glm::vec3 n = glm::normalize(normalPlane);
    glm::vec3 p0 = init;
    glm::vec3 p1 = end;
    glm::vec3 v0 = glm::vec3(t * glm::vec4(vertexArray[0].m_pos, 1.0));
    //glm::vec3 v0 = vertexArray[0].m_pos;
    float dotNDir = glm::dot(n, dir);
    if(dotNDir == 0){
        //std::cout << "The plane is parallel with the ray" << std::endl;
        float dotNC = glm::dot(n, p0 - v0);
        if(dotNC == 0){
            //std::cout << "The plane and the ray be intersected, The Line is contained in the plane" << std::endl;
            intersection = p0;
            return true;
        }
        //std::cout << "The plane and the ray not be intersected" << std::endl;
        return false;
    }
    //std::cout << "Validate the point of intersection" << std::endl;
    float si = glm::dot(n, v0 - p0) / glm::dot(n, p1 - p0);
    //std::cout << "Si:" << si << std::endl;

    if(!(si >= 0)){
        //std::cout << "The plane and the ray not intersected" << std::endl;
        return false;
    }
    //std::cout << "Compute the point of intersection" << std::endl;
    glm::vec3 pi = init + si * dir;
    //std::cout << "Point intersection:" << pi.x << "," << pi.y << "," << pi.z << std::endl;
    glm::vec3 piinv = glm::vec3(tinv * glm::vec4(pi, 1.0));

    float minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX, maxx = -FLT_MAX, maxy = -FLT_MAX, maxz = -FLT_MAX;
    for(std::vector<Vertex>::iterator it = vertexArray.begin(); it != vertexArray.end(); it++){
        float x = (*it).m_pos.x;
        float y = (*it).m_pos.y;
        float z = (*it).m_pos.z;
        if(x < minx)
            minx = x;
        if(y < miny)
            miny = y;
        if(z < minz)
            minz = z;
        if(x > maxx)
            maxx = x;
        if(y > maxy)
            maxy = y;
        if(z > maxz)
            maxz = z;
    }

    glm::vec3 mins = glm::vec3(minx, miny, minz);
    glm::vec3 maxs = glm::vec3(maxx, maxy, maxz);

    std::cout << "Validate that the point of intersection is into the quad" << std::endl;
    if(piinv.x >= mins.x && piinv.x <=maxs.x && piinv.y >= mins.y && piinv.y <=maxs.y){
        intersection = pi;
        //std::cout << "The point of intersection is into the quad" << std::endl;
        return true;
    }

    //std::cout << "The point of intersection is not into the quad" << std::endl;
    return false;

}
