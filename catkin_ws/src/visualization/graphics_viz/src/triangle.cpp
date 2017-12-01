#include "graphics_viz/triangle.h"

Triangle::Triangle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3){
    vertexArray.push_back(Vertex(v1, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v2 - v1, v3 - v1))));
    vertexArray.push_back(Vertex(v2, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v3 - v2, v1 - v2))));
    vertexArray.push_back(Vertex(v3, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v1 - v3, v2 - v3))));
    GLuint indexArray[3] = {0, 1, 2};
    index.insert(index.begin(), indexArray, indexArray + sizeof(indexArray) / sizeof(GLuint));
    typeModel = TypeModel::TRIANGLE;
}

Triangle::~Triangle(){
}

bool Triangle::rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection){
    glm::vec3 edge1, edge2, tvec, pvec, qvec;
    double det, inv_det;
    glm::quat oX = glm::angleAxis<float>(glm::radians(orientation.x), glm::vec3(1.0, 0.0, 0.0));
    glm::quat oY = glm::angleAxis<float>(glm::radians(orientation.y), glm::vec3(0.0, 1.0, 0.0));
    glm::quat oZ = glm::angleAxis<float>(glm::radians(orientation.z), glm::vec3(0.0, 0.0, 1.0));
    glm::quat ori = oZ * oY * oX;
    glm::mat4 tr = glm::translate(position) * glm::toMat4(ori) * glm::scale(scale);
    glm::vec3 vert0 = glm::vec3(tr * glm::vec4(vertexArray[0].m_pos, 1.0));
    glm::vec3 vert1 = glm::vec3(tr * glm::vec4(vertexArray[1].m_pos, 1.0));
    glm::vec3 vert2 = glm::vec3(tr * glm::vec4(vertexArray[2].m_pos, 1.0));
    glm::vec3 dir = end - init;
    // Se encuentran los vectores de dos aristas que contengan a ver0
    edge1 = glm::vec3(vert1 - vert0);
    edge2 = glm::vec3(vert2 - vert0);
    // Se calcula el determiante - se usa para calcular el parametro U.
    pvec = glm::cross(dir, edge2);
    // Si el determinante es muy cercano a cero, rayo cae en el plano
    // que contiene al triángulo.
    det = glm::dot(edge1, pvec);
    if (det > -EPSILON && det < EPSILON)
        return false;
    inv_det = 1.0 / det;
    // Distancia entre el vert0 al origen del rayo.
    tvec = glm::vec3(init - vert0);
    // Calcula el parametro u y prueba su valor.
    float u = glm::dot(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;
    qvec = glm::cross(tvec, edge1);
    // Calcula el parametro v y prueba que este dentro del rango.
    float v = glm::dot(dir, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;
    // Calcula t, interseccion del rayo con el triángulo
    float t = glm::dot(edge2, qvec) * inv_det;
    intersection = init + t * (end - init);
    //std::cout << "Intersection:" << intersection.x << "," << intersection.y << "," << intersection.z << std::endl;
    return true;
}
