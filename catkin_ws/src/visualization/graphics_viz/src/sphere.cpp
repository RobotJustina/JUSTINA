#include "graphics_viz/sphere.h"

Sphere::Sphere(int slices, int stacks, float ratio){
    vertexArray.resize(((slices + 1) * (stacks + 1)));
    index.resize((slices * stacks + slices) * 6);
    for (int i = 0; i <= stacks; ++i) {
        float V = i / (float) stacks;
        float phi = V * M_PI;

        for (int j = 0; j <= slices; ++j) {
            float U = j / (float) slices;
            float theta = U * M_PI * 2.0;

            float X = cos(theta) * sin(phi);
            float Y = cos(phi);
            float Z = sin(theta) * sin(phi);
            vertexArray[i * (slices + 1) + j].m_pos = ratio * glm::vec3(X, Y, Z);
            vertexArray[i * (slices + 1) + j].m_color = glm::vec4();
            vertexArray[i * (slices + 1) + j].m_tex = glm::vec2(U, V);
            vertexArray[i * (slices + 1) + j].m_normal = glm::vec3(X, Y, Z);
        }
    }

    for (int i = 0; i < slices * stacks + slices; ++i) {
        index[i * 6] = i;
        index[i * 6 + 1] = i + slices + 1;
        index[i * 6 + 2] = i + slices;
        index[i * 6 + 3] = i + slices + 1;
        index[i * 6 + 4] = i;
        index[i * 6 + 5] = i + 1;
    }
    sbb = SBB(position, scale.x * ratio);
    typeModel = TypeModel::SPHERE;
}

Sphere::~Sphere(){
}

bool Sphere::rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection){
    sbb = SBB(position, scale.x * sbb.ratio);
    glm::vec3 dir = glm::normalize(end - init);
    // Vector del Origen del rayo al centro de la esfera.
    glm::vec3 vDirToSphere = sbb.c - init;

    // Distancia del origen al destino del rayo.
    float fLineLength = glm::distance(init, end);

    // Proyección escalar de vDirToSphere sobre la direccion del rayo.
    float t = glm::dot(vDirToSphere, dir);

    glm::vec3 vClosestPoint;
    // Si la distancia proyectada del origen es menor o igual que cero
    // Significa que el punto mas cercano al centro es el origen.
    if (t <= 0.0f)
        vClosestPoint = init;
    // Si la proyección escalar del origen es mayor a distancia del origen
    // al destino, el punto mas cercano es el destino.
    else if (t >= fLineLength)
        vClosestPoint = end;
    // En caso contrario de calcula el punto sobre la linea usando t.
    else
        vClosestPoint = init + dir * t;

    // Se pureba si el punto mas cercao esta contenido en el radio de la esfera.
    bool test = glm::distance(sbb.c, vClosestPoint) <= sbb.ratio;

    if(!test)
        return false;

    float xd = dir.x, yd = dir.y, zd = dir.z;

    float A = xd * xd + yd * yd + zd * zd;
    float B = 2 * (xd * (init.x - sbb.c.x) + yd * (init.y - sbb.c.y) + zd * (init.z - sbb.c.z));
    float C = (init.x - sbb.c.x) * (init.x - sbb.c.x) + (init.y - sbb.c.y) * (init.y - sbb.c.y) + (init.z - sbb.c.z) * (init.z - sbb.c.z) - sbb.ratio * sbb.ratio;

    float disc = B * B - 4 * A * C;

    if(disc < 0)
        return false;

    float t1 = (-B + sqrt(disc)) / (2 * A);
    float t2 = (-B - sqrt(disc)) / (2 * A);

    if (t1 < 0)
        return false;

    float tmin = t2;
    if(t1 < tmin)
        tmin = t1;

    intersection = init + tmin * dir;
    return true;
}
