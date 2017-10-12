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
}

Sphere::~Sphere(){
}
