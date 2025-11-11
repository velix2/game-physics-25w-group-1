#ifndef SCENE1_H
#define SCENE1_H

#include "Scene.h"
#include <glm/glm.hpp>
#include <vector>

struct MassPoint
{
    glm::vec3 position; //x
    glm::vec3 velocity; //v
    float mass; //m
};

class Scene1 : public Scene
{
private:
    // Spring parameters
    float springStiffness = 40.0f; //$k
    float restLength = 1.0f; //L
    
    // Helper functions
    glm::vec3 computeSpringForce(const MassPoint& p0, const MassPoint& p1, float k, float L);
    void eulerStep(std::vector<MassPoint>& points, float dt);
    void midpointStep(std::vector<MassPoint>& points, float dt);
    
public:
    virtual void init() override;
};

#endif