#ifndef SCENE3_H
#define SCENE3_H

#include "Scene.h"
#include "Scene1.h"  // reuse MassPoint struct
#include <glm/glm.hpp>
#include <vector>

class Scene3 : public Scene
{
private:
    // spring params
    float springStiffness = 40.0f;
    float restLength = 1.0f;
    float dt = 0.005f;  // timestep size, adjustable in gui
    
    // simulation state
    std::vector<MassPoint> points;
    
    // helper funcs
    glm::vec3 computeSpringForce(const MassPoint& p0, const MassPoint& p1, float k, float L);
    void midpointStep(float timestep);
    
public:
    virtual void init() override;
    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;
};

#endif

