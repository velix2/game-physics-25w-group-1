#ifndef SCENE4_H
#define SCENE4_H

#include "Scene.h"
#include "Scene1.h"  // reuse MassPoint struct
#include <glm/glm.hpp>
#include <vector>

struct Spring
{
    int point0;  // index of first mass point
    int point1;  // index of second mass point
    float stiffness;
    float restLength;
};

class Scene4 : public Scene
{
private:
    // simulation state
    std::vector<MassPoint> points;
    std::vector<Spring> springs;
    
    // simulation params
    float dt = 0.005f;
    int integrationMethod = 1;  // 0=euler, 1=midpoint, 2=leapfrog
    bool gravityEnabled = true;
    glm::vec3 gravity = glm::vec3(0.0f, 0.0f, -9.81f);
    float groundLevel = -2.0f;
    float damping = 0.98f;  // velocity damping on collision
    float wallBounds = 2.5f;  // walls at +/- this distance
    
    // global spring/mass params (for all points/springs)
    float globalMass = 1.0f;
    float globalStiffness = 100.0f;
    float globalDamping = 0.0f;  // velocity damping (friction)
    
    // interaction
    int selectedPoint = -1;
    bool isPaused = false;
    glm::vec3 windForce = glm::vec3(0.0f);
    bool applyWind = false;
    
    // helper funcs
    glm::vec3 computeSpringForce(const MassPoint& p0, const MassPoint& p1, float k, float L);
    void eulerStep(float timestep);
    void midpointStep(float timestep);
    void leapfrogStep(float timestep);
    void handleCollisions();
    void applyGravity(float timestep);
    void applyGlobalParams();  // update all points/springs with global params
    void applyWindForce();
    void applyVelocityDamping();
    
public:
    virtual void init() override;
    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;
};

#endif

