#pragma once
#include "Scene.h"
#include "RigidBody.h"

class Simulation : public Scene {
    RigidBody body;
    
    // simulation params
    float dt = 0.01f;
    bool paused = true;
    bool applyInitialForce = true;  // apply the exercise force
    
    // external force (from exercise)
    glm::vec3 externalForce = glm::vec3(1.0f, 1.0f, 0.0f);
    glm::vec3 forcePosition = glm::vec3(0.3f, 0.5f, 0.25f);
    
    // user interaction force
    glm::vec3 userForce = glm::vec3(0);
    float forceMagnitude = 5.0f;
    
    // camera info for mouse drag
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 camRight = glm::vec3(1, 0, 0);
    glm::vec3 camUp = glm::vec3(0, 1, 0);
    
public:
    virtual void init() override;
    virtual void simulateStep() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
    
private:
    void resetBody();
    void handleInput();
};

