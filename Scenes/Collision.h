#pragma once
#include "Scene.h"
#include "RigidBody.h"
#include <util/CollisionInfo.h>

class Collision : public Scene {
    RigidBody bodyA;
    RigidBody bodyB;
    
    // simulation params
    float dt = 0.01f;
    bool paused = true;
    float restitution = 0.8f;  // bounciness: 0 = plastic, 1 = elastic
    
    // collision state - only handle collision once per contact
    bool wasColliding = false;
    
    // camera info
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 camRight = glm::vec3(1, 0, 0);
    glm::vec3 camUp = glm::vec3(0, 1, 0);
    
public:
    virtual void init() override;
    virtual void simulateStep() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
    
private:
    void resetBodies();
    bool handleCollision(RigidBody& a, RigidBody& b);
    void handleCollisionSwapped(RigidBody& a, RigidBody& b, const CollisionInfo& info);
};

