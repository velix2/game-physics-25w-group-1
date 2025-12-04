#pragma once
#include "Scene.h"
#include "RigidBody.h"
#include <vector>
#include <random>

class Complex : public Scene {
    std::vector<RigidBody> bodies;
    
    // simulation params
    float dt = 0.01f;
    bool paused = true;
    float restitution = 0.5f;
    bool gravityEnabled = true;
    glm::vec3 gravity = glm::vec3(0, 0, -9.81f);
    
    // spawner
    float spawnTimer = 0.0f;
    float spawnInterval = 5.0f;
    bool autoSpawn = true;
    std::mt19937 rng{std::random_device{}()};
    
    // collision state tracking
    struct Pair {
        int a, b;
        bool operator<(const Pair& other) const {
            if (a != other.a) return a < other.a;
            return b < other.b;
        }
    };
    std::vector<Pair> activeCollisions;
    
    // camera info for mouse interaction
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 camRight = glm::vec3(1, 0, 0);
    glm::vec3 camUp = glm::vec3(0, 1, 0);
    
public:
    virtual void init() override;
    virtual void simulateStep() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
    
private:
    void resetScene();
    void addBox(glm::vec3 pos, glm::vec3 size, float mass, glm::quat rot = glm::quat(1,0,0,0));
    void spawnRandomCube();
    bool handleCollision(RigidBody& a, RigidBody& b, bool& wasColliding);
    void handleInteraction();
};

