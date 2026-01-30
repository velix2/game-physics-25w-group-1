#include "Scene.h"
#include "Common.h"

#define PI_2 glm::pi<float>() / 2.0f
#define PI_4 glm::pi<float>() / 4.0f

#define NUM_SPRINGS 10
#define NUM_BODIES 20

class Complex : public Scene
{
    std::vector<Body> bodies = std::vector<Body>();
    std::vector<Spring> springs = std::vector<Spring>();

    float dt = 0.01;
    const float c = 0;
    float gravity = -9.81;
    float friction = 0.4f; // 0.0 = ice, 1.0 = rubber
    bool paused = true;
    bool oneStep = false;
    bool applyForce = true;
    float forceStrength = 1.0f;
    glm::vec3 lastcast1 = glm::vec3(0);
    glm::vec3 lastcast2 = glm::vec3(0);
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::mat4 projMatrix = glm::mat4(1);
    glm::vec3 cameraPosition = glm::vec3(0);
    float cameraNear = 0;
    glm::vec2 windowSize = glm::vec2(0);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);
    virtual void init() override;
    /// This is where you should update the physics of the scene.
    virtual void simulateStep() override;
    /// @brief Draw the scene. Gets called every frame after simulateStep.
    ///
    /// This is where you should call the Renderer draw functions.
    virtual void onDraw(Renderer &renderer) override;
    /// @brief Define the GUI for the scene. Gets called every frame after onDraw.
    virtual void onGUI() override;
};