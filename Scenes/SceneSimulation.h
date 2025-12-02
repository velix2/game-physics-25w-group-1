#include "Scene.h"

class SceneSimulation : public Scene
{
    bool should_run = false;
    float delta_t = 0.01f;

    // Interaction stuff
    float offset_user_force_to_cm = 1.0f;
    float input_force_strength = 0.1f;
    bool is_dragging_with_mouse = false;
    
    Rigidbody rb;

    void init() override;
    void onDraw(Renderer &renderer) override;
    void simulateStep() override;
    void onGUI() override;

    
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 cameraPos = glm::vec3(0);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);
};