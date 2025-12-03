#include "Scene.h"

class SceneComplex : public Scene
{
    bool should_run = true;
    float delta_t = 0.01f;

    float gravity = 9.81f;

    float coefficient_of_restitution = 1;

    // Interaction stuff
    float offset_user_force_to_cm = 1.0f;
    float input_force_strength = 1.0f;

    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 cameraPos = glm::vec3(0);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);

    const glm::vec4 FLOOR_COLOR = glm::vec4(0.2f, 0.2f, 0.2f, 1);

    std::vector<Rigidbody> rbs = std::vector<Rigidbody>();
    std::vector<glm::vec4> rb_colors = std::vector<glm::vec4>();

    void init() override;
    void onDraw(Renderer &renderer) override;
    void simulateStep() override;
    void onGUI() override;

    void SpawnRigidbody();
};