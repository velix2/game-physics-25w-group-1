#include "Scene.h"

class SceneCollision : public Scene
{
    const glm::vec4 RB_1_COLOR = glm::vec4(1, 0, 0, 1);
    const glm::vec4 RB_2_COLOR = glm::vec4(0, 0, 1, 1);

    bool should_run = false;
    float delta_t = 0.01f;

    Rigidbody rb1, rb2;

    void init() override;
    void onDraw(Renderer &renderer) override;
    void simulateStep() override;
    void onGUI() override;
};