#include "Scene.h"
#include "Common.h"

class SceneSimulation : public Scene
{
    bool should_run = false;
    float delta_t = 0.01f;

    std::vector<Rigidbody> rigidbodies;
    std::vector<Point> points;
    std::vector<glm::vec3> forces;

    
    void init() override;
    void onDraw(Renderer &renderer) override;
    void simulateStep() override;
    void onGUI() override;
};