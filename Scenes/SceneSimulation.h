#include "Scene.h"
#include "Common.h"

class SceneSimulation : public Scene
{
    bool should_run = false;
    float delta_t = 0.01f;

    Rigidbody rb;
    
    void init() override;
    void onDraw(Renderer &renderer) override;
    void simulateStep() override;
    void onGUI() override;
};