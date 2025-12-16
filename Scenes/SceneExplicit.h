#include "Scene.h"
#include "Common.h"

class SceneExplicit : public Scene
{
    TempField temp_field = TempField(0.1f, generateRandomField(16, 16, -250.0f, 250.0f), 0, 1, 0, 1);
    float delta_t = 0.01;
    float rendering_horizontal_scale = 5.0f;
    float rendering_vertical_scale = 0.01f;
    float min_temp = -500.0f, max_temp = 500.0f;
    bool is_running = false;

    void simulateStep() override;
    void onGUI() override;
    void onDraw(Renderer &r) override;
};