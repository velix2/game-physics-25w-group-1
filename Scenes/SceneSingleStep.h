#include "Scene.h"
#include "Common.h"

class SceneSingleStep : public Scene
{
    std::vector<std::vector<float>> temp_field;
    float thermal_diffusivity = 0.1f;
    float delta_t = 0.1f;
    // TODO provide method to calculate deltaX deltaY from domain size and discrete size

    void init() override;
};