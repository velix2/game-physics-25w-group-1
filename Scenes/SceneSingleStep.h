#include "Scene.h"
#include "Common.h"

class SceneSingleStep : public Scene
{
    TempField temp_field;
    float thermal_diffusivity = 0.1f;
    float delta_t = 0.1f;

    void init() override;
};