#include "Scene.h"
#include "Common.h"

class SceneSingleStep : public Scene
{
    std::vector<std::vector<float>> temp_field;

    void init() override;
};