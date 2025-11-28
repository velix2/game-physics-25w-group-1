#include "Scene.h"
#include "Common.h"

class SceneSingleStep : public Scene
{
    virtual void init() override;

    virtual void onDraw(Renderer &renderer) override;
};