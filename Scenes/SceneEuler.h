#include "Scene.h"
#include "Common.h"

class SceneEuler : public Scene
{
    PointMass p1 = PointMass(glm::vec3(0, 0, 0), glm::vec3(-1, 0, 0), 10),
              p2 = PointMass(glm::vec3(0, 2, 0), glm::vec3(1, 0, 0), 10);
    Spring s = Spring(p1, p2, 1, 40);

    float dt = 0.005;

    // virtual void init() override;
    virtual void onGUI() override;

    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
};