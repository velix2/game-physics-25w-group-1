#include "Scene.h"
#include "Common.h"
#include <array>
#include <random>

static const char *integratorNames[] = {"Euler", "Midpoint"};

#define MAX_SPRINGS 50
#define MAX_POINTS 50
#define INITIAL_POINTS 10
#define INITIAL_SPRINGS 10

class SceneComplex : public Scene
{
    std::array<PointMass, MAX_POINTS> points;
    int pointCount = 0;
    std::array<Spring, MAX_POINTS> springs;
    int springCount = 0;

    float dt = 0.005;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    enum class Integrator
    {
        Euler,
        Midpoint
    };

    Integrator integrator = Integrator::Midpoint;

    virtual void addPoint();
    virtual void addSpring();

    virtual void init() override;
    virtual void onGUI() override;

    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
};