#pragma once
#include "Scene.h"
#include <vector>
#include <random>
#include "util/pcgsolver.h"

/// @brief interactive heat equation simulation with both explicit and implicit schemes
class InteractiveSimulation : public Scene
{
    // grid dimensions (interior points only) - adjustable
    int m = 32;
    int n = 32;

    // physical domain size - adjustable
    float domainWidth = 1.0f;
    float domainHeight = 1.0f;

    // simulation params - adjustable
    float nu = 0.01f;
    float dt = 0.01f;

    // temperature grid
    std::vector<float> T;

    // pcg solver for implicit scheme
    SparsePCGSolver<double> solver;

    // random number generation
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    // simulation state
    bool running = false;
    int stepCount = 0;
    bool useImplicit = false;

    // solver stats (for implicit)
    int lastIterations = 0;
    double lastResidual = 0.0;

    // mouse interaction
    float brushTemperature = 1.0f;
    float brushRadius = 0.05f;
    bool painting = false;

    // camera info for mouse picking
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);

    float dx() const { return domainWidth / (m + 1); }
    float dy() const { return domainHeight / (n + 1); }

    // convert 2d index to 1d
    int idx(int i, int j) const { return i * n + j; }

    // get temperature with dirichlet bc
    float getT(int i, int j) const
    {
        if (i < 0 || i >= m || j < 0 || j >= n)
            return 0.0f;
        return T[i * n + j];
    }

    void explicitStep();
    void implicitStep();
    void resizeGrid();
    void initRandom();
    void initGaussian();
    void initSineWave();
    void initZero();
    void paintAtPosition(float normX, float normY);

public:
    InteractiveSimulation() : gen(rd()), dis(-1.0f, 1.0f) {}

    virtual void init() override;
    virtual void simulateStep() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
};

