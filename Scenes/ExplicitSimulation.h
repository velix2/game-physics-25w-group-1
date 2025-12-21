#pragma once
#include "Scene.h"
#include <vector>
#include <random>

/// @brief heat equation simulation with explicit euler (FTCS scheme)
class ExplicitSimulation : public Scene
{
    // grid dimensions (interior points only)
    int m = 16;
    int n = 16;

    // physical domain size (unit square)
    float domainWidth = 1.0f;
    float domainHeight = 1.0f;

    // simulation params (adjustable via gui)
    float nu = 0.01f;
    float dt = 0.01f;

    // temperature grid
    std::vector<float> T;

    // random number generation
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    // simulation state
    bool running = false;
    int stepCount = 0;

    float dx() const { return domainWidth / (m + 1); }
    float dy() const { return domainHeight / (n + 1); }

    // get temperature with dirichlet bc
    float getT(int i, int j) const
    {
        if (i < 0 || i >= m || j < 0 || j >= n)
            return 0.0f;
        return T[i * n + j];
    }

    void explicitStep();
    void initRandom();
    void initGaussian();
    void initSineWave();

public:
    ExplicitSimulation() : gen(rd()), dis(-1.0f, 1.0f) {}

    virtual void init() override;
    virtual void simulateStep() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
};

