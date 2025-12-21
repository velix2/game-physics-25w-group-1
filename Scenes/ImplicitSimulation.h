#pragma once
#include "Scene.h"
#include <vector>
#include <random>
#include "util/pcgsolver.h"

/// @brief heat equation simulation with implicit euler (BTCS scheme)
class ImplicitSimulation : public Scene
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

    // pcg solver
    SparsePCGSolver<double> solver;

    // random number generation
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    // simulation state
    bool running = false;
    int stepCount = 0;

    // solver stats
    int lastIterations = 0;
    double lastResidual = 0.0;

    float dx() const { return domainWidth / (m + 1); }
    float dy() const { return domainHeight / (n + 1); }

    // convert 2d index to 1d index for linear system
    int idx(int i, int j) const { return i * n + j; }

    void implicitStep();
    void initRandom();
    void initGaussian();
    void initSineWave();

public:
    ImplicitSimulation() : gen(rd()), dis(-1.0f, 1.0f) {}

    virtual void init() override;
    virtual void simulateStep() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
};

