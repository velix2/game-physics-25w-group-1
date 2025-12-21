#pragma once
#include "Scene.h"
#include <vector>
#include <random>
#include "util/pcgsolver.h"

/// @brief 3D heat equation simulation
class Simulation3D : public Scene
{
    // grid dimensions (interior points only)
    int m = 8;  // x
    int n = 8;  // y
    int p = 8;  // z

    // physical domain size
    float domainX = 1.0f;
    float domainY = 1.0f;
    float domainZ = 1.0f;

    // simulation params
    float nu = 0.01f;
    float dt = 0.01f;

    // temperature grid T[i,j,k] = T[i * n * p + j * p + k]
    std::vector<float> T;

    // pcg solver
    SparsePCGSolver<double> solver;

    // rng
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    // state
    bool running = false;
    int stepCount = 0;
    bool useImplicit = false;

    // solver stats
    int lastIterations = 0;
    double lastResidual = 0.0;

    // visualization
    int sliceAxis = 2;    // 0=x, 1=y, 2=z
    int sliceIndex = 4;   // which slice to show
    float sphereRadius = 0.03f;
    bool showVolume = false;
    float tempThreshold = 0.1f;
    float heightScale = 0.5f;  // how much temperature affects y position

    float dx() const { return domainX / (m + 1); }
    float dy() const { return domainY / (n + 1); }
    float dz() const { return domainZ / (p + 1); }

    int idx(int i, int j, int k) const { return i * n * p + j * p + k; }

    float getT(int i, int j, int k) const
    {
        if (i < 0 || i >= m || j < 0 || j >= n || k < 0 || k >= p)
            return 0.0f;
        return T[idx(i, j, k)];
    }

    void explicitStep();
    void implicitStep();
    void initZero();
    void initRandom();
    void initGaussian();

public:
    Simulation3D() : gen(rd()), dis(-1.0f, 1.0f) {}

    virtual void init() override;
    virtual void simulateStep() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
};

