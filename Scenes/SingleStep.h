#pragma once
#include "Scene.h"
#include <vector>
#include <iostream>

/// @brief heat equation single step test with explicit euler
class SingleStep : public Scene
{
    // grid dimensions (interior points only)
    int m = 3; // x direction
    int n = 6; // y direction

    // physical domain size
    float domainWidth = 2.0f;
    float domainHeight = 4.0f;

    // simulation params
    float nu = 0.1f;  // thermal diffusivity
    float dt = 0.1f;  // time step

    // 2d temperature grid stored as 1d array, row-major (j varies fastest)
    // indexing: T[i * n + j] for point (i, j)
    std::vector<float> T;

    // grid spacing
    float dx() const { return domainWidth / (m); }
    float dy() const { return domainHeight / (n); }

    // helper to get temperature at (i, j) with dirichlet bc (T=0 at boundaries)
    float getT(const std::vector<float>& grid, int i, int j) const
    {
        if (i < 0 || i >= m || j < 0 || j >= n)
            return 0.0f; // dirichlet bc
        return grid[i * n + j];
    }

    // explicit euler step for diffusion equation
    std::vector<float> explicitStep(const std::vector<float>& Told) const
    {
        std::vector<float> Tnew(m * n, 0.0f);
        float dx2 = dx() * dx();
        float dy2 = dy() * dy();

        for (int i = 0; i < m; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                float Tij = getT(Told, i, j);
                float laplacian_x = (getT(Told, i + 1, j) - 2.0f * Tij + getT(Told, i - 1, j)) / dx2;
                float laplacian_y = (getT(Told, i, j + 1) - 2.0f * Tij + getT(Told, i, j - 1)) / dy2;
                Tnew[i * n + j] = Tij + nu * dt * (laplacian_x + laplacian_y);
            }
        }
        return Tnew;
    }

public:
    virtual void init() override;
    virtual void onDraw(Renderer& renderer) override;
    virtual void onGUI() override;
};

