#include "ExplicitSimulation.h"
#include <imgui.h>
#include <cmath>

void ExplicitSimulation::init()
{
    initRandom();
    stepCount = 0;
    running = false;
}

void ExplicitSimulation::initRandom()
{
    T.resize(m * n);
    for (int i = 0; i < m * n; ++i)
        T[i] = dis(gen);
}

void ExplicitSimulation::initGaussian()
{
    T.resize(m * n);
    float cx = 0.5f, cy = 0.5f; // center of domain
    float sigma = 0.1f;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            // position in physical space
            float x = (i + 1) * dx();
            float y = (j + 1) * dy();
            float r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            T[i * n + j] = std::exp(-r2 / (2.0f * sigma * sigma));
        }
    }
}

void ExplicitSimulation::initSineWave()
{
    T.resize(m * n);
    const float pi = 3.14159265f;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            float x = (i + 1) * dx();
            float y = (j + 1) * dy();
            T[i * n + j] = std::sin(pi * x) * std::sin(pi * y);
        }
    }
}

void ExplicitSimulation::explicitStep()
{
    std::vector<float> Tnew(m * n);
    float dx2 = dx() * dx();
    float dy2 = dy() * dy();

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            float Tij = getT(i, j);
            float lap_x = (getT(i + 1, j) - 2.0f * Tij + getT(i - 1, j)) / dx2;
            float lap_y = (getT(i, j + 1) - 2.0f * Tij + getT(i, j - 1)) / dy2;
            Tnew[i * n + j] = Tij + nu * dt * (lap_x + lap_y);
        }
    }
    T = Tnew;
    stepCount++;
}

void ExplicitSimulation::simulateStep()
{
    if (running)
        explicitStep();
}

void ExplicitSimulation::onDraw(Renderer& renderer)
{
    // visualize temperature field as heatmap
    if (!T.empty())
    {
        renderer.drawImage(T, m, n, Colormap("coolwarm"), {0.0f, 0.0f}, {1.0f, 1.0f});
    }
}

void ExplicitSimulation::onGUI()
{
    ImGui::Text("Explicit FTCS Simulation");
    ImGui::Separator();

    // simulation controls
    if (ImGui::Button(running ? "Pause" : "Start"))
        running = !running;

    ImGui::SameLine();
    if (ImGui::Button("Step"))
        explicitStep();

    ImGui::SameLine();
    if (ImGui::Button("Reset"))
        init();

    ImGui::Text("Step: %d", stepCount);
    ImGui::Separator();

    // parameters
    ImGui::SliderFloat("dt", &dt, 0.0001f, 0.1f, "%.4f");
    ImGui::SliderFloat("nu", &nu, 0.001f, 1.0f, "%.3f");

    // stability indicator - cfl condition for explicit heat eq
    float cfl = nu * dt * (1.0f / (dx() * dx()) + 1.0f / (dy() * dy()));
    ImGui::Text("CFL: %.4f (should be < 0.5)", cfl);
    if (cfl > 0.5f)
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "Warning: Unstable!");

    ImGui::Separator();

    // initialization options
    ImGui::Text("Initialize:");
    if (ImGui::Button("Random Noise"))
    {
        initRandom();
        stepCount = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Gaussian"))
    {
        initGaussian();
        stepCount = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Sine Wave"))
    {
        initSineWave();
        stepCount = 0;
    }

    ImGui::Separator();
    ImGui::Text("Grid: %d x %d", m, n);
    ImGui::Text("dx = %.4f, dy = %.4f", dx(), dy());
}

