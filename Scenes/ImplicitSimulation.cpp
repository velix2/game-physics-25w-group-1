#include "ImplicitSimulation.h"
#include <imgui.h>
#include <cmath>

void ImplicitSimulation::init()
{
    initRandom();
    stepCount = 0;
    running = false;
    lastIterations = 0;
    lastResidual = 0.0;
}

void ImplicitSimulation::initRandom()
{
    T.resize(m * n);
    for (int i = 0; i < m * n; ++i)
        T[i] = dis(gen);
}

void ImplicitSimulation::initGaussian()
{
    T.resize(m * n);
    float cx = 0.5f, cy = 0.5f;
    float sigma = 0.1f;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            float x = (i + 1) * dx();
            float y = (j + 1) * dy();
            float r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            T[i * n + j] = std::exp(-r2 / (2.0f * sigma * sigma));
        }
    }
}

void ImplicitSimulation::initSineWave()
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

void ImplicitSimulation::implicitStep()
{
    // btcs scheme: (I - nu*dt*L) T[t+1] = T[t]
    // where L is the laplacian operator
    //
    // for each point (i,j):
    // (1 + 2*alpha_x + 2*alpha_y) * T[t+1]_{i,j}
    //   - alpha_x * T[t+1]_{i+1,j} - alpha_x * T[t+1]_{i-1,j}
    //   - alpha_y * T[t+1]_{i,j+1} - alpha_y * T[t+1]_{i,j-1}
    // = T[t]_{i,j}

    int N = m * n; // total unknowns
    double dx2 = dx() * dx();
    double dy2 = dy() * dy();
    double alpha_x = nu * dt / dx2;
    double alpha_y = nu * dt / dy2;
    double diag = 1.0 + 2.0 * alpha_x + 2.0 * alpha_y;

    // assemble sparse matrix A and rhs vector b
    SparseMatrix<double> A(N);
    std::vector<double> b(N);
    std::vector<double> x(N, 0.0);

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            int k = idx(i, j);

            // rhs is just the current temperature
            b[k] = T[k];

            // diagonal entry
            A.set_element(k, k, diag);

            // neighbor in i+1 direction
            if (i + 1 < m)
                A.set_element(k, idx(i + 1, j), -alpha_x);
            // else: dirichlet bc, T=0 at boundary, no contribution to rhs

            // neighbor in i-1 direction
            if (i - 1 >= 0)
                A.set_element(k, idx(i - 1, j), -alpha_x);

            // neighbor in j+1 direction
            if (j + 1 < n)
                A.set_element(k, idx(i, j + 1), -alpha_y);

            // neighbor in j-1 direction
            if (j - 1 >= 0)
                A.set_element(k, idx(i, j - 1), -alpha_y);
        }
    }

    // solve Ax = b using pcg
    double residual;
    int iterations;
    solver.solve(A, b, x, residual, iterations);

    lastIterations = iterations;
    lastResidual = residual;

    // extract solution back to temperature grid
    for (int k = 0; k < N; ++k)
        T[k] = static_cast<float>(x[k]);

    stepCount++;
}

void ImplicitSimulation::simulateStep()
{
    if (running)
        implicitStep();
}

void ImplicitSimulation::onDraw(Renderer& renderer)
{
    if (!T.empty())
    {
        renderer.drawImage(T, m, n, Colormap("coolwarm"), {0.0f, 0.0f}, {1.0f, 1.0f});
    }
}

void ImplicitSimulation::onGUI()
{
    ImGui::Text("Implicit BTCS Simulation");
    ImGui::Separator();

    // simulation controls
    if (ImGui::Button(running ? "Pause" : "Start"))
        running = !running;

    ImGui::SameLine();
    if (ImGui::Button("Step"))
        implicitStep();

    ImGui::SameLine();
    if (ImGui::Button("Reset"))
        init();

    ImGui::Text("Step: %d", stepCount);
    ImGui::Separator();

    // parameters
    ImGui::SliderFloat("dt", &dt, 0.0001f, 0.5f, "%.4f");
    ImGui::SliderFloat("nu", &nu, 0.001f, 1.0f, "%.3f");

    ImGui::Text("Implicit scheme - unconditionally stable!");
    ImGui::Separator();

    // solver stats
    ImGui::Text("PCG iterations: %d", lastIterations);
    ImGui::Text("Residual: %.2e", lastResidual);

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

