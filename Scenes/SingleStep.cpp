#include "SingleStep.h"
#include <imgui.h>
#include <iomanip>

void SingleStep::init()
{
    // initial temperature field from the exercise
    // T[0] with i = 0..2 (rows), j = 0..5 (columns)
    // i\j   0   1   2   3   4   5
    // 0     6   5   1  -1  -2  -1
    // 1     4   3   0  -1  -3  -1
    // 2     3   2  -1  -2  -4  -2

    T.resize(m * n);
    // row i=0
    T[0 * n + 0] = 6;  T[0 * n + 1] = 5;  T[0 * n + 2] = 1;
    T[0 * n + 3] = -1; T[0 * n + 4] = -2; T[0 * n + 5] = -1;
    // row i=1
    T[1 * n + 0] = 4;  T[1 * n + 1] = 3;  T[1 * n + 2] = 0;
    T[1 * n + 3] = -1; T[1 * n + 4] = -3; T[1 * n + 5] = -1;
    // row i=2
    T[2 * n + 0] = 3;  T[2 * n + 1] = 2;  T[2 * n + 2] = -1;
    T[2 * n + 3] = -2; T[2 * n + 4] = -4; T[2 * n + 5] = -2;

    // perform one explicit euler step
    std::vector<float> T1 = explicitStep(T);

    // print the requested values
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Single Step Explicit Euler Test ===" << std::endl;
    std::cout << "Domain: " << domainWidth << " x " << domainHeight << std::endl;
    std::cout << "Grid: " << m << " x " << n << " interior points" << std::endl;
    std::cout << "dx = " << dx() << ", dy = " << dy() << std::endl;
    std::cout << "nu = " << nu << ", dt = " << dt << std::endl;
    std::cout << std::endl;
    std::cout << "T[1]_{1,3} = " << T1[1 * n + 3] << std::endl;
    std::cout << "T[1]_{0,3} = " << T1[0 * n + 3] << std::endl;
    std::cout << "T[1]_{0,5} = " << T1[0 * n + 5] << std::endl;
    std::cout << std::endl;

    // print full table after t=0.1
    std::cout << "Full T[1] table (after dt=0.1):" << std::endl;
    std::cout << "i\\j\t";
    for (int j = 0; j < n; ++j)
        std::cout << j << "\t\t";
    std::cout << std::endl;

    for (int i = 0; i < m; ++i)
    {
        std::cout << i << "\t";
        for (int j = 0; j < n; ++j)
            std::cout << T1[i * n + j] << "\t";
        std::cout << std::endl;
    }

    // update T to the new state for visualization
    T = T1;
}

void SingleStep::onDraw(Renderer& renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    // optionally visualize the grid as an image
    if (!T.empty())
    {
        renderer.drawImage(T, m, n, Colormap("coolwarm"), {0.5f, 0.5f}, {0.4f, 0.4f});
    }
}

void SingleStep::onGUI()
{
    ImGui::Text("Single Step Test - Explicit Euler");
    ImGui::Text("Check terminal for output values");
    ImGui::Separator();
    ImGui::Text("Grid: %d x %d", m, n);
    ImGui::Text("dx = %.4f, dy = %.4f", dx(), dy());
    ImGui::Text("nu = %.4f, dt = %.4f", nu, dt);
}

