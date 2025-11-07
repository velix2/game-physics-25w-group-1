#include "SceneSingleStep.h"
#include <imgui.h>

// Helpers decl
std::string vec3ToString(glm::vec3 v);
// ============

inline glm::vec3 SceneSingleStep::eulerStep(glm::vec3 x_n, glm::vec3 xPrime_n, float h)
{
    return x_n + h * xPrime_n;
}

glm::vec3 SceneSingleStep::calculateForce_ab(float k, float L, glm::vec3 x_a, glm::vec3 x_b)
{
    float l = glm::length(x_a - x_b);
    return -k * (l - L) * (x_a - x_b) / l;
}

inline glm::vec3 SceneSingleStep::calculateAcceleration(glm::vec3 F, float m)
{
    return F / m;
}

void SceneSingleStep::mssWithEuler()
{
    // Steps n and n+1 variables
    glm::vec3 x0_n = mss_data.x0, x1_n = mss_data.x1, v0_n = mss_data.v0, v1_n = mss_data.v1,
              x0_n1, x1_n1, v0_n1, v1_n1;

    std::cout << "Initial Values:\n\n"
              << "Masspoint 0\n"
              << "position: " << vec3ToString(x0_n) << '\n'
              << "velocity: " << vec3ToString(v0_n) << '\n'
              << "Masspoint 1\n"
              << "position: " << vec3ToString(x1_n) << '\n'
              << "velocity: " << vec3ToString(v1_n) << "\n\n";

    for (int i = 0; i < mss_data.steps; i++)
    {
        // Forces
        glm::vec3 F01_n = calculateForce_ab(mss_data.k, mss_data.L, x0_n, x1_n);
        glm::vec3 F10_n = -F01_n;

        // Accel
        glm::vec3 a0_n = calculateAcceleration(F01_n, mss_data.m0);
        glm::vec3 a1_n = calculateAcceleration(F10_n, mss_data.m1);

        // Integration (velocity)
        v0_n1 = eulerStep(v0_n, a0_n, mss_data.delta_t);
        v1_n1 = eulerStep(v1_n, a1_n, mss_data.delta_t);

        // Integration (position)
        x0_n1 = eulerStep(x0_n, v0_n, mss_data.delta_t);
        x1_n1 = eulerStep(x1_n, v1_n, mss_data.delta_t);

        // Do the logging
        std::cout << "After 0.1s using Euler Method:\n\n"
                  << "Masspoint 0\n"
                  << "position: " << vec3ToString(x0_n1) << '\n'
                  << "velocity: " << vec3ToString(v0_n1) << '\n'
                  << "Masspoint 1\n"
                  << "position: " << vec3ToString(x1_n1) << '\n'
                  << "velocity: " << vec3ToString(v1_n1) << "\n\n";

        // move values forward for next iter
        v0_n = v0_n1;
        v1_n = v1_n1;

        x0_n = x0_n1;
        x1_n = x1_n1;
    }
}

void SceneSingleStep::mssWithMidpoint()
{
    // Steps n and n+1 variables
    glm::vec3 x0_n = mss_data.x0, x1_n = mss_data.x1, v0_n = mss_data.v0, v1_n = mss_data.v1,
              x0_n1, x1_n1, v0_n1, v1_n1;

    std::cout << "Initial Values:\n\n"
              << "Masspoint 0\n"
              << "position: " << vec3ToString(x0_n) << '\n'
              << "velocity: " << vec3ToString(v0_n) << '\n'
              << "Masspoint 1\n"
              << "position: " << vec3ToString(x1_n) << '\n'
              << "velocity: " << vec3ToString(v1_n) << "\n\n";

    for (int i = 0; i < mss_data.steps; i++)
    {
        // Approx x at half step
        glm::vec3 x0_half = eulerStep(x0_n, v0_n, mss_data.delta_t / 2.0); 
        glm::vec3 x1_half = eulerStep(x1_n, v1_n, mss_data.delta_t / 2.0); 

        // Forces at n
        glm::vec3 F01_n = calculateForce_ab(mss_data.k, mss_data.L, x0_n, x1_n);
        glm::vec3 F10_n = -F01_n;

        // Accel at n
        glm::vec3 a0_n = calculateAcceleration(F01_n, mss_data.m0);
        glm::vec3 a1_n = calculateAcceleration(F10_n, mss_data.m1);

        // Approx v at half step
        glm::vec3 v0_half = eulerStep(v0_n, a0_n, mss_data.delta_t / 2.0); 
        glm::vec3 v1_half = eulerStep(v1_n, a1_n, mss_data.delta_t / 2.0); 

        // Integration (position)
        x0_n1 = eulerStep(x0_n, v0_half, mss_data.delta_t);
        x1_n1 = eulerStep(x1_n, v1_half, mss_data.delta_t);

        // Calc Force at half step
        glm::vec3 F01_half = calculateForce_ab(mss_data.k, mss_data.L, x0_half, x1_half);
        glm::vec3 F10_half = -F01_half;

        // Approx a at half step
        glm::vec3 a0_half = calculateAcceleration(F01_half, mss_data.m0);
        glm::vec3 a1_half = calculateAcceleration(F10_half, mss_data.m0);

        // Integration (velocity)
        v0_n1 = eulerStep(v0_n, a0_half, mss_data.delta_t);
        v1_n1 = eulerStep(v1_n, a1_half, mss_data.delta_t);

        // Do the logging
        std::cout << "After 0.1s using Midpoint Method:\n\n"
                  << "Masspoint 0\n"
                  << "position: " << vec3ToString(x0_n1) << '\n'
                  << "velocity: " << vec3ToString(v0_n1) << '\n'
                  << "Masspoint 1\n"
                  << "position: " << vec3ToString(x1_n1) << '\n'
                  << "velocity: " << vec3ToString(v1_n1) << "\n\n";

        // move values forward for next iter
        v0_n = v0_n1;
        v1_n = v1_n1;

        x0_n = x0_n1;
        x1_n = x1_n1;
    }
}

void SceneSingleStep::onGUI()
{

    ImGui::InputFloat3("Position 0", &mss_data.x0[0]);
    
    ImGui::InputFloat3("Position 1", &mss_data.x1[0]);
    
    ImGui::InputFloat3("Velocity 0", &mss_data.v0[0]);
    
    ImGui::InputFloat3("Velocity 1", &mss_data.v1[0]);

    ImGui::Separator();

    ImGui::DragFloat("Stiffness", &mss_data.k, 0.1f, 0.0f, 100.0f, "%.2f");
    
    ImGui::DragFloat("Rest Length", &mss_data.L, 0.1f, 0.0f, 100.0f, "%.2f");
    
    ImGui::DragFloat("Mass 0", &mss_data.m0, 0.1f, 0.01f, 100.0f, "%.2f");
    
    ImGui::DragFloat("Mass 1", &mss_data.m1, 0.1f, 0.01f, 100.0f, "%.2f");

    ImGui::Separator();
    
    ImGui::Text("Simulation Settings");

    ImGui::InputInt("Steps", &mss_data.steps, 1, 10, ImGuiInputTextFlags_CharsDecimal);
    
    ImGui::InputFloat("Time Delta", &mss_data.delta_t, 0.0001f, 0.001f, "%.4f");

    auto runEuler = ImGui::Button("Run Euler");
    auto runMidpoint = ImGui::Button("Run Midpoint");

    if (runEuler) mssWithEuler();
    if (runMidpoint) mssWithMidpoint();
}

// Helpers

std::string vec3ToString(glm::vec3 v)
{
    std::stringstream ss;
    ss << "vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
    return ss.str();
}
