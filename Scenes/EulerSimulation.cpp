#include "EulerSimulation.h"
#include "imgui.h">

void EulerSimulation::simulateStep()
{
    // Forces
    float springCurrentLength = glm::length(EulerSimulation::x0 - EulerSimulation::x1);
    glm::vec3 springForce0_1 = calculateSpringForce(EulerSimulation::stiffness, springCurrentLength,EulerSimulation::springRestLength,
                                                    EulerSimulation::x0, EulerSimulation::x1);
    glm::vec3 springForce1_0 = -springForce0_1;

    // acceleration
    glm::vec3 a0 = springForce0_1 / EulerSimulation::mass;
    glm::vec3 a1 = springForce1_0 / EulerSimulation::mass;

    //For x0
    EulerSimulation::x0 = EulerSimulation::x0 + EulerSimulation::deltaT * EulerSimulation::v0;
    EulerSimulation::v0 = EulerSimulation::v0 + EulerSimulation::deltaT * a0;

    //For x1
    EulerSimulation::x1 = EulerSimulation::x1 + EulerSimulation::deltaT * EulerSimulation::v1;
    EulerSimulation::v1 = EulerSimulation::v1 + EulerSimulation::deltaT * a1;
}

void EulerSimulation::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    glm::vec4 red (1.0f, 0.0f, 0.0f, 1.0f);
    glm::vec4 blue (0.0f, 0.0f, 1.0f, 1.0f);

    // masspoints
    renderer.drawSphere(EulerSimulation::x0, 0.1f, red);
    renderer.drawSphere(EulerSimulation::x1, 0.1f, red);

    renderer.drawLine(EulerSimulation::x0, EulerSimulation::x1, blue);
}

void EulerSimulation::onGUI()
{
    ImGui::SliderFloat("Time Step Δt", &deltaT, 0.001f, 0.02f);
}

void EulerSimulation::init()
{
    EulerSimulation::x0 = glm::vec3(0.0f, 0.0f, 0.0f);
    EulerSimulation::v0 = glm::vec3(-1.0f, 0.0f, 0.0f);

    EulerSimulation::x1 = glm::vec3(0.0f, 2.0f, 0.0f);
    EulerSimulation::v1 = glm::vec3(1.0f, 0.0f, 0.0f);
}

// Hilfsmethoden
glm::vec3 EulerSimulation::calculateSpringForce (float stiffness, float springCurrentLength, float springRestLength,  
                                glm::vec3 &x0, glm::vec3 &x1)
{
    glm::vec3 springForce = -stiffness * (springCurrentLength - springRestLength) * ((x0 - x1) / springCurrentLength);
    return springForce;
}