#include "MidpointSimulation.h"
#include <imgui.h>

void MidpointSimulation::simulateStep()
{
    // Force in current point
    float springCurrentLength = glm::length(MidpointSimulation::x0 - MidpointSimulation::x1);
    glm::vec3 springForce0_1 = MidpointSimulation::calculateSpringForce(stiffness, springCurrentLength, springRestLength, MidpointSimulation::x0, MidpointSimulation::x1);
    glm::vec3 springForce1_0 = - springForce0_1;

    glm::vec3 a0 = springForce0_1 / MidpointSimulation::mass;
    glm::vec3 a1 = springForce1_0 /MidpointSimulation::mass;

    // predictions
    glm::vec3 x0_mid = MidpointSimulation::x0 + (MidpointSimulation::deltaT / 2.0f) * MidpointSimulation::v0;
    glm::vec3 x1_mid = MidpointSimulation::x1 + (MidpointSimulation::deltaT / 2.0f) * MidpointSimulation::v1;

    float springCurrentLength_mid = glm::length(x0_mid - x1_mid);
    glm::vec3 springForce0_1_mid = MidpointSimulation::calculateSpringForce(stiffness, springCurrentLength_mid, springRestLength, x0_mid, x1_mid);
    glm::vec3 springForce1_0_mid = - springForce0_1_mid;

    glm::vec3 a0_mid = springForce0_1_mid / MidpointSimulation::mass;
    glm::vec3 a1_mid = springForce1_0_mid / MidpointSimulation::mass;

    glm::vec3 v0_mid = MidpointSimulation::v0 + (MidpointSimulation::deltaT / 2.0f) * a0;
    glm::vec3 v1_mid = MidpointSimulation::v1 + (MidpointSimulation::deltaT / 2.0f) * a1;

    MidpointSimulation::x0 = MidpointSimulation::x0 + MidpointSimulation::deltaT * v0_mid;
    MidpointSimulation::x1 = MidpointSimulation::x1 + MidpointSimulation::deltaT * v1_mid;

    MidpointSimulation::v0 = MidpointSimulation::v0 + MidpointSimulation::deltaT * a0_mid;
    MidpointSimulation::v1 = MidpointSimulation::v1 + MidpointSimulation::deltaT * a1_mid;
}

void MidpointSimulation::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    glm::vec4 red (1.0f, 0.0f, 0.0f, 1.0f);
    glm::vec4 blue (0.0f, 0.0f, 1.0f, 1.0f);

    // masspoints
    renderer.drawSphere(MidpointSimulation::x0, 0.1f, red);
    renderer.drawSphere(MidpointSimulation::x1, 0.1f, red);

    renderer.drawLine(MidpointSimulation::x0, MidpointSimulation::x1, blue);
}

void MidpointSimulation::onGUI()
{
    ImGui::SliderFloat("Time Step Δt", &deltaT, 0.001f, 0.02f);
}

void MidpointSimulation::init()
{
    MidpointSimulation::x0 = glm::vec3(0.0f, 0.0f, 0.0f);
    MidpointSimulation::v0 = glm::vec3(-1.0f, 0.0f, 0.0f);

    MidpointSimulation::x1 = glm::vec3(0.0f, 2.0f, 0.0f);
    MidpointSimulation::v1 = glm::vec3(1.0f, 0.0f, 0.0f);
}

// Hilfsmethoden
glm::vec3 MidpointSimulation::calculateSpringForce( float stiffness, float springCurrentLength, float springRestLength,  
                                        glm::vec3 &x0, glm::vec3 &x1)
{
    glm::vec3 springForce = -stiffness * (springCurrentLength - springRestLength) * ((x0 - x1) / springCurrentLength);
    return springForce;
}