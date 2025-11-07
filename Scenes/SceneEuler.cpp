#include "SceneEuler.h"
#include <imgui.h>


void SceneEuler::init()
{
    masspoints.push_back({// id
                          0,
                          // x0
                          {0.0f, 0.0f, 0.0f},
                          // v0
                          {-1.0f, 0.0f, 0.0f},
                          // m0
                          10.0f});

    masspoints.push_back({// id
                          1,
                          // x1
                          {0.0f, 2.0f, 0.0f},
                          // v1
                          {1.0f, 0.0f, 0.0f},
                          // m1
                          10.0f});

    springs.push_back({40.0f,
                       1.0f,
                       masspoints[0],
                       masspoints[1]});
}

void SceneEuler::simulateStep()
{
    if (isSimulationRunning) runSimulationStep(ImGui::GetIO().DeltaTime);
}

void SceneEuler::runSimulationStep(float stepsize)
{
    // Vector to store summed up forces
    std::vector<glm::vec3> elastic_forces(masspoints.size(), glm::vec3(0.0f));

    for (auto &spring : springs)
    {
        SceneEuler::calculateElasticForces(spring, elastic_forces);
    }

    for (auto &point : masspoints)
    {
        auto force = elastic_forces[point.id];

        auto a = calculateAcceleration(force, point.mass);

        auto new_velocity = eulerStep(point.velocity, a, stepsize);
        auto new_position = eulerStep(point.position, point.velocity, stepsize);

        point.velocity = new_velocity;
        point.position = new_position;
    }
}

void SceneEuler::calculateElasticForces(spring_t &spring, std::vector<glm::vec3> &masspointForces)
{
    float l = glm::length(spring.p1.position - spring.p2.position);
    auto forceOnP1 = -spring.stiffness * (l - spring.rest_length) * (spring.p1.position - spring.p2.position) / l;
    auto forceOnP2 = -forceOnP1;

    masspointForces[spring.p1.id] += forceOnP1;
    masspointForces[spring.p2.id] += forceOnP2;
}

glm::vec3 SceneEuler::calculateAcceleration(glm::vec3 force, float mass)
{
    return force / mass;
}

glm::vec3 SceneEuler::eulerStep(glm::vec3 x_n, glm::vec3 xPrime_n, float h)
{
    return x_n + h * xPrime_n;
}

void SceneEuler::onDraw(Renderer &renderer)
{
    // Draw points
    for (auto &&point : masspoints)
    {
        renderer.drawSphere(point.position, POINT_RADIUS, POINT_COLOR);
    }

    // Draw springs
    for (auto &&spring : springs)
    {
        renderer.drawLine(spring.p1.position, spring.p2.position, SPRING_COLOR);
    }
}

void SceneEuler::onGUI()
{
    auto startStopClicked = ImGui::Button(isSimulationRunning ? "Stop Simulation" : "Start Simulation");

    if (startStopClicked) isSimulationRunning = !isSimulationRunning;
}
