#include "SceneMidpoint.h"
#include <imgui.h>

void SceneMidpoint::init()
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

void SceneMidpoint::simulateStep()
{
    if (isSimulationRunning)
        runSimulationStep(delta_t);
}

void SceneMidpoint::runSimulationStep(float stepsize)
{
    // Vector to store summed up forces
    std::vector<glm::vec3> elastic_forces(masspoints.size(), glm::vec3(0.0f));

    // Vectors to store midpoint estimates
    std::vector<glm::vec3> velocities_midpoint(masspoints.size(), glm::vec3(0.0f));
    std::vector<glm::vec3> positions_midpoint(masspoints.size(), glm::vec3(0.0f));

    // Forces at n
    for (auto &spring : springs)
    {
        SceneMidpoint::calculateElasticForces(spring, elastic_forces);
    }

    // integrate midpoint values
    for (auto &point : masspoints)
    {
        auto force = elastic_forces[point.id];

        auto a = calculateAcceleration(force, point.mass);

        velocities_midpoint[point.id] = eulerStep(point.velocity, a, stepsize / 2.0);
        positions_midpoint[point.id] = eulerStep(point.position, point.velocity, stepsize / 2.0);
    }

    std::vector<glm::vec3> elastic_forces_midpoint(masspoints.size(), glm::vec3(0.0f));
    // Calculate forces at midpoint
    for (auto &spring : springs)
    {
        SceneMidpoint::calculateElasticForcesAtMidpoint(spring, positions_midpoint[spring.p1.id], positions_midpoint[spring.p2.id], elastic_forces_midpoint);
    }

    // do the midpoint step
    for (auto &point : masspoints)
    {
        auto force = elastic_forces_midpoint[point.id];

        auto a_midpoint = calculateAcceleration(force, point.mass);

        auto new_velocity = eulerStep(point.velocity, a_midpoint, stepsize);
        auto new_position = eulerStep(point.position, velocities_midpoint[point.id], stepsize);

        point.velocity = new_velocity;
        point.position = new_position;
    }
}

void SceneMidpoint::calculateElasticForces(spring_t &spring, std::vector<glm::vec3> &masspointForces)
{
    float l = glm::length(spring.p1.position - spring.p2.position);
    auto forceOnP1 = -spring.stiffness * (l - spring.rest_length) * (spring.p1.position - spring.p2.position) / l;
    auto forceOnP2 = -forceOnP1;

    masspointForces[spring.p1.id] += forceOnP1;
    masspointForces[spring.p2.id] += forceOnP2;
}

void SceneMidpoint::calculateElasticForcesAtMidpoint(spring_t &spring, glm::vec3 p1_midpoint_position, glm::vec3 p2_midpoint_position, std::vector<glm::vec3> &masspointForces)
{
    float l = glm::length(p1_midpoint_position - p2_midpoint_position);
    auto forceOnP1 = -spring.stiffness * (l - spring.rest_length) * (p1_midpoint_position - p2_midpoint_position) / l;
    auto forceOnP2 = -forceOnP1;

    masspointForces[spring.p1.id] += forceOnP1;
    masspointForces[spring.p2.id] += forceOnP2;
}

glm::vec3 SceneMidpoint::calculateAcceleration(glm::vec3 force, float mass)
{
    return force / mass;
}

glm::vec3 SceneMidpoint::eulerStep(glm::vec3 x_n, glm::vec3 xPrime_n, float h)
{
    return x_n + h * xPrime_n;
}

void SceneMidpoint::onDraw(Renderer &renderer)
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

void SceneMidpoint::onGUI()
{
    ImGui::InputFloat("Delta Time", &delta_t);
    auto startStopClicked = ImGui::Button(isSimulationRunning ? "Stop Simulation" : "Start Simulation");
    auto oneStepClicked = ImGui::Button("Run one step");

    if (startStopClicked)
        isSimulationRunning = !isSimulationRunning;
    if (oneStepClicked)
        runSimulationStep(delta_t);
}
