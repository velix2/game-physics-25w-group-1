#include "SceneComplexSimulation.h"
#include <imgui.h>

void SceneComplexSimulation::init()
{
     masspoints.push_back({// id
                          0,
                          // x0
                          {0.0f, 0.0f, 0.0f},
                          // v0
                          {0.0f, 0.0f, 0.0f},
                          // m0
                          10.0f});

    masspoints.push_back({// id
                          1,
                          // x1
                          {0.0f, 2.0f, 0.0f},
                          // v1
                          {0.0f, 0.0f, 0.0f},
                          // m1
                          10.0f});

    masspoints.push_back({2, {2.0f, 2.0f, 0.5f}, {0.0f, 2.0f, 0.0f}, 10.0f});
    masspoints.push_back({3, {4.0f, 1.0f, 1.0f}, {0.0f, 0.1f, 0.0f}, 10.0f});
    masspoints.push_back({4, {2.0f, 0.0f, 0.0f}, {0.0f, 0.4f, 0.0f}, 10.0f});
    masspoints.push_back({5, {0.0f, -2.0f, -0.5f}, {0.0f, 0.0f, 0.0f}, 10.0f});
    masspoints.push_back({6, {-2.0f, -2.0f, 0.0f}, {0.0f, 0.0f, -1.0f}, 10.0f});
    masspoints.push_back({7, {-4.0f, 0.0f, 1.5f}, {0.0f, 0.2f, 0.0f}, 10.0f});
    masspoints.push_back({8, {-2.0f, 2.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 10.0f});
    masspoints.push_back({9, {0.0f, 4.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, 10.0f});

    springs.push_back({40.0f,
                       1.0f,
                       masspoints[0],
                       masspoints[1]});

    springs.push_back({40.0f, 8.0f, masspoints[1], masspoints[2]});
    springs.push_back({40.0f, 1.0f, masspoints[2], masspoints[3]});
    springs.push_back({20.0f, 2.0f, masspoints[3], masspoints[4]});
    springs.push_back({40.0f, 1.0f, masspoints[4], masspoints[5]});
    springs.push_back({40.0f, 1.0f, masspoints[5], masspoints[6]});
    springs.push_back({50.0f, 4.0f, masspoints[6], masspoints[7]});
    springs.push_back({40.0f, 1.5f, masspoints[7], masspoints[8]});
    springs.push_back({10.0f, 1.0f, masspoints[8], masspoints[9]});
    springs.push_back({40.0f, 2.0f, masspoints[9], masspoints[0]}); 
/* 
    // create 3x3x3 masspoints
    const int N = 3;
    const float spacing = 2.0f;
    uint id = 0;
    for (int ix = 0; ix < N; ++ix)
    {
        for (int iy = 0; iy < N; ++iy)
        {
            for (int iz = 0; iz < N; ++iz)
            {
                glm::vec3 pos = glm::vec3((ix - 1) * spacing, (iy - 1) * spacing, (iz - 1) * spacing);
                masspoints.push_back({id + 1, pos, glm::vec3(0.0f), 10.0f});
            }
        }
    }

    // connect axis-adjacent neighbors with springs
    const float stiffness = 50.0f;
    for (int ix = 0; ix < N; ++ix)
    {
        for (int iy = 0; iy < N; ++iy)
        {
            for (int iz = 0; iz < N; ++iz)
            {
                int idx = ix * N * N + iy * N + iz;
                // +x neighbor
                if (ix + 1 < N)
                {
                    int nb = (ix + 1) * N * N + iy * N + iz;
                    float rest_len = glm::length(masspoints[idx].position - masspoints[nb].position);
                    springs.push_back({stiffness, rest_len, masspoints[idx], masspoints[nb]});
                }
                // +y neighbor
                if (iy + 1 < N)
                {
                    int nb = ix * N * N + (iy + 1) * N + iz;
                    float rest_len = glm::length(masspoints[idx].position - masspoints[nb].position);
                    springs.push_back({stiffness, rest_len, masspoints[idx], masspoints[nb]});
                }
                // +z neighbor
                if (iz + 1 < N)
                {
                    int nb = ix * N * N + iy * N + (iz + 1);
                    float rest_len = glm::length(masspoints[idx].position - masspoints[nb].position);
                    springs.push_back({stiffness, rest_len, masspoints[idx], masspoints[nb]});
                }
            }
        }
    }  */

    collision_planes.push_back({{1.f, 0.f, 0.f}, -5.f});
    collision_planes.push_back({{-1.f, 0.f, 0.f}, -5.0f});

    collision_planes.push_back({{0.f, 1.f, 0.f}, -5.0f});
    collision_planes.push_back({{0.f, -1.f, 0.f}, -5.0f});

    collision_planes.push_back({{0.f, 0.f, 1.f}, -5.0f});
    collision_planes.push_back({{0.f, 0.f, -1.f}, -5.0f});
}

void SceneComplexSimulation::simulateStep()
{
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);

    // Input handling:
    // Drag controls

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
    {
        auto drag = ImGui::GetMouseDragDelta(1);
        if (!(drag.x == 0 && drag.y == 0))
        {
            auto dx = drag.x * right;
            auto dy = -drag.y * up;

            for (auto &&p : masspoints)
            {
                p.velocity += (dx + dy) * 0.01f;
            }
        }
    }

    if (isSimulationRunning)
        runSimulationStep(delta_t);
}

void SceneComplexSimulation::runSimulationStep(float stepsize)
{
    // Collision Handling
    for (auto &&plane : collision_planes)
    {
        for (auto &&point : masspoints)
        {
            handleCollision(plane, point);
        }
    }

    if (isUsingMidpoint)
        runSimulationStepWithMidpoint(stepsize);
    else
        runSimulationStepWithEuler(stepsize);
}

void SceneComplexSimulation::runSimulationStepWithEuler(float stepsize)
{
    // Vector to store summed up forces
    std::vector<glm::vec3> elastic_forces(masspoints.size(), glm::vec3(0.0f));

    for (auto &spring : springs)
    {
        calculateElasticForcesWithGravity(spring, elastic_forces);
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

void SceneComplexSimulation::runSimulationStepWithMidpoint(float stepsize)
{
    // Vector to store summed up forces
    std::vector<glm::vec3> elastic_forces(masspoints.size(), glm::vec3(0.0f));

    // Vectors to store midpoint estimates
    std::vector<glm::vec3> velocities_midpoint(masspoints.size(), glm::vec3(0.0f));
    std::vector<glm::vec3> positions_midpoint(masspoints.size(), glm::vec3(0.0f));

    // Forces at n
    for (auto &spring : springs)
    {
        calculateElasticForcesWithGravity(spring, elastic_forces);
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
        calculateElasticForcesAtMidpointWithGravity(spring, positions_midpoint[spring.p1.id], positions_midpoint[spring.p2.id], elastic_forces_midpoint);
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

void SceneComplexSimulation::handleCollision(collision_plane_t &plane, masspoint_t &point)
{
    float distanceBetweenSurfaceAndPoint = glm::dot(plane.surfaceNormal, point.position) - plane.offsetAlongNormal;

    // check if collission occured
    if (distanceBetweenSurfaceAndPoint > 0.0001f)
        return;

    // reset location
    point.position += distanceBetweenSurfaceAndPoint * plane.surfaceNormal;

    // mirror velocity
    //auto reflection = point.velocity - 2.f * glm::dot(point.velocity, plane.surfaceNormal) * plane.surfaceNormal;

    point.velocity = plane.surfaceNormal * glm::length(point.velocity);
    //point.velocity = reflection;
}

void SceneComplexSimulation::calculateElasticForcesWithGravity(spring_t &spring, std::vector<glm::vec3> &masspointForces)
{
    float l = glm::length(spring.p1.position - spring.p2.position);
    auto down = glm::vec3(0.f, 0.f, -1.f);

    auto forceOnP1 = -spring.stiffness * (l - spring.rest_length) * (spring.p1.position - spring.p2.position) / l;
    auto forceOnP2 = -forceOnP1;

    // Gravity
    forceOnP1 += down * (spring.p1.mass * gravity_accel);
    forceOnP2 += down * (spring.p2.mass * gravity_accel);

    masspointForces[spring.p1.id] += forceOnP1;
    masspointForces[spring.p2.id] += forceOnP2;
}

void SceneComplexSimulation::calculateElasticForcesAtMidpointWithGravity(spring_t &spring, glm::vec3 p1_midpoint_position, glm::vec3 p2_midpoint_position, std::vector<glm::vec3> &masspointForces)
{
    float l = glm::length(p1_midpoint_position - p2_midpoint_position);
    auto down = glm::vec3(0.f, 0.f, -1.f);

    auto forceOnP1 = -spring.stiffness * (l - spring.rest_length) * (p1_midpoint_position - p2_midpoint_position) / l;
    auto forceOnP2 = -forceOnP1;

    masspointForces[spring.p1.id] += forceOnP1;
    masspointForces[spring.p2.id] += forceOnP2;

    // Gravity
    forceOnP1 += down * (spring.p1.mass * gravity_accel);
    forceOnP2 += down * (spring.p2.mass * gravity_accel);

    masspointForces[spring.p1.id] += forceOnP1;
    masspointForces[spring.p2.id] += forceOnP2;
}

glm::vec3 SceneComplexSimulation::calculateAcceleration(glm::vec3 force, float mass)
{
    return force / mass;
}

glm::vec3 SceneComplexSimulation::eulerStep(glm::vec3 x_n, glm::vec3 xPrime_n, float h)
{
    return x_n + h * xPrime_n;
}

void SceneComplexSimulation::onDraw(Renderer &renderer)
{
    // Wireframe cube
    renderer.drawWireCube(glm::vec3(0), glm::vec3(10), glm::vec3(1));

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

void SceneComplexSimulation::onGUI()
{
    ImGui::InputFloat("Delta Time", &delta_t);
    ImGui::InputFloat("Gravity Acceleration", &gravity_accel);

    auto startStopClicked = ImGui::Button(isSimulationRunning ? "Stop Simulation" : "Start Simulation");
    auto oneStepClicked = ImGui::Button("Run one step");
    ImGui::Separator();
    auto toggleAlgorithm = ImGui::Button(isUsingMidpoint ? "Switch to Euler Method" : "Switch to Midpoint Method");

    if (startStopClicked)
        isSimulationRunning = !isSimulationRunning;
    if (oneStepClicked)
        runSimulationStep(delta_t);

    if (toggleAlgorithm)
        isUsingMidpoint = !isUsingMidpoint;
}
