#include "Complex.h"
#include <imgui.h>

// Creates a 3d grid of blocks
// returns the start index, can be used for the spring stuff
int SummonXxYxZBlocks(std::vector<Body> &bodies, glm::vec3 anchor, glm::vec3 blocksize, glm::vec3 spacing, int X, int Y, int Z, float mass)
{
    auto startIdx = bodies.size();

    for (size_t i = 0; i < X; i++)
    {
        for (size_t j = 0; j < Y; j++)
        {
            for (size_t k = 0; k < Z; k++)
            {
                auto b = Body(anchor + glm::vec3(i * spacing.x, j * spacing.y, k * spacing.z), glm::vec3(0, 0, 0), glm::quat(glm::vec3(0, 0, 0)), glm::vec3(0), mass, blocksize, false, getRandomColor());
                bodies.push_back(b);
            }
        }
    }

    return startIdx;
}

void ConnectXxYxZBlocks(std::vector<Spring> &springs, std::vector<Body> &bodies, size_t startIndex, int X, int Y, int Z, float restLen, float stiffness)
{

    for (size_t i = 0; i < X; i++)
    {
        for (size_t j = 0; j < Y; j++)
        {
            for (size_t k = 0; k < Z; k++)
            {
                auto idxCenter = startIndex + k + j * Z + i * Z * Y;
                auto idxX = idxCenter + Y * Z;
                auto idxY = idxCenter + Z;
                auto idxZ = idxCenter + 1;

                if (i < X - 1)
                    springs.push_back(Spring(bodies[idxCenter], bodies[idxX], restLen, stiffness));
                if (j < Y - 1)
                    springs.push_back(Spring(bodies[idxCenter], bodies[idxY], restLen, stiffness));
                if (k < Z - 1)
                    springs.push_back(Spring(bodies[idxCenter], bodies[idxZ], restLen, stiffness));
            }
        }
    }
}

void SummonTower(std::vector<Body> &bodies, glm::vec3 blocksize, glm::vec3 spacing, int blockCountsXYZ[3], float mass, bool createSprings, std::vector<Spring> &springs, float restLen, float stiffness)
{
    auto anchorOffsetY = (spacing.y * (blockCountsXYZ[1] - 1)) / 2; // Center along y axis
    auto anchorOffsetZ = (spacing.z * (blockCountsXYZ[2] - 1)) / 2; // Center along y axis

    auto anchor = glm::vec3(3, -anchorOffsetY, -2 + anchorOffsetZ);

    auto towerIdx = SummonXxYxZBlocks(bodies, anchor, blocksize, spacing, blockCountsXYZ[0], blockCountsXYZ[1], blockCountsXYZ[2], mass);

    // tower one springs
    if (createSprings)
        ConnectXxYxZBlocks(springs, bodies, towerIdx, blockCountsXYZ[0], blockCountsXYZ[1], blockCountsXYZ[2], 1.2, 1000);
}

void Complex::init()
{
    bodies.reserve(256);

    // Floor
    auto floor = Body(glm::vec3(0, 0, -4.75), glm::vec3(0), glm::quat(glm::vec3(0)), glm::vec3(0), 1000, glm::vec3(40, 30, 1), true, glm::vec4(0.1, 0.1, 0.1, 1));
    bodies.push_back(floor);

    // Wrecking ball anchor
    auto anchorIdx = bodies.size();
    auto anchor = Body(glm::vec3(0, 0, 10), glm::vec3(0), glm::quat(glm::vec3(0)), glm::vec3(0), 1, glm::vec3(.5f), true, glm::vec4(0, 0, 0, 1));
    bodies.push_back(anchor);

    // Wrecking ball
    auto ballIdx = bodies.size();
    auto ball = Body(glm::vec3(-8, 0, 3), glm::vec3(2, 0, -2), glm::normalize(glm::quat(glm::vec3(PI_4))), glm::vec3(2), 10000, glm::vec3(2), false, glm::vec4(0.6, 0.6, 0.6, 1));
    bodies.push_back(ball);

    // ALWAYS init springs after bodies cuz of them pointers
    // Not great i know but its fine for now i guess

    // Wrecking ball spring
    auto ballSpring = Spring(bodies[anchorIdx], bodies[ballIdx], 11, 500000);
    springs.push_back(ballSpring);
}

void Complex::simulateStep()
{
    if (!paused || oneStep)
    {
        // Gravity first
        for (size_t i = 0; i < bodies.size(); i++)
        {
            bodies[i].applyDirectForce(glm::vec3(0, 0, gravity * bodies[i].mass));
        }

        // Do collision checks
        for (size_t i = 0; i < bodies.size() - 1; i++)
        {
            for (size_t j = i + 1; j < bodies.size(); j++)
            {
                bodies[i].doCollide(bodies[j], c, friction);
            }
        }

        // Now springs
        for (size_t i = 0; i < springs.size(); i++)
        {
            springs[i].computeElasticForces(dt, true);
        }

        // Integrate bodies positions
        for (size_t i = 0; i < bodies.size(); i++)
        {
            bodies[i].integrate(dt);
        }

        oneStep = false;
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        auto pos = ImGui::GetMousePos();

        glm::vec3 hitPoint;

        glm::vec3 rel = screenToWorldRay(projMatrix, cameraMatrix, pos.x, pos.y, windowSize.x, windowSize.y);
        for (size_t i = 0; i < bodies.size(); i++)
        {
            if (bodies[i].intersectRay(cameraPosition, rel, hitPoint))
            {
                lastcast1 = hitPoint;
                lastcast2 = hitPoint + rel;
                bodies[i].applyForceAt(hitPoint, forceStrength * rel * bodies[i].mass);
                break;
            }
        }
    }
}

void Complex::onDraw(Renderer &renderer)
{
    // Draw interact
    renderer.drawLine(lastcast1, lastcast2, glm::vec4(1, 1, 0, 1));

    // Draw bodies
    for (size_t i = 0; i < bodies.size(); i++)
    {
        bodies[i].draw(renderer, useNativeCubeRendering);
    }

    // Draw springs
    auto cmap = Colormap("jet");

    for (size_t i = 0; i < springs.size(); i++)
    {
        float dist = glm::length(springs[i].point1->cm - springs[i].point2->cm);
        float colval = (dist + 3 * springs[i].restLength) / (6 * springs[i].restLength);

        renderer.drawLine(springs[i].point1->cm, springs[i].point2->cm,
                          glm::vec4(cmap(colval), 1.0));
    }

    projMatrix = renderer.camera.projectionMatrix();
    cameraMatrix = renderer.camera.viewMatrix;
    cameraPosition = renderer.camera.position;
    // cameraNear = renderer.camera.near;
    windowSize = glm::vec2(renderer.camera.width, renderer.camera.height);
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, -1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);
}

void Complex::onGUI()
{
    ImGui::SeparatorText("Rendering");
    ImGui::Checkbox("Render As Native Cubes", &useNativeCubeRendering);

    if (!hasTowerBeenSpawned)
    {
        ImGui::SeparatorText("Spawn Cube Tower");

        ImGui::DragFloat3("Block Size", &blocksize[0], 0.1f, 0.1f, 10.0f);
        ImGui::DragFloat3("Spacing", &spacing[0], 0.1f, 0.1f, 10.0f);
        ImGui::InputInt3("Block Counts (X, Y, Z)", blockCountsXYZ);
        ImGui::DragFloat("Block Mass", &mass, 0.1f, 0.1f, 100.0f);
        ImGui::Checkbox("Create Springs", &createSprings);
        if (createSprings)
        {
            ImGui::DragFloat("Spring Rest Length", &restLength, 0.1f, 0.1f, 10.0f);
            ImGui::DragFloat("Spring Stiffness", &springStiffness, 0.1f, 0.1f, 1000.0f);
        }
        if (ImGui::Button("Spawn Tower"))
        {
            SummonTower(bodies, blocksize, spacing, blockCountsXYZ, mass, createSprings, springs, restLength, springStiffness);
            hasTowerBeenSpawned = true;
        }
    }
    else
    {
        ImGui::SeparatorText("Reset");
        if (ImGui::Button("Reset Simulation (Retains Settings)"))
        {
            bodies.clear();
            springs.clear();

            paused = true;
            hasTowerBeenSpawned = false;

            init();
        }
    }

    ImGui::SeparatorText("Interaction");
    ImGui::Text("Right click a body to apply a force.");
    ImGui::SliderFloat("Click strength", &forceStrength, 0, 250);

    ImGui::SeparatorText("Simulation Controls");
    ImGui::SliderFloat("Dt", &dt, 0, 0.1f);
    ImGui::SliderFloat("Friction", &friction, 0, 1);
    ImGui::Separator();
    ImGui::Checkbox("Paused", &paused);
    auto btnOneStep = ImGui::Button("Simulate Step");
    if (btnOneStep)
    {
        oneStep = true;
    }

}