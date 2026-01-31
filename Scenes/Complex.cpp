#include "Complex.h"
#include <imgui.h>

// Creates a 3d grid of blocks
// returns the start index, can be used for the spring stuff
int SummonXxYxZBlocks(std::vector<Body> &bodies, glm::vec3 anchor, glm::vec3 blocksize, glm::vec3 spacing, int X, int Y, int Z, float mass, glm::vec4 color)
{
    auto startIdx = bodies.size();

    for (size_t i = 0; i < X; i++)
    {
        for (size_t j = 0; j < Y; j++)
        {
            for (size_t k = 0; k < Z; k++)
            {
                auto b = Body(anchor + glm::vec3(i * spacing.x, j * spacing.y, k * spacing.z), glm::vec3(0, 0, 0), glm::quat(glm::vec3(0, 0, 0)), glm::vec3(0), mass, blocksize, false, color);
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

                if (i < X - 1) springs.push_back(Spring(bodies[idxCenter], bodies[idxX], restLen, stiffness));
                if (j < Y - 1) springs.push_back(Spring(bodies[idxCenter], bodies[idxY], restLen, stiffness));
                if (k < Z - 1) springs.push_back(Spring(bodies[idxCenter], bodies[idxZ], restLen, stiffness));
            }
        }
    }
}

void Complex::init()
{
    auto tower1Idx = SummonXxYxZBlocks(bodies, glm::vec3(5, -2, -3.5), glm::vec3(1.25), glm::vec3(1.3), 4,4,5, 10, glm::vec4(1,0,0,1));

    // Floor
    auto floor = Body(glm::vec3(0, 0, -4.75), glm::vec3(0), glm::quat(glm::vec3(0)), glm::vec3(0), 1000, glm::vec3(50, 50, 1), true);
    bodies.push_back(floor);

    // Wrecking ball anchor
    auto anchorIdx = bodies.size();
    auto anchor = Body(glm::vec3(0, 0, 10), glm::vec3(0), glm::quat(glm::vec3(0)), glm::vec3(0), 1, glm::vec3(.5f), true);
    bodies.push_back(anchor);

    // Wrecking ball
    auto ballIdx = bodies.size();
    auto ball = Body(glm::vec3(-8, 0, 3), glm::vec3(2,0,-2), glm::quat(glm::vec3(0)), glm::vec3(2), 10000, glm::vec3(2), false);
    bodies.push_back(ball);

    // ALWAYS init springs after bodies cuz of them pointers
    // Not great i know but its fine for now i guess

    // tower one springs
    ConnectXxYxZBlocks(springs, bodies, tower1Idx, 4,4,5, 1.2, 1000);

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
        for (size_t i = 0; i < NUM_BODIES; i++)
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
    ImGui::Checkbox("Native Cubes", &useNativeCubeRendering);
    ImGui::Separator();
    ImGui::SliderFloat("Dt", &dt, 0, 0.1f);
    ImGui::SliderFloat("Friction", &friction, 0, 1);
    ImGui::Checkbox("Paused", &paused);
    auto btnOneStep = ImGui::Button("Step");
    if (btnOneStep)
    {
        oneStep = true;
    }
    // ImGui::Checkbox("Apply Force", &applyForce);
    ImGui::Text("Right click a body to apply a force.");
    ImGui::SliderFloat("Click strength", &forceStrength, 0, 100);
}