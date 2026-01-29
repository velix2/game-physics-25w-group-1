#include "Complex.h"
#include <imgui.h>

void SummonXxYxZBlocks(std::vector<Body> &bodies, glm::vec3 anchor, glm::vec3 blocksize, glm::vec3 spacing, int X, int Y, int Z, float mass)
{
    for (size_t i = 0; i < X; i++)
    {
        for (size_t j = 0; j < Y; j++)
        {
            for (size_t k = 0; k < Z; k++)
            {
                auto b = Body(anchor + glm::vec3(i * spacing.x, j * spacing.y, k * spacing.z), glm::vec3(0, 0, 0), glm::quat(glm::vec3(0, 0, 0)), glm::vec3(0), mass, blocksize, false);
                bodies.push_back(b);
            }
        }
    }
}

int ConnectXxYxZBlocks(std::vector<Spring> &springs, std::vector<Body> &bodies, size_t startIndex, int X, int Y, int Z, float restLen, float stiffness)
{
    int blockCount = X * Y * Z;

    for (size_t i = 0; i < X - 1; i++)
    {
        for (size_t j = 0; j < Y - 1; j++)
        {
            for (size_t k = 0; k < Z - 1; k++)
            {
                auto idxCenter = startIndex + k + j * Z + i * Z * Y;
                auto idxX = idxCenter + Y * Z;
                auto idxY = idxCenter + Z;
                auto idxZ = idxCenter + 1;

                springs.push_back(Spring(bodies[idxCenter], bodies[idxX], restLen, stiffness));
                springs.push_back(Spring(bodies[idxCenter], bodies[idxY], restLen, stiffness));
                springs.push_back(Spring(bodies[idxCenter], bodies[idxZ], restLen, stiffness));
            }
        }
    }

    return blockCount;
}

void Complex::init()
{
    // for (size_t i = 0; i < NUM_SPRINGS + 1; i++)
    // {
    //     bodies[i] = Body(glm::vec3(0, 0, i * .75), glm::vec3(0, 0, 0), glm::quat(glm::vec3(0, 0, 0)), glm::vec3(0), 2, glm::vec3(0.5), false);
    // }

    // for (size_t i = NUM_SPRINGS + 1; i < NUM_BODIES; i++)
    // {
    //     bodies[i] = Body(glm::vec3(5, 5, 5 + i * .75), glm::vec3(0, 0, 0), glm::quat(glm::vec3(0, 0, 0)), glm::vec3(0), 2, glm::vec3(0.5), false);
    // }

    SummonXxYxZBlocks(bodies, glm::vec3(0), glm::vec3(1), glm::vec3(1.25), 5, 5, 5, 2);

    // Floor
    auto floor = Body(glm::vec3(0,0,-5), glm::vec3(0), glm::quat(glm::vec3(0)), glm::vec3(0), 1000, glm::vec3(50,50,1), true);
    bodies.push_back(floor);

    // ALWAYS init springs after bodies cuz of them pointers
    // Not great i know but its fine for now i guess

    // for (size_t i = 0; i < NUM_SPRINGS; i++)
    // {
    //     springs[i] = Spring(bodies[i], bodies[i + 1], 1, 1);
    // }
    ConnectXxYxZBlocks(springs, bodies, 0, 5, 5, 5, 1.25, 4);
}

void Complex::simulateStep()
{
    if (!paused || oneStep)
    {
        // Check collisions
        for (size_t i = 0; i < bodies.size() - 1; i++)
        {
            for (size_t j = i + 1; j < bodies.size(); j++)
            {
                bodies[i].doCollide(bodies[j], c);
            }
        }

        for (size_t i = 0; i < bodies.size(); i++)
        {
            // gravity
            bodies[i].applyDirectForce(glm::vec3(0, 0, gravity));

            // integration
            bodies[i].integrate(dt);
        }

        for (size_t i = 0; i < springs.size(); i++)
        {
            springs[i].computeElasticForces(dt);
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
                bodies[i].applyForceAt(hitPoint, forceStrength * rel);
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
        bodies[i].draw(renderer);
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
    ImGui::SliderFloat("Dt", &dt, 0, 0.1f);
    ImGui::SliderFloat("Bouncyness", &c, 0, 1);
    ImGui::Checkbox("Paused", &paused);
    auto btnOneStep = ImGui::Button("Step");
    if (btnOneStep)
    {
        oneStep = true;
    }
    // ImGui::Checkbox("Apply Force", &applyForce);
    ImGui::Text("Right click a body to apply a force.");
    ImGui::SliderFloat("Click strength", &forceStrength, 0, 10);
}