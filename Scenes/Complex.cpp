#include "Complex.h"
#include <imgui.h>

void Complex::init()
{
    for (size_t i = 0; i < NUM_SPRINGS + 1; i++)
    {
        bodies[i] = Body(glm::vec3(0, 0, i * .75), glm::vec3(0, 0, 0), glm::quat(glm::vec3(0, 0, 0)), glm::vec3(0), 2, glm::vec3(0.5), false);
    }

    for (size_t i = NUM_SPRINGS + 1; i < NUM_BODIES; i++)
    {
        bodies[i] = Body(glm::vec3(5, 5,5 + i * .75), glm::vec3(0, 0, 0), glm::quat(glm::vec3(0, 0, 0)), glm::vec3(0), 2, glm::vec3(0.5), false);
    }

    // ALWAYS init springs after bodies cuz of them pointers
    // Not great i know but its fine for now i guess

    for (size_t i = 0; i < NUM_SPRINGS; i++)
    {
        springs[i] = Spring(bodies[i], bodies[i + 1], 1, 1);
     }
    
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