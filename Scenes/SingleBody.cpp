#include "SingleBody.h"
#include <imgui.h>

void SingleBody::init()
{
    body.applyForceAt(glm::vec3(0.3, 0.5, 0.25), glm::vec3(1, 1, 0));
}

void SingleBody::simulateStep()
{
    if (!paused || oneStep)
    {
        // if (applyForce)
        // {
        // }
        body.integrate(dt);
        oneStep = false;
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        auto pos = ImGui::GetMousePos();
        // printf("heyi");
        //  auto size = ImGui::GetWindowSize();
        // printf("heyo");
        // glm::vec3 rel = -glm::normalize((pos.x / windowSize.x - 0.5f) * right - (pos.y / windowSize.y - 0.5f) * up + cameraNear * fwd);
        glm::vec3 hitPoint;
        glm::vec3 rel = screenToWorldRay(projMatrix, cameraMatrix, pos.x, pos.y, windowSize.x, windowSize.y);
        if (body.intersectRay(cameraPosition, rel, hitPoint))
        {
            // printf("t %f, %f, %f\n", hitPoint.x, hitPoint.y, hitPoint.z);
            lastcast1 = hitPoint;
            lastcast2 = hitPoint + rel;
            body.applyForceAt(hitPoint, forceStrength * rel);
        }
        else
        {
            // printf("Nor\n");
        }
        // printf("t %f, %f, %f\n", rel.x, rel.y, rel.z);
        // printf("r %f, %f, %f\n", fwd.x, fwd.y, fwd.z);
        // printf("");
    }
}

void SingleBody::onDraw(Renderer &renderer)
{
    renderer.drawLine(lastcast1, lastcast2, glm::vec4(1, 1, 0, 1));
    renderer.drawLine(glm::vec3(0), glm::vec3(1, 0, 0), glm::vec4(1, 0, 0, 1));
    renderer.drawLine(glm::vec3(0), glm::vec3(0, 1, 0), glm::vec4(0, 1, 0, 1));
    renderer.drawLine(glm::vec3(0), glm::vec3(0, 0, 1), glm::vec4(0, 0, 1, 1));
    body.draw(renderer);
    // printf("%f", renderer.camera.near);
    projMatrix = renderer.camera.projectionMatrix();
    cameraMatrix = renderer.camera.viewMatrix;
    cameraPosition = renderer.camera.position;
    // cameraNear = renderer.camera.near;
    windowSize = glm::vec2(renderer.camera.width, renderer.camera.height);
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, -1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);
}

void SingleBody::onGUI()
{
    ImGui::SliderFloat("Dt", &dt, 0, 0.1f);
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