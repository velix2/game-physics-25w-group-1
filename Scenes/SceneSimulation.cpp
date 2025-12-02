#include "SceneSimulation.h"
#include <imgui.h>

#define RB_COLOR glm::vec4(0.2f, 0.3f, 0.8f, 1.0f)

void SceneSimulation::init()
{
    // Setup stuff, see SceneSingleStep
    rb = CreateBoxRigidbody(ZERO_VECTOR,
                            glm::vec3(1.0f, 0.6f, 0.5f),
                            ZERO_VECTOR,
                            2.0f,
                            glm::quat(glm::vec3(0.0f, 0.0f, 0.5f * M_PI)), // 90 degrees around z (yaw)
                            glm::vec3(0.0f),
                            true);

    // Initial force
    rb.ApplyForce(Force({glm::vec3(0.3f, 0.5f, 0.25f), glm::vec3(1, 1, 0)}));
}

void SceneSimulation::onDraw(Renderer &renderer)
{
    // Camera Matrix
    cameraMatrix = renderer.camera.viewMatrix;
    cameraPos = renderer.camera.position;
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);

    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    // draw rigidbody
    renderer.drawCube(rb.x_cm_world, rb.rotation, rb.dimensions, RB_COLOR);

    // draw points
    for (auto &p : rb.points)
    {
        renderer.drawSphere(p.x_world, 0.02f);
    }

    if (is_dragging_with_mouse)
    {
        renderer.drawSphere(rb.x_cm_world, offset_user_force_to_cm, glm::vec4(0.7f,0.7f,0.7f,0.04f), Renderer::DrawFlags::unlit);
    }
}

void SceneSimulation::simulateStep()
{
    if (!should_run)
        return;

    // Drag controls
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right))
    {
        is_dragging_with_mouse = true;
    }
    else if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
    {
        is_dragging_with_mouse = false;
        auto drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        if (!(drag.x == 0 && drag.y == 0))
        {
            auto dx = drag.x * right;
            auto dy = -drag.y * up;

            auto rb_to_cam_unit_vector = glm::normalize(cameraPos - rb.x_cm_world);

            rb.ApplyForce(Force({rb.x_cm_world + rb_to_cam_unit_vector * offset_user_force_to_cm, input_force_strength * (dx + dy)}));
        }
    }

    UpdateRigidbodyStep(rb, delta_t);
}

void SceneSimulation::onGUI()
{
    ImGui::InputFloat("Delta Time", &delta_t);
    ImGui::InputFloat("Offset of Applied Force to CM", &offset_user_force_to_cm);

    ImGui::Checkbox("Simulation running", &should_run);
}
