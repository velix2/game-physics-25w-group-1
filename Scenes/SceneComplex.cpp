#include "SceneComplex.h"
#include <imgui.h>

void SceneComplex::init()
{
    // Set up floor
    auto floor = CreateFixedBoxRigidbody(glm::vec3(0, 0, -5), glm::vec3(10, 10, 1), glm::quat(ZERO_VECTOR), false);
    rbs.push_back(floor);
    rb_colors.push_back(FLOOR_COLOR);
}

void SceneComplex::onDraw(Renderer &renderer)
{
    // Camera Matrix
    cameraMatrix = renderer.camera.viewMatrix;
    cameraPos = renderer.camera.position;
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);

    for (int i = 0; i < rbs.size(); i++)
    {
        // draw rigidbodies
        auto rb = rbs[i];
        renderer.drawCube(rb.x_cm_world, rb.rotation, rb.dimensions, rb_colors[i]);

        // draw points
        for (auto &p : rb.points)
        {
            renderer.drawSphere(p.x_world, 0.02f);
        }
    }
}

void SceneComplex::simulateStep()
{
    if (!should_run)
        return;

    if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
        SpawnRigidbody();

    // Collision handling
    for (size_t i = 0; i < rbs.size() - 1; i++)
    {
        for (size_t j = i + 1; j < rbs.size(); j++)
        {
            HandleCollision(rbs[i], rbs[j], coefficient_of_restitution);
        }
    }

    auto has_interacted = false;
    glm::vec3 dx, dy;

    // Drag controls
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
    {
        has_interacted = true;
        auto drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        if (!(drag.x == 0 && drag.y == 0))
        {
            dx = drag.x * right;
            dy = -drag.y * up;
        }
    }

    for (auto &&rb : rbs)
    {
        // Gravity
        rb.ApplyGravity(gravity);

        // Interaction force
        if (has_interacted)
        {
            auto rb_to_cam_unit_vector = glm::normalize(cameraPos - rb.x_cm_world);

            rb.ApplyForce(Force({rb.x_cm_world + rb_to_cam_unit_vector * offset_user_force_to_cm, input_force_strength * (dx + dy)}));
        }

        // Step
        UpdateRigidbodyStep(rb, delta_t);
    }
}

void SceneComplex::onGUI()
{
    ImGui::InputFloat("Delta Time", &delta_t);
    ImGui::InputFloat("Gravity", &gravity);
    ImGui::SliderFloat("Coefficient of restitution", &coefficient_of_restitution, 0, 1);

    ImGui::Separator();

    ImGui::InputFloat("Interaction Force Strength", &input_force_strength);
    ImGui::InputFloat("Interaction Force Offset", &offset_user_force_to_cm);

    ImGui::Separator();

    ImGui::Checkbox("Simulation running", &should_run);

    if (should_run)
    {
        if (ImGui::Button("Spawn Rigidbody [SPACE]"))
            SpawnRigidbody();

        ImGui::Text("Interact with RMB + Drag! :)");
    }
}

void SceneComplex::SpawnRigidbody()
{
    auto random_pos = 2.5f * (2.0f * RandomVec3() - glm::vec3(1));
    random_pos.z = 0;
    auto rb = CreateBoxRigidbody(random_pos,
                                 glm::vec3(0.1f) + 1.25f * RandomVec3(),
                                 ZERO_VECTOR,
                                 2,
                                 glm::normalize(glm::quat(RandomVec3())),
                                 ZERO_VECTOR,
                                 true);

    auto random_color = glm::vec4(RandomVec3(), 1.0f);
    rb_colors.push_back(random_color);

    rbs.push_back(rb);
}
