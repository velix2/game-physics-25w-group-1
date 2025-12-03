#include "SceneCollision.h"
#include <imgui.h>

void SceneCollision::init()
{
    // Setup rigidbodies with intial velocity
    rb1 = CreateBoxRigidbody(glm::vec3(0, -2.5f, 0),
                             glm::vec3(1),
                             glm::vec3(0, 1, 0),
                             10,
                             glm::quat(glm::vec3(0.25f * M_PI)), // 45 degrees around each axis
                             ZERO_VECTOR,
                             true);

    rb2 = CreateBoxRigidbody(glm::vec3(0, 2.5f, 0),
                             glm::vec3(1, 0.5f, 1),
                             glm::vec3(0, -5, 0),
                             1,
                             glm::quat(ZERO_VECTOR), // no rotation
                             ZERO_VECTOR,
                             true);
    //rb2 = CreateFixedBoxRigidbody(glm::vec3(0, 2.5f, 0),
    //                              glm::vec3(1, 0.5f, 1), glm::quat(ZERO_VECTOR), false);
}

void SceneCollision::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    // draw rigidbodies
    renderer.drawCube(rb1.x_cm_world, rb1.rotation, rb1.dimensions, RB_1_COLOR);
    renderer.drawCube(rb2.x_cm_world, rb2.rotation, rb2.dimensions, RB_2_COLOR);

    // draw points
    for (auto &p : rb1.points)
    {
        renderer.drawSphere(p.x_world, 0.02f);
    }
    for (auto &p : rb2.points)
    {
        renderer.drawSphere(p.x_world, 0.02f);
    }
}

void SceneCollision::simulateStep()
{
    if (!should_run)
        return;

    UpdateRigidbodyStep(rb1, delta_t);
    UpdateRigidbodyStep(rb2, delta_t);

    HandleCollision(rb1, rb2, coefficient_of_restitution);
}

void SceneCollision::onGUI()
{
    ImGui::InputFloat("Delta Time", &delta_t);
    ImGui::SliderFloat("Coefficient of restitution", &coefficient_of_restitution, 0, 1);

    ImGui::Checkbox("Simulation running", &should_run);
}
