#include "SceneSimulation.h"
#include <imgui.h>

#define RB_COLOR glm::vec4(0.2f, 0.3f, 0.8f, 1.0f)

void SceneSimulation::init()
{
    // Setup stuff, see SceneSingleStep
    auto rb = CreateBoxRigidbody(ZERO_VECTOR,
                                 glm::vec3(1.0f, 0.6f, 0.5f),
                                 ZERO_VECTOR,
                                 2.0f,
                                 glm::quat(glm::vec3(0.0f, 0.0f, 0.5f * M_PI)), // 90 degrees around z (yaw)
                                 glm::vec3(0.0f));

    rigidbodies = std::vector<Rigidbody>({rb});

    points = std::vector<Point>({
        CreatePoint(glm::vec3(0.3f, 0.5f, 0.25f), rigidbodies, 0),    // for applying the force
        CreatePoint(glm::vec3(-0.3f, -0.5f, -0.25f), rigidbodies, 0), // for reading requested results
    });

    forces = std::vector<glm::vec3>({glm::vec3(1, 1, 0), ZERO_VECTOR});
}

void SceneSimulation::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    // draw rigidbody
    auto rb = rigidbodies[0];
    renderer.drawCube(rb.x_cm_world, rb.rotation, rb.dimensions, RB_COLOR);

    // draw force points
    renderer.drawSphere(points[0].x_world, 0.01f);
    renderer.drawSphere(points[1].x_world, 0.01f);
}

void SceneSimulation::simulateStep()
{
    if (!should_run) return;

    UpdateRigidbodyStep(rigidbodies[0], points, forces, delta_t);

    // // clear forces
    // for (size_t i = 0; i < forces.size(); i++)
    // {
    //     forces[i] = ZERO_VECTOR;
    // }
    
}

void SceneSimulation::onGUI()
{
    ImGui::InputFloat("Delta Time", &delta_t);

    ImGui::Checkbox("Simulation running", &should_run);
}
