#include "SceneSingleStep.h"

#define TIMESTEP 2.0f

void SceneSingleStep::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
}

void SceneSingleStep::init()
{
    auto rb = CreateBoxRigidbody(ZERO_VECTOR,
                                 glm::vec3(1.0f, 0.6f, 0.5f),
                                 ZERO_VECTOR,
                                 2.0f,
                                 glm::quat(glm::vec3(0.0f, 0.0f, 0.5f * M_PI)), // 90 degrees around z (yaw)
                                 glm::vec3(0.0f));

    auto rigidbodies = std::vector<Rigidbody>({rb});

    auto points = std::vector<Point>({
        CreatePoint(glm::vec3(0.3f, 0.5f, 0.25f), rigidbodies, 0),    // for applying the force
        CreatePoint(glm::vec3(-0.3f, -0.5f, -0.25f), rigidbodies, 0), // for reading requested results
    });

    auto forces = std::vector<glm::vec3>({glm::vec3(1, 1, 0), ZERO_VECTOR});

    // print initial state of rb and point
    printf("Initial states:\n");
    rb.PrintState();
    points[1].PrintState();

    UpdateRigidbodyStep(rb, points, forces, TIMESTEP);

    printf("After %.2f s:\n", TIMESTEP);
    rb.PrintState();
    points[1].PrintState();
}