#include "SingleStep.h"

void SingleStep::init()
{
    Body b = Body(glm::vec3(0), glm::vec3(0), glm::quat(glm::vec3(0, 0, glm::pi<float>() / 2.0f)), glm::vec3(0), 2, glm::vec3(1, 0.6, 0.5), false);
    printf("Initial States:\n");
    b.print();
    b.printPoint(glm::vec3(-0.3, -0.5, -0.25));
    b.applyForceAt(glm::vec3(0.3, 0.5, 0.25), glm::vec3(1, 1, 0));
    b.integrate(2);
    printf("After 2.00 s:\n");
    b.print();
    b.printPoint(glm::vec3(-0.3, -0.5, -0.25));
}