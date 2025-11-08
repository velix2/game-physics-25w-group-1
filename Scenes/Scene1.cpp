#include "Scene1.h"
#include "Common.h"

void Scene1::init()
{
    float dt = 0.1;
    PointMass p1 = {glm::vec3(0, 0, 0), glm::vec3(-1, 0, 0), 10};
    PointMass p2 = {glm::vec3(0, 2, 0), glm::vec3(1, 0, 0), 10};
    Spring s = Spring(p1, p2, 1, 40);

    printf("## Initial Values:\n");
    printf("# Point 1\n");
    p1.printInfo();
    printf("# Point 2\n");
    p2.printInfo();

    // Euler
    s.computeElasticForces(dt);
    p1.integrateEuler(dt);
    p2.integrateEuler(dt);
    printf("\n## After 0.1s using Euler Method:\n");
    printf("# Point 1\n");
    p1.printInfo();
    printf("# Point 2\n");
    p2.printInfo();

    // Midpoint
    p1 = {glm::vec3(0, 0, 0), glm::vec3(-1, 0, 0), 10};
    p2 = {glm::vec3(0, 2, 0), glm::vec3(1, 0, 0), 10};
    s.computeElasticForces(dt);
    p1.integrateMidpoint1(dt);
    p2.integrateMidpoint1(dt);
    p1.force = glm::vec3(0);
    p2.force = glm::vec3(0);
    s.computeElasticForces(dt);
    p1.integrateMidpoint2(dt);
    p2.integrateMidpoint2(dt);
    printf("\n## After 0.1s using Midpoint Method:\n");
    printf("# Point 1\n");
    p1.printInfo();
    printf("# Point 2\n");
    p2.printInfo();
}
