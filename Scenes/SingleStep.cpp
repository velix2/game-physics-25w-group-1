#include "SingleStep.h"
#include <array>
#include "Common.h"

void SingleStep::init()
{
    std::array<std::array<double, 6>, 3> field = {{{6, 5, 1, -1, -2, -1},
                                                   {4, 3, 0, -1, -3, -1},
                                                   {3, 2, -1, -2, -4, -2}}};
    std::array<std::array<double, 6>, 3> buffer = {0};
    printf("Initial values:\n");
    printField(field);
    float nu = 0.1f;
    float dt = 0.1f;
    float xdomain = 4;
    float ydomain = 2;
    explicitStep(field, buffer, nu, dt, xdomain, ydomain);
    printf("After %.2f seconds with explicit euler:\n", dt);
    printField(buffer);
    implicitStep(field, buffer, nu, dt, xdomain, ydomain);
    printf("After %.2f seconds with implicit:\n", dt);
    printField(buffer);
}