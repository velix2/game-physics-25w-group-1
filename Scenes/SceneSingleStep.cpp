#include "SceneSingleStep.h"

void SceneSingleStep::init()
{
    std::vector<std::vector<float>> temp_field_array = {
        {6, 5, 1, -1, -2, -1},
        {4, 3, 0, -1, -3, -1},
        {3, 2, -1, -2, -4, -2}};

    temp_field = TempField(thermal_diffusivity, temp_field_array, 0.0f, 2.0f, 0.0f, 4.0f);

    printf("Initial Temperature Field:\n");
    temp_field.print();

    explicitStep(temp_field, delta_t);

    printf("Temperature Field after t=%.1f:\n", delta_t);
    temp_field.print();
}
