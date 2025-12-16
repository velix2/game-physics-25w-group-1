#include "SceneImplicit.h"

#include "imgui.h"

void SceneImplicit::simulateStep()
{
    if (!is_running)
        return;

    explicitStep(temp_field, delta_t);
}

void SceneImplicit::onGUI()
{
    ImGui::SliderFloat("Delta time", &delta_t, 0.005f, 0.1f);

    auto thermal_diff = temp_field.getThermalDiffusivity();
    ImGui::SliderFloat("Thermal diffusivity", &thermal_diff, 0.005f, 0.1f);
    if (thermal_diff != temp_field.getThermalDiffusivity())
        temp_field.setThermalDiffusivity(thermal_diff);

    ImGui::Checkbox("Simulation runnning", &is_running);
}

void SceneImplicit::onDraw(Renderer &r)
{
    auto corner_point = glm::vec3(-0.5f * rendering_horizontal_scale * (temp_field.getXDomainUpper() - temp_field.getXDomainLower()),
                                  -0.5f * rendering_horizontal_scale * (temp_field.getYDomainUpper() - temp_field.getYDomainLower()),
                                  0);

    // Base side x
    for (size_t i = 1; i < temp_field.getM(); i++)
    {
        auto local_pos_center = glm::vec3(rendering_horizontal_scale * i * temp_field.deltaX(), 0, rendering_vertical_scale * temp_field[i][0]);
        auto local_pos_south = glm::vec3(rendering_horizontal_scale * (i - 1) * temp_field.deltaX(), 0, rendering_vertical_scale * temp_field[i - 1][0]);

        r.drawSphere(corner_point + local_pos_center, 0.01, mapTemperatureToColor(min_temp, max_temp, local_pos_center.z / rendering_vertical_scale));
        auto line_south_temp = ((local_pos_center.z + local_pos_south.z) / rendering_vertical_scale) / 2.0f;

        r.drawLine(corner_point + local_pos_center, corner_point + local_pos_south, mapTemperatureToColor(min_temp, max_temp, line_south_temp));
    }

    // Base side y
    for (size_t j = 1; j < temp_field.getN(); j++)
    {
        auto local_pos_center = glm::vec3(0, rendering_horizontal_scale * j * temp_field.deltaY(), rendering_vertical_scale * temp_field[0][j]);
        auto local_pos_west = glm::vec3(0, rendering_horizontal_scale * (j - 1) * temp_field.deltaY(), rendering_vertical_scale * temp_field[0][j - 1]);

        r.drawSphere(corner_point + local_pos_center, 0.01, mapTemperatureToColor(min_temp, max_temp, local_pos_center.z));

        auto line_west_temp = ((local_pos_center.z + local_pos_west.z) / rendering_vertical_scale) / 2.0f;

        r.drawLine(corner_point + local_pos_center, corner_point + local_pos_west, mapTemperatureToColor(min_temp, max_temp, line_west_temp));
    }

    // "Inner" points
    for (size_t i = 1; i < temp_field.getM(); i++)
    {
        for (size_t j = 1; j < temp_field.getN(); j++)
        {
            auto local_pos_center = glm::vec3(rendering_horizontal_scale * i * temp_field.deltaX(), rendering_horizontal_scale * j * temp_field.deltaY(), rendering_vertical_scale * temp_field[i][j]);
            auto local_pos_south = glm::vec3(rendering_horizontal_scale * (i - 1) * temp_field.deltaX(), rendering_horizontal_scale * j * temp_field.deltaY(), rendering_vertical_scale * temp_field[i - 1][j]);
            auto local_pos_west = glm::vec3(rendering_horizontal_scale * i * temp_field.deltaX(), rendering_horizontal_scale * (j - 1) * temp_field.deltaY(), rendering_vertical_scale * temp_field[i][j - 1]);

            r.drawSphere(corner_point + local_pos_center, 0.01, mapTemperatureToColor(min_temp, max_temp, local_pos_center.z));

            auto line_west_temp = ((local_pos_center.z + local_pos_west.z) / rendering_vertical_scale) / 2.0f;
            auto line_south_temp = ((local_pos_center.z + local_pos_south.z) / rendering_vertical_scale) / 2.0f;

            r.drawLine(corner_point + local_pos_center, corner_point + local_pos_west, mapTemperatureToColor(min_temp, max_temp, line_west_temp));
            r.drawLine(corner_point + local_pos_center, corner_point + local_pos_south, mapTemperatureToColor(min_temp, max_temp, line_south_temp));
        }
    }
}
