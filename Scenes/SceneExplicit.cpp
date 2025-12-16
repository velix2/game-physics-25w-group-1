#include "SceneExplicit.h"

#include "imgui.h"

void SceneExplicit::simulateStep()
{
    if (!is_running)
        return;

    explicitStep(temp_field, delta_t);
}

void SceneExplicit::onGUI()
{
    ImGui::SliderFloat("Delta time", &delta_t, 0.005f, 0.1f);

    auto thermal_diff = temp_field.getThermalDiffusivity();
    ImGui::SliderFloat("Thermal diffusivity", &thermal_diff, 0.005f, 0.1f);
    if (thermal_diff != temp_field.getThermalDiffusivity())
        temp_field.setThermalDiffusivity(thermal_diff);

    ImGui::Checkbox("Simulation runnning", &is_running);
}

void SceneExplicit::onDraw(Renderer &r)
{
    auto corner_point = glm::vec3(-0.5f * rendering_horizontal_scale * (temp_field.getXDomainUpper() - temp_field.getXDomainLower()),
                                  -0.5f * rendering_horizontal_scale * (temp_field.getYDomainUpper() - temp_field.getYDomainLower()),
                                  0);

    // Base side x
    for (size_t i = 1; i < temp_field.getM(); i++)
    {
        auto local_pos_center = glm::vec3(rendering_horizontal_scale * i * temp_field.deltaX(), 0, rendering_vertical_scale * temp_field[i][0]);
        auto local_pos_south = glm::vec3(rendering_horizontal_scale * (i - 1) * temp_field.deltaX(), 0, rendering_vertical_scale * temp_field[i - 1][0]);

        r.drawSphere(corner_point + local_pos_center, 0.01);

        r.drawLine(corner_point + local_pos_center, corner_point + local_pos_south, glm::vec4(1));
    }

    // Base side y
    for (size_t j = 1; j < temp_field.getN(); j++)
    {
        auto local_pos_center = glm::vec3(0, rendering_horizontal_scale * j * temp_field.deltaY(), rendering_vertical_scale * temp_field[0][j]);
        auto local_pos_west = glm::vec3(0, rendering_horizontal_scale * (j - 1) * temp_field.deltaY(), rendering_vertical_scale * temp_field[0][j - 1]);

        r.drawSphere(corner_point + local_pos_center, 0.01);

        r.drawLine(corner_point + local_pos_center, corner_point + local_pos_west, glm::vec4(1));
    }

    // "Inner" points
    for (size_t i = 1; i < temp_field.getM(); i++)
    {
        for (size_t j = 1; j < temp_field.getN(); j++)
        {
            auto local_pos_center = glm::vec3(rendering_horizontal_scale * i * temp_field.deltaX(), rendering_horizontal_scale * j * temp_field.deltaY(), rendering_vertical_scale * temp_field[i][j]);
            auto local_pos_south = glm::vec3(rendering_horizontal_scale * (i - 1) * temp_field.deltaX(), rendering_horizontal_scale * j * temp_field.deltaY(), rendering_vertical_scale * temp_field[i - 1][j]);
            auto local_pos_west = glm::vec3(rendering_horizontal_scale * i * temp_field.deltaX(), rendering_horizontal_scale * (j - 1) * temp_field.deltaY(), rendering_vertical_scale * temp_field[i][j - 1]);

            r.drawSphere(corner_point + local_pos_center, 0.01);

            r.drawLine(corner_point + local_pos_center, corner_point + local_pos_south, glm::vec4(1));
            r.drawLine(corner_point + local_pos_center, corner_point + local_pos_west, glm::vec4(1));
        }
    }
}
