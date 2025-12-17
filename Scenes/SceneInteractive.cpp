#include "SceneInteractive.h"

#include "imgui.h"

void SceneInteractive::simulateStep()
{
    if (!is_running)
        return;

    if (is_using_implicit)
        implicitStep(temp_field, delta_t);
    else
        explicitStep(temp_field, delta_t);
}

void SceneInteractive::onGUI()
{
    ImGui::SliderFloat("Delta time", &delta_t, 0.005f, 0.1f);

    auto thermal_diff = temp_field.getThermalDiffusivity();
    ImGui::SliderFloat("Thermal diffusivity", &thermal_diff, 0.005f, 0.1f);
    if (thermal_diff != temp_field.getThermalDiffusivity())
        temp_field.setThermalDiffusivity(thermal_diff);

    ImGui::Checkbox("Use Implicit Method", &is_using_implicit);
    ImGui::Checkbox("Simulation runnning", &is_running);

    // Re-initialization
    static const char *items[] = {"Random Noise", "Waves", "Gaussian Blob"};

    ImGui::Separator();

    ImGui::Combo("Initial Condition", &selected_method, items, IM_ARRAYSIZE(items));

    switch (selected_method)
    {
    case 0: // Random
        ImGui::InputFloat("Min. Random Temp", &rand_min_val);
        ImGui::InputFloat("Max. Random Temp", &rand_max_val);
        break;
    case 1: // Waves
        ImGui::InputFloat("Amplitude", &sine_amplitude);
        ImGui::InputFloat("Waves", &sine_repetitions);
        break;
    case 2: // Gaussian
        ImGui::InputFloat("Amplitude", &gaussian_amplitude);
        ImGui::InputFloat("Sigma", &gaussian_sigma);
        ImGui::InputFloat2("Mean Pos", gaussian_mean_pos);
        break;
    }

    int arr[] = {m, n};
    ImGui::InputInt2("M/N", arr);
    m = arr[0];
    n = arr[1];

    ImGui::InputFloat("X-Domain Lower", &x_domain_lower_bound);
    ImGui::InputFloat("X-Domain Upper", &x_domain_upper_bound);

    ImGui::InputFloat("Y-Domain Lower", &y_domain_lower_bound);
    ImGui::InputFloat("Y-Domain Upper", &y_domain_upper_bound);

    if (ImGui::Button("Reinstantate Field"))
    {
        temp_field.setXDomainLower(x_domain_lower_bound);
        temp_field.setXDomainUpper(x_domain_upper_bound);

        temp_field.setYDomainLower(y_domain_lower_bound);
        temp_field.setYDomainUpper(y_domain_upper_bound);

        switch (selected_method)
        {
        case 0: // Random
            temp_field.setTempField(generatePixelWiseRandomField(m, n, rand_min_val, rand_max_val));
            break;
        case 1: // Waves
            temp_field.setTempField(generateSineWaveField(m, n, sine_amplitude, sine_repetitions));
            break;
        case 2: // Gaussian
            temp_field.setTempField(generateGaussianField(m, n, gaussian_amplitude, gaussian_mean_pos[0], gaussian_mean_pos[1], gaussian_sigma));
            break;
        }
    }
}

void SceneInteractive::onDraw(Renderer &r)
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
