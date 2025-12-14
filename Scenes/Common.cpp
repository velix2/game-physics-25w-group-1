#include "Common.h"

const glm::vec2 ZERO_VEC_2 = glm::vec2(0);

void ExplicitStep(const std::vector<std::vector<glm::vec2>> &current_temp_field, std::vector<std::vector<glm::vec2>> &updated_temp_field, int i, int j, float thermal_diffusivity, float delta_x, float delta_y, float delta_t)
{
    // Check for boundary conditions
    auto x_len = current_temp_field.size();
    auto y_len = current_temp_field[0].size();

    glm::vec2 T_center = current_temp_field[i][j];
    glm::vec2 T_x_dir_prev, T_x_dir_next, T_y_dir_prev, T_y_dir_next;

    T_x_dir_prev = 0 < i ? current_temp_field[i - 1][j] : ZERO_VEC_2;
    T_x_dir_next = i < x_len - 1 ? current_temp_field[i + 1][j] : ZERO_VEC_2;
    T_y_dir_prev = 0 < i ? current_temp_field[i][j - 1] : ZERO_VEC_2;
    T_y_dir_next = i < y_len - 1 ? current_temp_field[i][j + 1] : ZERO_VEC_2;

    auto d_dx2 = (T_x_dir_next - 2.0f * T_center + T_x_dir_prev) / (delta_x * delta_x);
    auto d_dy2 = (T_y_dir_next - 2.0f * T_center + T_y_dir_prev) / (delta_y * delta_y);

    auto delta_change = thermal_diffusivity * (d_dx2 + d_dy2);

    auto new_temp = delta_change * delta_t + T_center;
    updated_temp_field[i][j] = new_temp;
}
