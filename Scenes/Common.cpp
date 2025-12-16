#include "Common.h"

void explicitStep(TempField &temp_field, float delta_t)
{
    // New field
    auto next_field = std::vector<std::vector<float>>(temp_field.getN(), std::vector<float>(temp_field.getM(), 0.0f));

    // Iterate over entire field
    for (int i = 0; i < temp_field.getN(); i++)
    {
        for (int j = 0; j < temp_field.getM(); j++)
        {
            explicitStepHelper(temp_field, next_field, i, j, delta_t);
        }
    }

    // Apply new field
    temp_field.setTempField(std::move(next_field));
}

void explicitStepHelper(TempField &current_temp_field, std::vector<std::vector<float>> &updated_temp_field, int i, int j, float delta_t)
{
    // Check for boundary conditions
    auto x_len = current_temp_field.getN();
    auto y_len = current_temp_field.getM();

    float T_center = current_temp_field[i][j];
    float T_x_dir_prev, T_x_dir_next, T_y_dir_prev, T_y_dir_next;

    T_x_dir_prev = 0 < i ? current_temp_field[i - 1][j] : 0;
    T_x_dir_next = i < x_len - 1 ? current_temp_field[i + 1][j] : 0;
    T_y_dir_prev = 0 < i ? current_temp_field[i][j - 1] : 0;
    T_y_dir_next = i < y_len - 1 ? current_temp_field[i][j + 1] : 0;

    auto delta_x = current_temp_field.deltaX();
    auto delta_y = current_temp_field.deltaY();

    auto d_dx2 = (T_x_dir_next - 2.0f * T_center + T_x_dir_prev) / (delta_x * delta_x);
    auto d_dy2 = (T_y_dir_next - 2.0f * T_center + T_y_dir_prev) / (delta_y * delta_y);

    auto delta_change = current_temp_field.getThermalDiffusivity() * (d_dx2 + d_dy2);

    auto new_temp = delta_change * delta_t + T_center;
    updated_temp_field[i][j] = new_temp;
}

TempField::TempField(float thermal_diffusivity, std::vector<std::vector<float>> initial_temp_field, float x_domain_lower, float x_domain_upper, float y_domain_lower, float y_domain_upper)
    : thermal_diffusivity(thermal_diffusivity),
      temp_field(std::move(initial_temp_field)),
      x_domain_lower(x_domain_lower),
      x_domain_upper(x_domain_upper),
      y_domain_lower(y_domain_lower),
      y_domain_upper(y_domain_upper)
{
}

TempField::TempField(float thermal_diffusivity, int n, int m, float x_domain_lower, float x_domain_upper, float y_domain_lower, float y_domain_upper)
    : thermal_diffusivity(thermal_diffusivity),
      temp_field(std::vector<std::vector<float>>(n, std::vector<float>(m, 0.0f))),
      x_domain_lower(x_domain_lower),
      x_domain_upper(x_domain_upper),
      y_domain_lower(y_domain_lower),
      y_domain_upper(y_domain_upper)
{
}

std::vector<float> &TempField::operator[](int i)
{
    return this->temp_field[i];
}

float TempField::deltaX()
{
    return (x_domain_upper - x_domain_lower) / n;
}

float TempField::deltaY()
{
    return (y_domain_upper - y_domain_lower) / m;
}

int TempField::totalSize()
{
    return this->n * this->m;
}
