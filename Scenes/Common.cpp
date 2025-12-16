#include "Common.h"

#include <random>
#include <chrono>

void explicitStep(TempField &temp_field, float delta_t)
{
    // New field
    auto next_field = std::vector<std::vector<float>>(temp_field.getM(), std::vector<float>(temp_field.getN(), 0.0f));

    // Iterate over entire field
    for (int i = 0; i < temp_field.getM(); i++)
    {
        for (int j = 0; j < temp_field.getN(); j++)
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
    auto x_len = current_temp_field.getM();
    auto y_len = current_temp_field.getN();

    float T_center = current_temp_field[i][j];
    float T_x_dir_prev, T_x_dir_next, T_y_dir_prev, T_y_dir_next;

    T_x_dir_prev = 0 < i ? current_temp_field[i - 1][j] : 0;
    T_x_dir_next = i < x_len - 1 ? current_temp_field[i + 1][j] : 0;
    T_y_dir_prev = 0 < j ? current_temp_field[i][j - 1] : 0;
    T_y_dir_next = j < y_len - 1 ? current_temp_field[i][j + 1] : 0;

    auto delta_x = current_temp_field.deltaX();
    auto delta_y = current_temp_field.deltaY();

    auto d_dx2 = (T_x_dir_next - 2.0f * T_center + T_x_dir_prev) / (delta_x * delta_x);
    auto d_dy2 = (T_y_dir_next - 2.0f * T_center + T_y_dir_prev) / (delta_y * delta_y);

    auto delta_change = current_temp_field.getThermalDiffusivity() * (d_dx2 + d_dy2);

    auto new_temp = delta_change * delta_t + T_center;
    updated_temp_field[i][j] = new_temp;
}

TempField::TempField()
    : thermal_diffusivity(0.1f),
      temp_field(std::vector<std::vector<float>>(0, std::vector<float>(0, 0.0f))),
      x_domain_lower(0.0f),
      x_domain_upper(1.0f),
      y_domain_lower(0.0f),
      y_domain_upper(1.0f)
{
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

TempField::TempField(float thermal_diffusivity, int m, int n, float x_domain_lower, float x_domain_upper, float y_domain_lower, float y_domain_upper)
    : thermal_diffusivity(thermal_diffusivity),
      temp_field(std::vector<std::vector<float>>(m, std::vector<float>(n, 0.0f))),
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
    return (x_domain_upper - x_domain_lower) / getN();
}

float TempField::deltaY()
{
    return (y_domain_upper - y_domain_lower) / getM();
}

int TempField::getM()
{
    return this->temp_field.size();
}

int TempField::getN()
{
    if (getM() == 0)
        return 0;
    return this->temp_field[0].size();
}

int TempField::totalSize()
{
    return getN() * getM();
}

std::vector<std::vector<float>> generateRandomField(int m, int n, float min_val, float max_val)
{
    // Initialize the 2D vector
    std::vector<std::vector<float>> random_field(m, std::vector<float>(n));

    // Use the current time as seed
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator(seed);

    std::uniform_real_distribution<float> distribution(min_val, max_val);

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            // Generate a random float and assign it
            random_field[i][j] = distribution(generator);
        }
    }

    return random_field;
}

template <typename T>
inline T inverse_lerp(T a, T b, T x)
{
    if (a == b)
    {
        return 0.0;
    }

    return std::clamp((x - a) / (b - a), 0.0f, 1.0f);
}

glm::vec4 mapTemperatureToColor(float min, float max, float temp)
{
    auto interpol = inverse_lerp(min, max, temp);
    return glm::mix(COLD_COLOR, WARM_COLOR, interpol);
}
