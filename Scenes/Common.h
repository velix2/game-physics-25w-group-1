#ifndef COMMON_H_
#define COMMON_H_

#include "Renderer.h"
#include <vector>

class TempField
{

private:
    std::vector<std::vector<float>> temp_field;
    float thermal_diffusivity;
    float x_domain_lower, x_domain_upper;
    float y_domain_lower, y_domain_upper;

public:
    /**
     * @brief Default constructor for TempField, initializes to zero-sized grid.
     */
    TempField();

    /**
     * @brief Constructs a TempField object with a given thermal diffusivity, initial temperature field, and domain bounds.
     *
     * @param thermal_diffusivity The thermal diffusivity constant for the simulation.
     * @param initial_temp_field 2D vector representing the initial temperature distribution.
     * @param x_domain_lower Lower bound of the x-domain.
     * @param x_domain_upper Upper bound of the x-domain.
     * @param y_domain_lower Lower bound of the y-domain.
     * @param y_domain_upper Upper bound of the y-domain.
     */
    TempField(float thermal_diffusivity, std::vector<std::vector<float>> initial_temp_field, float x_domain_lower, float x_domain_upper, float y_domain_lower, float y_domain_upper);

    /**
     * @brief Constructs a TempField object with a given thermal diffusivity, grid size, and domain bounds.
     *        The temperature field is initialized to zero.
     *
     * @param thermal_diffusivity The thermal diffusivity constant for the simulation.
     * @param m Number of grid points in the x-direction.
     * @param n Number of grid points in the y-direction.
     * @param x_domain_lower Lower bound of the x-domain.
     * @param x_domain_upper Upper bound of the x-domain.
     * @param y_domain_lower Lower bound of the y-domain.
     * @param y_domain_upper Upper bound of the y-domain.
     */
    TempField(float thermal_diffusivity, int m, int n, float x_domain_lower, float x_domain_upper, float y_domain_lower, float y_domain_upper);

    std::vector<float> &operator[](int);

    /**
     * @brief Prints the temperature field in a nicely formatted way.
     */
    void print() const
    {
        for (const auto &row : temp_field)
        {
            for (const auto &val : row)
            {
                printf("%8.3f ", val);
            }
            printf("\n");
        }
    }

    // Calculated Attributes
    float deltaX();
    float deltaY();
    int getM();
    int getN();

    int totalSize();

    void applyFlattenedTemperatureField(std::vector<float> flattened);

    std::vector<float> getFlattenedTemperatureField();

    int flattenedIndexFrom2DIndex(int i, int j);

    // Getters
    const std::vector<std::vector<float>> &getTempField() const { return temp_field; }
    float getThermalDiffusivity() const { return thermal_diffusivity; }
    float getXDomainLower() const { return x_domain_lower; }
    float getXDomainUpper() const { return x_domain_upper; }
    float getYDomainLower() const { return y_domain_lower; }
    float getYDomainUpper() const { return y_domain_upper; }

    // Setters
    void setTempField(const std::vector<std::vector<float>> &field) { temp_field = field; }
    void setThermalDiffusivity(float diffusivity) { thermal_diffusivity = diffusivity; }
    void setXDomainLower(float value) { x_domain_lower = value; }
    void setXDomainUpper(float value) { x_domain_upper = value; }
    void setYDomainLower(float value) { y_domain_lower = value; }
    void setYDomainUpper(float value) { y_domain_upper = value; }
};

const glm::vec4 COLD_COLOR = glm::vec4(0, 0.1f, 0.9f, 1);
const glm::vec4 WARM_COLOR = glm::vec4(1, 0.3f, 0.1f, 1);

void explicitStep(TempField &temp_field, float delta_t);
void explicitStepHelper(TempField &current_temp_field, std::vector<std::vector<float>> &updated_temp_field, int i, int j, float delta_t);

void implicitStep(TempField &temp_field, float delta_t);

/**
 * @brief Generates a 2D float vector initialized with random noise.
 *
 * @param m Number of rows (y-dimension).
 * @param n Number of columns (x-dimension).
 * @param min_val The lower bound for the random float values (inclusive).
 * @param max_val The upper bound for the random float values (inclusive).
 * @return std::vector<std::vector<float>> The initialized 2D vector.
 */
std::vector<std::vector<float>> generatePixelWiseRandomField(int m, int n, float min_val, float max_val);

template <typename T>
T inverse_lerp(T a, T b, T x);

glm::vec4 mapTemperatureToColor(float min, float max, float temp);

#endif
