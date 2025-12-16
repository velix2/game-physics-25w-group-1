#ifndef COMMON_H_
#define COMMON_H_

#include <glm.hpp>
#include <vector>

class TempField
{

private:
    std::vector<std::vector<float>> temp_field;
    float thermal_diffusivity;
    int n, m;
    float x_domain_lower, x_domain_upper;
    float y_domain_lower, y_domain_upper;

public:
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
    TempField::TempField(float thermal_diffusivity, std::vector<std::vector<float>> initial_temp_field, float x_domain_lower, float x_domain_upper, float y_domain_lower, float y_domain_upper);

    /**
     * @brief Constructs a TempField object with a given thermal diffusivity, grid size, and domain bounds.
     *        The temperature field is initialized to zero.
     *
     * @param thermal_diffusivity The thermal diffusivity constant for the simulation.
     * @param n Number of grid points in the x-direction.
     * @param m Number of grid points in the y-direction.
     * @param x_domain_lower Lower bound of the x-domain (inclusive).
     * @param x_domain_upper Upper bound of the x-domain (exclusive).
     * @param y_domain_lower Lower bound of the y-domain (inclusive).
     * @param y_domain_upper Upper bound of the y-domain (exclusive).
     */
    TempField(float thermal_diffusivity, int n, int m, float x_domain_lower, float x_domain_upper, float y_domain_lower, float y_domain_upper);

    std::vector<float> &operator[](int);

    float deltaX();
    float deltaY();

    int totalSize();

    // Getters
    const std::vector<std::vector<float>> &getTempField() const { return temp_field; }
    float getThermalDiffusivity() const { return thermal_diffusivity; }
    int getN() const { return n; }
    int getM() const { return m; }
    float getXDomainLower() const { return x_domain_lower; }
    float getXDomainUpper() const { return x_domain_upper; }
    float getYDomainLower() const { return y_domain_lower; }
    float getYDomainUpper() const { return y_domain_upper; }

    // Setters
    void setTempField(const std::vector<std::vector<float>> &field) { temp_field = field; }
    void setThermalDiffusivity(float diffusivity) { thermal_diffusivity = diffusivity; }
    void setN(int value) { n = value; }
    void setM(int value) { m = value; }
    void setXDomainLower(float value) { x_domain_lower = value; }
    void setXDomainUpper(float value) { x_domain_upper = value; }
    void setYDomainLower(float value) { y_domain_lower = value; }
    void setYDomainUpper(float value) { y_domain_upper = value; }
};

void explicitStep(TempField &temp_field, float delta_t);
void explicitStepHelper(TempField &current_temp_field, std::vector<std::vector<float>> &updated_temp_field, int i, int j, float delta_t);

#endif