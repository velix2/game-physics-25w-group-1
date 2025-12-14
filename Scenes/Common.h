#ifndef COMMON_H_
#define COMMON_H_

#include <glm.hpp>
#include <vector>

void ExplicitStep(const std::vector<std::vector<glm::vec2>> &current_temp_field, std::vector<std::vector<glm::vec2>> &updated_temp_field, int i, int j, float thermal_diffusivity, float delta_x, float delta_y, float delta_t);

#endif