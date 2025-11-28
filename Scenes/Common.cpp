#include "Common.h"

void UpdateRigidbodyStep(Rigidbody &rigidbody, std::vector<Point> &points, std::vector<glm::vec3> &forces_world, float delta_t)
{   
    // calculate F (total force)
    glm::vec3 total_force = glm::vec3(0.f);
    for (auto &&force : forces_world)
    {
        total_force += force;
    }
    
    // calculate torque
    glm::vec3 torque = glm::vec3(0.f);
    for (size_t i = 0; i < points.size(); i++)
    {
        auto cross = glm::cross(points[i].x_local, forces_world[i]);
        torque += cross;
    }

    // Euler step for x_cm
    rigidbody.x_cm_world = EulerStep(rigidbody.x_cm_world, rigidbody.v_cm_world, delta_t);

    // Euler step for v_cm
    rigidbody.v_cm_world = EulerStep(rigidbody.v_cm_world, total_force / rigidbody.total_mass, delta_t);

    // Integrate rotation
    rigidbody.rotation = rigidbody.rotation + (delta_t / 2.0f) * glm::quat(0.0f, rigidbody.angular_velocity) * rigidbody.rotation;

    // Integrate angular momentum
    rigidbody.angular_momentum = EulerStep(rigidbody.angular_momentum, torque, delta_t);

    auto rotation_matrix = glm::mat3_cast(rigidbody.rotation);
    auto inverse_rotation_matrix = glm::inverse(rotation_matrix);

    // Update inverse inertia tensor
    rigidbody.inverse_inertia_tensor = inverse_rotation_matrix * rigidbody.initial_inverse_inertia_tensor * rotation_matrix;

    // Calculate new angular velocity
    rigidbody.angular_velocity = rigidbody.inverse_inertia_tensor * rigidbody.angular_momentum;

    // Update points' world position
    for (size_t i = 0; i < points.size(); i++)
    {
        points[i].x_world = rigidbody.x_cm_world + rotation_matrix * points[i].x_local;
        points[i].v_world = rigidbody.v_cm_world + glm::cross(rigidbody.angular_velocity, points[i].x_local);
    }
}

inline glm::vec3 EulerStep(glm::vec3 x, glm::vec3 x_prime, float h) {
    return x + h * x_prime;
}

Point CreatePoint(glm::vec3 world_pos, std::vector<Rigidbody> &rigidbodies, size_t rigidbody_index)
{
    auto local_pos = world_pos - rigidbodies[rigidbody_index].x_cm_world;

    // Revert body's rotation
    local_pos = glm::inverse(glm::mat3_cast(rigidbodies[rigidbody_index].rotation)) * local_pos;

    return Point({world_pos, glm::vec3(0.0f), rigidbody_index, local_pos});
}

Rigidbody CreateBoxRigidbody(glm::vec3 world_pos_center, glm::vec3 dimensions, glm::vec3 world_initial_velocity, float mass, glm::quat initial_rotation, glm::vec3 initial_angular_momentum)
{
    auto inertia_tensor = glm::mat3(0.0f);

    // values from https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    inertia_tensor[0][0] = mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z) / 12.0;
    inertia_tensor[1][1] = mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z) / 12.0;
    inertia_tensor[2][2] = mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y) / 12.0;

    auto inverse_inertia_tensor = glm::inverse(inertia_tensor);

    auto angular_velocity = inverse_inertia_tensor * initial_angular_momentum;

    return Rigidbody({
        false,
        world_pos_center,
        world_initial_velocity,
        mass,
        initial_rotation,
        initial_angular_momentum,
        angular_velocity,
        inverse_inertia_tensor,
        inverse_inertia_tensor
    });
}
