#include "Common.h"

void UpdateRigidbodyStep(Rigidbody &rigidbody, float delta_t)
{
    // Euler step for x_cm
    rigidbody.x_cm_world = EulerStep(rigidbody.x_cm_world, rigidbody.v_cm_world, delta_t);

    // Euler step for v_cm
    rigidbody.v_cm_world = EulerStep(rigidbody.v_cm_world, rigidbody.total_force / rigidbody.total_mass, delta_t);

    // Integrate rotation
    rigidbody.rotation = rigidbody.rotation + (delta_t / 2.0f) * glm::quat(0.0f, rigidbody.angular_velocity) * rigidbody.rotation;
    // Normalize quat
    rigidbody.rotation = glm::normalize(rigidbody.rotation);

    // Integrate angular momentum
    rigidbody.angular_momentum = EulerStep(rigidbody.angular_momentum, rigidbody.torque, delta_t);

    auto rotation_matrix = glm::mat3_cast(rigidbody.rotation);
    auto inverse_rotation_matrix = glm::inverse(rotation_matrix);

    // Update inverse inertia tensor
    rigidbody.inverse_inertia_tensor = inverse_rotation_matrix * rigidbody.initial_inverse_inertia_tensor * rotation_matrix;

    // Calculate new angular velocity
    rigidbody.angular_velocity = rigidbody.inverse_inertia_tensor * rigidbody.angular_momentum;

    // Update points' world position
    for (size_t i = 0; i < rigidbody.points.size(); i++)
    {
        rigidbody.points[i].x_world = rigidbody.x_cm_world + rotation_matrix * rigidbody.points[i].x_local;
        rigidbody.points[i].v_world = rigidbody.v_cm_world + glm::cross(rigidbody.angular_velocity, rigidbody.points[i].x_local);
    }

    // Clear force, torque when done
    rigidbody.total_force = ZERO_VECTOR;
    rigidbody.torque = ZERO_VECTOR;
}

inline glm::vec3 EulerStep(glm::vec3 x, glm::vec3 x_prime, float h)
{
    return x + h * x_prime;
}

Point CreatePointOnRigidbody(glm::vec3 world_pos, Rigidbody &rb)
{
    auto local_pos = world_pos - rb.x_cm_world;

    // Revert body's rotation
    local_pos = glm::inverse(glm::mat3_cast(rb.rotation)) * local_pos;

    auto p = Point({world_pos, glm::vec3(0.0f), local_pos});
    rb.points.push_back(p);
    return p;
}

Rigidbody CreateBoxRigidbody(glm::vec3 world_pos_center, glm::vec3 dimensions, glm::vec3 world_initial_velocity, float mass, glm::quat initial_rotation, glm::vec3 initial_angular_momentum, bool generate_corner_points)
{
    auto initial_inertia_tensor = glm::mat3(0.0f);

    // values from https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    initial_inertia_tensor[0][0] = mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z) / 12.0;
    initial_inertia_tensor[1][1] = mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z) / 12.0;
    initial_inertia_tensor[2][2] = mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y) / 12.0;

    auto initial_inverse_inertia_tensor = glm::inverse(initial_inertia_tensor);

    auto rot_matrix = glm::mat3_cast(initial_rotation);
    auto rot_matrix_T = glm::transpose(rot_matrix);

    auto inverse_inertia_tensor = rot_matrix * initial_inverse_inertia_tensor * rot_matrix_T;

    auto angular_velocity = inverse_inertia_tensor * initial_angular_momentum;

    auto rb = Rigidbody({
        false,
        world_pos_center,
        world_initial_velocity,
        mass,
        initial_rotation,
        initial_angular_momentum,
        angular_velocity,
        inverse_inertia_tensor,
        initial_inverse_inertia_tensor,
        ZERO_VECTOR,
        ZERO_VECTOR,
        dimensions,
    });
    rb.points = std::vector<Point>();

    // Corner point creation
    if (generate_corner_points)
    {
        rb.points.resize(8);
        glm::vec3 half = 0.5f * dimensions;

        int index = 0;
        for (int xi = -1; xi <= 1; xi += 2)
        {
            for (int yi = -1; yi <= 1; yi += 2)
            {
                for (int zi = -1; zi <= 1; zi += 2)
                {
                    glm::vec3 local = glm::vec3(xi * half.x, yi * half.y, zi * half.z);
                    rb.points[index].x_local = local;
                    rb.points[index].x_world = rb.x_cm_world + rb.rotation * local;
                    rb.points[index].v_world = ZERO_VECTOR;
                    index++;
                }
            }
        }
    }

    return rb;
}
