#include "Common.h"

void UpdateRigidbodyStep(Rigidbody &rigidbody, float delta_t)
{
    if (rigidbody.is_fixed)
        return;

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

Rigidbody CreateFixedBoxRigidbody(glm::vec3 world_pos_center, glm::vec3 dimensions, glm::quat initial_rotation, bool generate_corner_points)
{

    auto rb = Rigidbody({
        true,
        world_pos_center,
        ZERO_VECTOR,
        10000000,
        initial_rotation,
        ZERO_VECTOR,
        ZERO_VECTOR,
        glm::mat3(0), // inertia tensor set to zero cuz fixed
        glm::mat3(0), // inertia tensor set to zero cuz fixed
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

glm::mat4x4 GetObjectMatrix(const glm::vec3 &translation, const glm::vec3 &scale, const glm::quat &rotation)
{
    glm::mat4 rotationMatrix = glm::toMat4(rotation);
    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1), scale);
    glm::mat4 translationMatrix = glm::translate(glm::mat4(1), translation);
    return translationMatrix * rotationMatrix * scaleMatrix;
}

void HandleCollision(Rigidbody &rb1, Rigidbody &rb2, float c)
{
    auto rb1_mat = GetObjectMatrix(rb1.x_cm_world, rb1.dimensions, rb1.rotation);
    auto rb2_mat = GetObjectMatrix(rb2.x_cm_world, rb2.dimensions, rb2.rotation);

    auto coll_info = collisionTools::checkCollisionSAT(rb1_mat, rb2_mat);

    if (!coll_info.isColliding)
        return;

    CalculateAndApplyImpulse(rb1, rb2, coll_info, c);
}

void CalculateAndApplyImpulse(Rigidbody &rb_A, Rigidbody &rb_B, const CollisionInfo &info, float c)
{
    auto v_rel = rb_A.GetVelocityAt(info.collisionPointWorld) - rb_B.GetVelocityAt(info.collisionPointWorld);
    auto x_a = info.collisionPointWorld - rb_A.x_cm_world;
    auto x_b = info.collisionPointWorld - rb_B.x_cm_world;

    auto n = info.normalWorld;
    auto v_rel_dot_n = glm::dot(v_rel, n);

    if (v_rel_dot_n > 0)
        return; // bodies are separating, return early

    float numerator = -(1 + c) * v_rel_dot_n;

    float denominator_A = 0;
    if (!rb_A.is_fixed)
        denominator_A = 1 / rb_A.total_mass + glm::dot(rb_A.inverse_inertia_tensor * glm::cross(glm::cross(x_a, n), x_a), n);

    float denominator_B = 0;
    if (!rb_B.is_fixed)
        denominator_B = 1 / rb_B.total_mass + glm::dot(rb_B.inverse_inertia_tensor * glm::cross(glm::cross(x_b, n), x_b), n);

    auto J = numerator / (denominator_A + denominator_B);

    if (!rb_A.is_fixed)
    {
        rb_A.v_cm_world = rb_A.v_cm_world + J * n / rb_A.total_mass;
        rb_A.angular_momentum = rb_A.angular_momentum + glm::cross(x_a, J * n);
    }
    if (!rb_B.is_fixed)
    {
        rb_B.v_cm_world = rb_B.v_cm_world - J * n / rb_B.total_mass;
        rb_B.angular_momentum = rb_B.angular_momentum - glm::cross(x_b, J * n);
    }
}
/// @brief Returns a random vec3 with each component in [0,1]
/// @return a random vec3 with each component in [0,1]
glm::vec3 RandomVec3()
{
    return glm::vec3(static_cast<float>(rand()) / RAND_MAX,
                     static_cast<float>(rand()) / RAND_MAX,
                     static_cast<float>(rand()) / RAND_MAX);
}