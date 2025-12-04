#pragma once

#include "Renderer.h"
#include <vector>

#include "util/CollisionDetection.h"

#define ZERO_VECTOR glm::vec3(0.0f)
#define M_PI 3.14159265358979323846

#define FLOAT_PRECISION "%.3f"
#define PRINT_VEC3_FORMAT "(" FLOAT_PRECISION ", " FLOAT_PRECISION ", " FLOAT_PRECISION ")"
#define PRINT_VEC3_ARGS(v) (v).x, (v).y, (v).z

#define PRINT_QUAT_FORMAT "(" FLOAT_PRECISION ", " FLOAT_PRECISION ", " FLOAT_PRECISION ", " FLOAT_PRECISION ")"
#define PRINT_QUAT_ARGS(q) (q).w, (q).x, (q).y, (q).z

struct Point
{
    glm::vec3 x_world;
    glm::vec3 v_world;
    glm::vec3 x_local;

    void PrintState()
    {
        printf("==================== Point state: ====================\n");
        printf("World Position:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(x_world));
        printf("Local Position:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(x_local));
        printf("Velocity:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(v_world));
        printf("======================================================\n");
        printf("\n");
    }
};

struct Force
{
    glm::vec3 position;
    glm::vec3 direction;
};

struct Rigidbody
{
    bool is_fixed;
    glm::vec3 x_cm_world;
    glm::vec3 v_cm_world;
    /// @brief M
    float total_mass;
    /// @brief r
    glm::quat rotation;
    /// @brief L
    glm::vec3 angular_momentum;
    /// @brief w
    glm::vec3 angular_velocity;
    /// @brief I^-1
    glm::mat3x3 inverse_inertia_tensor;
    /// @brief I_0^-1
    glm::mat3x3 initial_inverse_inertia_tensor;
    /// @brief q
    glm::vec3 torque;
    /// @brief F
    glm::vec3 total_force;

    glm::vec3 dimensions;

    std::vector<Point> points;

    glm::vec3 WorldPositionToLocalPosition(const glm::vec3 world) const
    {
        auto local_pos = world - x_cm_world;

        // Revert body's rotation
        local_pos = glm::inverse(glm::mat3_cast(rotation)) * local_pos;

        return local_pos;
    }

    void ApplyForce(const Force &force)
    {
        total_force += force.direction;

        auto local_force_position = force.position - x_cm_world;

        // update torque
        auto cross = glm::cross(local_force_position, force.direction);
        torque += cross;
    }

    void ApplyGravity(float g)
    {
        total_force += glm::vec3(0, 0, -g);
    }

    void PrintState()
    {
        printf("================== Rigidbody state: ==================\n");

        printf("Is fixed?\t\t");
        printf(is_fixed ? "yes\n" : "no\n");

        printf("CM Position:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(x_cm_world));

        printf("Velocity:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(v_cm_world));

        printf("Mass:\t\t\t" FLOAT_PRECISION "\n", total_mass);

        printf("Rotation:\t\t" PRINT_QUAT_FORMAT "\n", PRINT_QUAT_ARGS(rotation));
        printf("Rotation Matrix:\n");
        auto rot_mat = glm::mat3_cast(rotation);
        PrintMat3x3(rot_mat);

        printf("Angular Momentum:\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(angular_momentum));

        printf("Angular Velocity:\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(angular_velocity));

        printf("Current Inverted Inertia Tensor:\n");
        PrintMat3x3(inverse_inertia_tensor);

        printf("Initial Inverted Inertia Tensor:\n");
        PrintMat3x3(initial_inverse_inertia_tensor);

        printf("======================================================\n");

        printf("\n");
    }

    void PrintMat3x3(glm::mat3 &m)
    {
        for (int i = 0; i < 3; ++i)
        {
            printf("| %5.2f %5.2f %5.2f |\n",
                   m[0][i],
                   m[1][i],
                   m[2][i]);
        }
    }

    glm::vec3 GetVelocityAt(glm::vec3 worldPos)
    {
        return v_cm_world + glm::cross(angular_velocity, WorldPositionToLocalPosition(worldPos));
    }
};

inline glm::vec3 EulerStep(glm::vec3 x, glm::vec3 x_prime, float h);

void UpdateRigidbodyStep(Rigidbody &rigidbody, float delta_t);

Point CreatePointOnRigidbody(glm::vec3 world_pos, Rigidbody &rb);

Rigidbody CreateBoxRigidbody(glm::vec3 world_pos_center, glm::vec3 dimensions, glm::vec3 world_initial_velocity, float mass, glm::quat initial_rotation, glm::vec3 initial_angular_momentum, bool generate_corner_points);

Rigidbody CreateFixedBoxRigidbody(glm::vec3 world_pos_center, glm::vec3 dimensions, glm::quat initial_rotation, bool generate_corner_points);

glm::mat4x4 GetObjectMatrix(const glm::vec3 &translation, const glm::vec3 &scale, const glm::quat &rotation);

void HandleCollision(Rigidbody &rb1, Rigidbody &rb2, float c);
void CalculateAndApplyImpulse(Rigidbody &rb_A, Rigidbody &rb_B, const CollisionInfo &info, float c);

glm::vec3 RandomVec3();