#pragma once

#include "Renderer.h"
#include <vector>

#define ZERO_VECTOR glm::vec3(0.0f)

#define FLOAT_PRECISION "%.2f"
#define PRINT_VEC3_FORMAT "(" FLOAT_PRECISION ", " FLOAT_PRECISION ", " FLOAT_PRECISION ")"
#define PRINT_VEC3_ARGS(v) (v).x, (v).y, (v).z

#define PRINT_QUAT_FORMAT "(" FLOAT_PRECISION ", " FLOAT_PRECISION ", " FLOAT_PRECISION ", " FLOAT_PRECISION ")"
#define PRINT_QUAT_ARGS(q) (q).w, (q).x, (q).y, (q).z

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

    glm::vec3 dimensions;

    void PrintState()
    {
        printf("================ Rigidbody state: ================\n");

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

        printf("==================================================\n");

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
};

struct Point
{
    glm::vec3 x_world;
    glm::vec3 v_world;
    size_t rb_idx;
    glm::vec3 x_local;

    void PrintState()
    {
        printf("================== Point state: ==================\n");
        printf("World Position:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(x_world));
        printf("Local Position:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(x_local));
        printf("Velocity:\t\t" PRINT_VEC3_FORMAT "\n", PRINT_VEC3_ARGS(v_world));
        printf("==================================================\n");
        printf("\n");
    }
};

inline glm::vec3 EulerStep(glm::vec3 x, glm::vec3 x_prime, float h);

Point CreatePoint(glm::vec3 world_pos, std::vector<Rigidbody> &rigidbodies, size_t rigidbody_index);

Rigidbody CreateBoxRigidbody(glm::vec3 world_pos_center, glm::vec3 dimensions, glm::vec3 world_initial_velocity, float mass, glm::quat initial_rotation, glm::vec3 initial_angular_momentum);

void UpdateRigidbodyStep(Rigidbody &rigidbody, std::vector<Point> &points, std::vector<glm::vec3> &forces_world, float delta_t);
