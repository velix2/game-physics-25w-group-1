#include "SceneComplex.h"
#include <imgui.h>

int clamp(float &value, float min, float max)
{
    if (value < min)
    {
        value = min;
        return -1;
    }
    else if (value > max)
    {
        value = max;
        return 1;
    }
    return 0;
}

void SceneComplex::addPoint()
{
    if (pointCount == MAX_POINTS)
    {
        return;
    }
    points[pointCount++] = PointMass(
        glm::vec3(dis(gen) * 5.0f - 2.5f, dis(gen) * 5.0f - 2.5f, dis(gen) * 5.0f - 2.5f),
        glm::vec3(dis(gen) * 1.0f - 0.5f, dis(gen) * 1.0f - 0.5f, dis(gen) * 1.0f - 0.5f),
        dis(gen) * 10);
}

void SceneComplex::addSpring()
{
    if (springCount == MAX_POINTS)
    {
        return;
    }
    int p1 = static_cast<int>(dis(gen) * pointCount);
    int p2 = static_cast<int>(dis(gen) * pointCount);
    if (p2 == p1)
    {
        p2++;
        if (p2 == pointCount)
        {
            p2 -= 2;
        }
    }
    springs[springCount++] = Spring(points[p1], points[p2], dis(gen) * 2.0f, dis(gen) * 40.0f + 10.0f);
}

void SceneComplex::init()
{
    pointCount = 0;
    points[pointCount++] = PointMass(glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(1, 0, 0), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(0, 1, 0), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(1, 1, 0), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(0, 0, 1), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(1, 0, 1), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(0, 1, 1), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(1, 1, 1), glm::vec3(0, 0, 0), 10);
    points[pointCount++] = PointMass(glm::vec3(0.5, 0.5, 0.5), glm::vec3(0, 0, 0), 100);
    points[pointCount++] = PointMass(glm::vec3(0, 0, 2.5), glm::vec3(0, 1, 1), 10, true);

    for (; pointCount < INITIAL_POINTS;)
    {
        addPoint();
    }
    springCount = 0;
    float outer_stiffness = 40000;
    // bottom layer
    springs[springCount++] = Spring(points[0], points[1], 1, outer_stiffness);
    springs[springCount++] = Spring(points[0], points[2], 1, outer_stiffness);
    springs[springCount++] = Spring(points[3], points[1], 1, outer_stiffness);
    springs[springCount++] = Spring(points[3], points[2], 1, outer_stiffness);
    // connection
    springs[springCount++] = Spring(points[0], points[4], 1, outer_stiffness);
    springs[springCount++] = Spring(points[1], points[5], 1, outer_stiffness);
    springs[springCount++] = Spring(points[2], points[6], 1, outer_stiffness);
    springs[springCount++] = Spring(points[3], points[7], 1, outer_stiffness);
    // top layer
    springs[springCount++] = Spring(points[4], points[5], 1, outer_stiffness);
    springs[springCount++] = Spring(points[4], points[6], 1, outer_stiffness);
    springs[springCount++] = Spring(points[7], points[5], 1, outer_stiffness);
    springs[springCount++] = Spring(points[7], points[6], 1, outer_stiffness);
    // reinforcement
    springs[springCount++] = Spring(points[3], points[5], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[1], points[7], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[2], points[7], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[3], points[6], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[2], points[4], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[0], points[6], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[0], points[5], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[1], points[4], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[4], points[7], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[5], points[6], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[2], points[1], 1.414, outer_stiffness);
    springs[springCount++] = Spring(points[0], points[3], 1.414, outer_stiffness);
    // middle point
    float inner_stiffness = 400;
    springs[springCount++] = Spring(points[8], points[0], 0.866, inner_stiffness);
    springs[springCount++] = Spring(points[8], points[1], 0.866, inner_stiffness);
    springs[springCount++] = Spring(points[8], points[2], 0.866, inner_stiffness);
    springs[springCount++] = Spring(points[8], points[3], 0.866, inner_stiffness);
    springs[springCount++] = Spring(points[8], points[4], 0.866, inner_stiffness);
    springs[springCount++] = Spring(points[8], points[5], 0.866, inner_stiffness);
    springs[springCount++] = Spring(points[8], points[6], 0.866, inner_stiffness);
    springs[springCount++] = Spring(points[8], points[7], 0.866, inner_stiffness);

    // chain
    springs[springCount++] = Spring(points[9], points[8], 0.1, 1000);

    for (; springCount < INITIAL_SPRINGS;)
    {
        addSpring();
    }
}

void SceneComplex::simulateStep()
{
    switch (integrator)
    {
    case Integrator::Euler:
        for (int i = 0; i < pointCount; i++)
        {
            points[i].force = glm::vec3(0, 0, -9.81f * points[i].mass);
        }
        for (int i = 0; i < springCount; i++)
        {
            springs[i].computeElasticForces(dt, doDamping);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].integrateEuler(dt);
        }
        break;
    case Integrator::Leapfrog:
        for (int i = 0; i < pointCount; i++)
        {
            points[i].force = glm::vec3(0, 0, -9.81f * points[i].mass);
        }
        for (int i = 0; i < springCount; i++)
        {
            springs[i].computeElasticForces(dt, doDamping);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].integrateLeapfrog(dt);
        }
        break;
    case Integrator::Midpoint:
        for (int i = 0; i < pointCount; i++)
        {
            points[i].force = glm::vec3(0, 0, -9.81f * points[i].mass);
        }
        for (int i = 0; i < springCount; i++)
        {
            springs[i].computeElasticForces(dt, doDamping);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].integrateMidpoint1(dt);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].force = glm::vec3(0, 0, -9.81f * points[i].mass);
        }
        for (int i = 0; i < springCount; i++)
        {
            springs[i].computeElasticForces(dt, doDamping);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].integrateMidpoint2(dt);
        }
        break;
    case Integrator::Frogpoint:
        for (int i = 0; i < pointCount; i++)
        {
            points[i].force = glm::vec3(0, 0, -9.81f * points[i].mass);
        }
        for (int i = 0; i < springCount; i++)
        {
            springs[i].computeElasticForces(dt, doDamping);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].integrateFrogpoint1(dt);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].force = glm::vec3(0, 0, -9.81f * points[i].mass);
        }
        for (int i = 0; i < springCount; i++)
        {
            springs[i].computeElasticForces(dt, doDamping);
        }
        for (int i = 0; i < pointCount; i++)
        {
            points[i].integrateFrogpoint2(dt);
        }
        break;
    }

    for (int i = 0; i < pointCount; i++)
    {
        glm::vec3 &position = points[i].position;
        if (clamp(position.x, -2.5f, 2.5f))
        {
            points[i].velocity.x *= -0.4;
        }
        if (clamp(position.y, -2.5f, 2.5f))
        {
            points[i].velocity.y *= -0.4;
        }
        if (clamp(position.z, -2.5f, 2.5f))
        {
            points[i].velocity.z *= -0.4; // collision damping
        }
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
    {
        auto drag = ImGui::GetMouseDragDelta(1);
        if (!(drag.x == 0 && drag.y == 0))
        {
            auto dx = drag.x * right;
            auto dy = -drag.y * up;
            for (int i = 0; i < pointCount; i++)
            {
                points[i].velocity += (dx + dy) * 0.01f;
            }
        }
    }
}

void SceneComplex::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    for (int i = 0; i < springCount; i++)
    {
        renderer.drawLine(springs[i].point1->position, springs[i].point2->position, glm::vec4(0.5, 0.5, 0.5, 1));
    }
    for (int i = 0; i < pointCount; i++)
    {
        renderer.drawSphere(points[i].position, 0.1f, glm::vec4(points[i].mass / 100.0, 1, 1, 1));
    }
    cameraMatrix = renderer.camera.viewMatrix;
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);
}

void SceneComplex::onGUI()
{
    ImGui::SliderFloat("Dt", &dt, 0, 0.02f);
    ImGui::Checkbox("Do damping", &doDamping);
    int current = static_cast<int>(integrator);
    if (ImGui::Combo("Integrator", &current, integratorNames, IM_ARRAYSIZE(integratorNames)))
    {
        integrator = static_cast<Integrator>(current);
    }
    auto addPointButton = ImGui::Button("Add Random Point");
    if (addPointButton)
    {
        addPoint();
    }
    auto addSpringButton = ImGui::Button("Add Random Spring");
    if (addSpringButton)
    {
        addSpring();
    }
    auto resetVelocities = ImGui::Button("Set all velocities to 0");
    if (resetVelocities)
    {
        for (int i = 0; i < pointCount; i++)
        {
            points[i].velocity = glm::vec3(0);
        }
    }
    auto resetPositions = ImGui::Button("Set all positions to 0");
    if (resetPositions)
    {
        for (int i = 0; i < pointCount; i++)
        {
            if (!points[i].fixed)
            {
                points[i].position = glm::vec3(0);
            }
        }
    }
    auto fixPoint = ImGui::Button("Fix random point");
    if (fixPoint)
    {
        bool canFix = false;
        for (int i = 0; i < pointCount; i++)
        {
            if (!points[i].fixed)
            {
                canFix = true;
            }
        }
        while (canFix)
        {
            int i = static_cast<int>(dis(gen) * pointCount);
            if (!points[i].fixed)
            {
                points[i].fixed = true;
                break;
            }
        }
    }
    auto unfixPoint = ImGui::Button("Unfix random point");
    if (unfixPoint)
    {
        bool canUnfix = false;
        for (int i = 0; i < pointCount; i++)
        {
            if (points[i].fixed)
            {
                canUnfix = true;
            }
        }
        while (canUnfix)
        {
            int i = static_cast<int>(dis(gen) * pointCount);
            if (points[i].fixed)
            {
                points[i].fixed = false;
                break;
            }
        }
    }
}
