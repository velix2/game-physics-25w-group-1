#include "Common.h"

float dot(glm::vec3 v1, glm::vec3 v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

void Spring::computeElasticForces(float dt, bool doDamping)
{
    glm::vec3 lVec = (point1->position - point2->position);
    double l = sqrt(lVec.x * lVec.x + lVec.y * lVec.y + lVec.z * lVec.z);
    // If two points have the same position, the computed force would be infinite.
    // In this case, the assumed force is based on the velocities of the points, as if the points collided.
    if (l == 0)
    {
        glm::vec3 vrel = point2->velocity - point1->velocity;
        float normFactor = sqrt(dot(vrel, vrel));
        if (normFactor == 0)
        {
            vrel = glm::vec3(0, 0, 1);
            normFactor = 1;
        }
        glm::vec3 n = vrel / normFactor;
        glm::vec3 f = 1000000 * 0.001f * n;
        point1->force += f;
        point2->force += -f;
        return;
    }
    float scaleFactor = (-stiffness * (l - restLength)) / l;
    glm::vec3 force = lVec * scaleFactor;

    point1->force += force;
    if (doDamping)
    {
        point1->force -= point1->damping * point1->velocity;
    }
    point2->force += -force;
    if (doDamping)
    {
        point2->force -= point2->damping * point2->velocity;
    }
}

void PointMass::integrateEuler(float dt)
{
    if (fixed)
    {
        return;
    }
    position += dt * velocity;
    velocity += dt * (force / mass);
}

void PointMass::integrateMidpoint1(float dt)
{
    if (fixed)
    {
        return;
    }
    position2 = position;
    velocity2 = velocity;

    position += (dt / 2) * velocity;
    velocity += (dt / 2) * (force / mass);
}

void PointMass::integrateMidpoint2(float dt)
{
    if (fixed)
    {
        return;
    }
    position = position2 + dt * velocity;
    velocity = velocity2 + dt * (force / mass);
}

void PointMass::integrateLeapfrog(float dt)
{
    if (fixed)
    {
        return;
    }
    velocity += dt * (force / mass);
    position += dt * velocity;
}

void PointMass::integrateFrogpoint1(float dt)
{
    if (fixed)
    {
        return;
    }
    position2 = position;
    velocity2 = velocity;

    velocity += (dt / 2) * (force / mass);
    position += (dt / 2) * velocity;
}

void PointMass::integrateFrogpoint2(float dt)
{
    if (fixed)
    {
        return;
    }
    velocity = velocity2 + dt * (force / mass);
    position = position2 + dt * velocity;
}

void PointMass::printInfo()
{
    printf("position: vec3(%f, %f, %f)\n", position.x, position.y, position.z);
    printf("velocity: vec3(%f, %f, %f)\n", velocity.x, velocity.y, velocity.z);
    printf("mass: %f\n", mass);
}