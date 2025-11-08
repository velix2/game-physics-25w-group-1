#pragma once
#include "Renderer.h"

struct PointMass
{
    glm::vec3 position;
    glm::vec3 position2;
    glm::vec3 velocity;
    glm::vec3 velocity2;
    glm::vec3 force;
    glm::vec4 color;
    float mass, damping;
    bool fixed;

    PointMass(glm::vec3 pos, glm::vec3 vel, float mass, bool fixed = false) : position(pos), position2(glm::vec3(0)),
                                                                              velocity(vel), velocity2(glm::vec3(0)),
                                                                              force(glm::vec3(0)), color(glm::vec4(1)), mass(mass), damping(0),
                                                                              fixed(fixed) {};

    PointMass() {};
    virtual void integrateEuler(float dt);
    virtual void integrateMidpoint1(float dt);
    virtual void integrateMidpoint2(float dt);

    virtual void printInfo();
};

struct Spring
{
    PointMass *point1, *point2;
    float restLength, stiffness;

    Spring(PointMass &point1, PointMass &point2, float restLength, float stiffness) : point1(&point1), point2(&point2),
                                                                                      restLength(restLength), stiffness(stiffness) {};

    Spring() {};

    virtual void computeElasticForces(float dt);
};