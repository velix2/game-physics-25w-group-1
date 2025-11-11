#include "SingleStep.h"
#include <iostream>


void SingleStep::printMassPoint(const std::string &name, const glm::vec3 &x, const glm::vec3 &v)
{
    std::cout << name << std::endl;
    std::cout << "Position: vec3(" << x.x << ", " << x.y << ", " << x.z << ")" << std::endl;
    std::cout << "Velocity: vec3(" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    std::cout << std::endl;
}


glm::vec3 SingleStep::calculateSpringForce(  float stiffness, float springCurrentLength, float springRestLength,  
                                        glm::vec3 &x0, glm::vec3 &x1)
{
    glm::vec3 springForce = -stiffness * (springCurrentLength - springRestLength) * ((x0 - x1) / springCurrentLength);
    return springForce;
}

void SingleStep::integrateStepEuler(glm::vec3 &x0, glm::vec3 &v0, float m0, 
                                    glm::vec3 &x1, glm::vec3 &v1, float m1,
                                    float springRestLength, float stiffness, float deltaT)
{
    // Force
    float springCurrentLength = glm::length(x0 - x1);
    glm::vec3 springForce0_1 = calculateSpringForce(stiffness, springCurrentLength, springRestLength, x0, x1);
    glm::vec3 springForce1_0 = -springForce0_1;

    // acceleration
    glm::vec3 a0 = springForce0_1 / m0;
    glm::vec3 a1 = springForce1_0 / m1;

    //For x0
    x0 = x0 + deltaT * v0;
    v0 = v0 + deltaT * a0;

    //For x1
    x1 = x1 + deltaT * v1;
    v1 = v1 + deltaT * a1;
}

void SingleStep::integrateStepMidpoint( glm::vec3 &x0, glm::vec3 &v0, float m0, 
                                        glm::vec3 &x1, glm::vec3 &v1, float m1,
                                        float springRestLength, float stiffness, float deltaT)
{
    // Force in 0
    float springCurrentLength = glm::length(x0 - x1);
    glm::vec3 springForce0_1 = calculateSpringForce(stiffness, springCurrentLength, springRestLength, x0, x1);
    glm::vec3 springForce1_0 = - springForce0_1;

    glm::vec3 a0 = springForce0_1 / m0;
    glm::vec3 a1 = springForce1_0 /m1;

    // predictions
    glm::vec3 x0_mid = x0 + (deltaT / 2.0f) * v0;
    glm::vec3 x1_mid = x1 + (deltaT /2.0f) * v1;

    // Force in 1/2 delta T
    float springCurrentLength_mid = glm::length(x0_mid - x1_mid);
    glm::vec3 springForce0_1_mid = calculateSpringForce(stiffness, springCurrentLength_mid, springRestLength, x0_mid, x1_mid);
    glm::vec3 springForce1_0_mid = - springForce0_1_mid;

    // predictions
    glm::vec3 a0_mid = springForce0_1_mid / m0;
    glm::vec3 a1_mid = springForce1_0_mid / m1;

    glm::vec3 v0_mid = v0 + (deltaT / 2.0f) * a0;
    glm::vec3 v1_mid = v1 + (deltaT / 2.0f) * a1;

    // Final
    x0 = x0 + deltaT * v0_mid;
    x1 = x1 + deltaT * v1_mid;

    v0 = v0 + deltaT * a0_mid;
    v1 = v1 + deltaT * a1_mid;
}

void SingleStep::init()
{
    //Initialize values
    glm::vec3 x0(0.0f, 0.0f, 0.0f);
    glm::vec3 v0(-1.0f, 0.0f, 0.0f);

    glm::vec3 x1(0.0f, 2.0f, 0.0f);
    glm::vec3 v1(1.0f, 0.0f, 0.0f);

    float mass = 10.0f;
    float springRestLength = 1.0f;
    float stiffness = 40.0f;
    float deltaT = 0.1f;

    std::cout << "Initial Values:" << std::endl;
    
    std::cout << "" << std::endl;

    printMassPoint("Masspoint 0", x0, v0);
    printMassPoint("Masspoint 1", x1, v1);

    std::cout << "" << std::endl;

    // Midpoint method
    glm::vec3 x0_m = x0, v0_m = v0, x1_m = x1, v1_m = v1; // speichere werte zwischen
    std::cout << "After " << deltaT << "s using Midpoint Method:" << std::endl;
    std::cout << "" << std::endl;

    integrateStepMidpoint(x0_m, v0_m, mass, x1_m, v1_m, mass, springRestLength, stiffness, deltaT);

    printMassPoint("Masspoint 0", x0_m, v0_m);
    printMassPoint("Masspoint 1", x1_m, v1_m);

    // Euler method
    glm::vec3 x0_e = x0, v0_e = v0, x1_e = x1, v1_e = v1; // speichere werte zwischen
    std::cout << "After " << deltaT << "s using Euler Method:" << std::endl;
    std::cout << "" << std::endl;

    integrateStepEuler(x0_e, v0_e, mass, x1_e, v1_e, mass, springRestLength, stiffness, deltaT);

    printMassPoint("Masspoint 0", x0_e, v0_e);
    printMassPoint("Masspoint 1", x1_e, v1_e);
}