#include "MidpointSimulation.h"

void MidpointSimulation::simulateStep()
{

}

void MidpointSimulation::onDraw(Renderer &renderer)
{

}

void MidpointSimulation::init()
{
    MidpointSimulation::x0 = glm::vec3(0.0f, 0.0f, 0.0f);
    MidpointSimulation::v0 = glm::vec3(-1.0f, 0.0f, 0.0f);

    MidpointSimulation::x1 = glm::vec3(0.0f, 2.0f, 0.0f);
    MidpointSimulation::v1 = glm::vec3(1.0f, 0.0f, 0.0f);
}

// Hilfsmethoden
glm::vec3 MidpointSimulation::calculateSpringForce( float stiffness, float springCurrentLength, float springRestLength,  
                                        glm::vec3 &x0, glm::vec3 &x1)
{
    
}