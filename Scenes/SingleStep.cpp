#include "SingleStep.h"
#include <iostream>

void SingleStep::init() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "SINGLE STEP RIGID BODY TEST" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // setup rigid body as specified:
    // - center of mass at origin
    // - extent: [1, 0.6, 0.5]
    // - mass: 2
    // - orientation: rotated 90 degrees around z axis
    // - initial velocity: zero
    // - initial angular velocity: zero
    
    body.position = glm::vec3(0, 0, 0);
    body.extent = glm::vec3(1.0f, 0.6f, 0.5f);
    body.mass = 2.0f;
    body.orientation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1));
    body.velocity = glm::vec3(0);
    body.angularVelocity = glm::vec3(0);
    
    body.print("initial state");
    
    // external force and application point
    glm::vec3 force(1.0f, 1.0f, 0.0f);
    glm::vec3 forcePos(0.3f, 0.5f, 0.25f);
    float dt = 2.0f;
    
    std::cout << "\nforce: [" << force.x << ", " << force.y << ", " << force.z << "]" << std::endl;
    std::cout << "force position (world): [" << forcePos.x << ", " << forcePos.y << ", " << forcePos.z << "]" << std::endl;
    std::cout << "dt: " << dt << std::endl;
    
    // compute torque
    glm::vec3 torque = body.computeTorque(force, forcePos);
    std::cout << "\ntorque = r x F: [" << torque.x << ", " << torque.y << ", " << torque.z << "]" << std::endl;
    
    // print inertia tensor for debugging
    glm::mat3 I = body.getBodyInertiaTensor();
    std::cout << "\nbody inertia tensor (diagonal):" << std::endl;
    std::cout << "  Ixx = " << I[0][0] << std::endl;
    std::cout << "  Iyy = " << I[1][1] << std::endl;
    std::cout << "  Izz = " << I[2][2] << std::endl;
    
    glm::mat3 I_world = body.getWorldInertiaTensor();
    std::cout << "\nworld inertia tensor:" << std::endl;
    std::cout << "  [" << I_world[0][0] << ", " << I_world[1][0] << ", " << I_world[2][0] << "]" << std::endl;
    std::cout << "  [" << I_world[0][1] << ", " << I_world[1][1] << ", " << I_world[2][1] << "]" << std::endl;
    std::cout << "  [" << I_world[0][2] << ", " << I_world[1][2] << ", " << I_world[2][2] << "]" << std::endl;
    
    // integrate one step
    body.integrate(dt, force, torque);
    
    std::cout << "\n";
    body.print("after one time step (dt=2)");
    
    // compute world space position and velocity of body-local point [-0.3, -0.5, -0.25]
    glm::vec3 queryPoint(-0.3f, -0.5f, -0.25f);
    glm::vec3 worldPos = body.getWorldPosition(queryPoint);
    glm::vec3 pointVel = body.getPointVelocity(queryPoint);
    
    std::cout << "\nquery point (body local): [" << queryPoint.x << ", " << queryPoint.y << ", " << queryPoint.z << "]" << std::endl;
    std::cout << "world position of query point: [" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << "]" << std::endl;
    std::cout << "world velocity of query point: [" << pointVel.x << ", " << pointVel.y << ", " << pointVel.z << "]" << std::endl;
    
    std::cout << "\n========================================" << std::endl;
}

void SingleStep::onDraw(Renderer& renderer) {
    // draw world bounds
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    
    // draw the rigid body
    renderer.drawCube(body.position, body.orientation, body.extent, glm::vec4(0.8f, 0.2f, 0.2f, 1.0f));
}

