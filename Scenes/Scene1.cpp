#include "Scene1.h"
#include <glm/gtx/string_cast.hpp>
#include <iostream>

glm::vec3 Scene1::computeSpringForce(const MassPoint& p0, const MassPoint& p1, float k, float L)
{
    glm::vec3 direction = p1.position - p0.position;
    float currentLength = glm::length(direction);
    
    if (currentLength < 1e-6f) {
        return glm::vec3(0.0f);
    }
    
    glm::vec3 normalizedDirection = direction / currentLength;
    float extension = currentLength - L;
    
    return k * extension * normalizedDirection;
}

void Scene1::eulerStep(std::vector<MassPoint>& points, float dt)
{
    // calc spring force on p0, p1 gets opposite force (newtons 3rd law)
    glm::vec3 force0 = computeSpringForce(points[0], points[1], springStiffness, restLength);
    glm::vec3 force1 = -force0;
    
    // f=ma so a=f/m
    glm::vec3 accel0 = force0 / points[0].mass;
    glm::vec3 accel1 = force1 / points[1].mass;
    
    // basic euler step just use current velocity and acceleration
    points[0].position += dt * points[0].velocity;
    points[0].velocity += dt * accel0;
    
    points[1].position += dt * points[1].velocity;
    points[1].velocity += dt * accel1;
}

void Scene1::midpointStep(std::vector<MassPoint>& points, float dt)
{
    // save initial state cause we need it later
    std::vector<MassPoint> initialState = points;
    
    // get forces and accel at current time
    glm::vec3 force0_initial = computeSpringForce(points[0], points[1], springStiffness, restLength);
    glm::vec3 force1_initial = -force0_initial;
    
    glm::vec3 accel0_initial = force0_initial / points[0].mass;
    glm::vec3 accel1_initial = force1_initial / points[1].mass;
    
    // do half euler step to guess where well be at t+dt/2
    MassPoint mid0 = initialState[0];
    MassPoint mid1 = initialState[1];
    
    mid0.position = initialState[0].position + 0.5f * dt * initialState[0].velocity;
    mid0.velocity = initialState[0].velocity + 0.5f * dt * accel0_initial;
    
    mid1.position = initialState[1].position + 0.5f * dt * initialState[1].velocity;
    mid1.velocity = initialState[1].velocity + 0.5f * dt * accel1_initial;
    
    // recalc forces at midpoint - this makes it way more accurate
    glm::vec3 force0_mid = computeSpringForce(mid0, mid1, springStiffness, restLength);
    glm::vec3 force1_mid = -force0_mid;
    
    glm::vec3 accel0_mid = force0_mid / mid0.mass;
    glm::vec3 accel1_mid = force1_mid / mid1.mass;
    
    // now do full step with the midpoint velocity and accel
    points[0].position = initialState[0].position + dt * mid0.velocity;
    points[0].velocity = initialState[0].velocity + dt * accel0_mid;
    
    points[1].position = initialState[1].position + dt * mid1.velocity;
    points[1].velocity = initialState[1].velocity + dt * accel1_mid;
}

void Scene1::init()
{
    // setup 2 mass points connected by spring
    std::vector<MassPoint> points(2);
    
    // p0 at origin moving left
    points[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
    points[0].velocity = glm::vec3(-1.0f, 0.0f, 0.0f);
    points[0].mass = 10.0f;
    
    // p1 at y=2 moving right
    points[1].position = glm::vec3(0.0f, 2.0f, 0.0f);
    points[1].velocity = glm::vec3(1.0f, 0.0f, 0.0f);
    points[1].mass = 10.0f;
    
    float dt = 0.1f;
    
    // print initial state
    std::cout << "Initial Values:" << std::endl;
    std::cout << "Masspoint 0" << std::endl;
    std::cout << "  position: " << glm::to_string(points[0].position) << std::endl;
    std::cout << "  velocity: " << glm::to_string(points[0].velocity) << std::endl;
    std::cout << "Masspoint 1" << std::endl;
    std::cout << "  position: " << glm::to_string(points[1].position) << std::endl;
    std::cout << "  velocity: " << glm::to_string(points[1].velocity) << std::endl;
    std::cout << std::endl;
    
    // test midpoint (better accuracy)
    std::vector<MassPoint> pointsMidpoint = points;
    midpointStep(pointsMidpoint, dt);
    
    std::cout << "After " << dt << "s using Midpoint Method:" << std::endl;
    std::cout << "Masspoint 0" << std::endl;
    std::cout << "  position: " << glm::to_string(pointsMidpoint[0].position) << std::endl;
    std::cout << "  velocity: " << glm::to_string(pointsMidpoint[0].velocity) << std::endl;
    std::cout << "Masspoint 1" << std::endl;
    std::cout << "  position: " << glm::to_string(pointsMidpoint[1].position) << std::endl;
    std::cout << "  velocity: " << glm::to_string(pointsMidpoint[1].velocity) << std::endl;
    std::cout << std::endl;
    
    // test euler (simpler but less accurate)
    std::vector<MassPoint> pointsEuler = points;
    eulerStep(pointsEuler, dt);
    
    std::cout << "After " << dt << "s using Euler Method:" << std::endl;
    std::cout << "Masspoint 0" << std::endl;
    std::cout << "  position: " << glm::to_string(pointsEuler[0].position) << std::endl;
    std::cout << "  velocity: " << glm::to_string(pointsEuler[0].velocity) << std::endl;
    std::cout << "Masspoint 1" << std::endl;
    std::cout << "  position: " << glm::to_string(pointsEuler[1].position) << std::endl;
    std::cout << "  velocity: " << glm::to_string(pointsEuler[1].velocity) << std::endl;
    std::cout << std::endl;
}
