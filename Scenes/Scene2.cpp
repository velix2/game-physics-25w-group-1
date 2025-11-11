#include "Scene2.h"
#include <imgui.h>
#include <iostream>

glm::vec3 Scene2::computeSpringForce(const MassPoint& p0, const MassPoint& p1, float k, float L)
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

void Scene2::eulerStep(float timestep)
{
    // calc spring force on p0, p1 gets opposite
    glm::vec3 force0 = computeSpringForce(points[0], points[1], springStiffness, restLength);
    glm::vec3 force1 = -force0;
    
    // f=ma so a=f/m
    glm::vec3 accel0 = force0 / points[0].mass;
    glm::vec3 accel1 = force1 / points[1].mass;
    
    // euler integration
    points[0].position += timestep * points[0].velocity;
    points[0].velocity += timestep * accel0;
    
    points[1].position += timestep * points[1].velocity;
    points[1].velocity += timestep * accel1;
}

void Scene2::init()
{
    // setup 2 mass points with spring between them
    points.resize(2);
    
    // p0 at origin moving left
    points[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
    points[0].velocity = glm::vec3(-1.0f, 0.0f, 0.0f);
    points[0].mass = 10.0f;
    
    // p1 at y=2 moving right
    points[1].position = glm::vec3(0.0f, 2.0f, 0.0f);
    points[1].velocity = glm::vec3(1.0f, 0.0f, 0.0f);
    points[1].mass = 10.0f;
}

void Scene2::onDraw(Renderer &renderer)
{
    // draw the wire cube boundary
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    
    // draw the two mass points as spheres
    renderer.drawSphere(points[0].position, 0.1f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));  // red
    renderer.drawSphere(points[1].position, 0.1f, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));  // blue
    
    // draw the spring as a line between them
    renderer.drawLine(points[0].position, points[1].position, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));  // green
}

void Scene2::simulateStep()
{
    // do one euler step with current timestep
    eulerStep(dt);
}

void Scene2::onGUI()
{
    // slider to adjust timestep size
    ImGui::SliderFloat("timestep dt", &dt, 0.0001f, 0.02f);
    
    // show current positions and velocities
    ImGui::Text("mass point 0:");
    ImGui::Text("  pos: (%.3f, %.3f, %.3f)", points[0].position.x, points[0].position.y, points[0].position.z);
    ImGui::Text("  vel: (%.3f, %.3f, %.3f)", points[0].velocity.x, points[0].velocity.y, points[0].velocity.z);
    
    ImGui::Text("mass point 1:");
    ImGui::Text("  pos: (%.3f, %.3f, %.3f)", points[1].position.x, points[1].position.y, points[1].position.z);
    ImGui::Text("  vel: (%.3f, %.3f, %.3f)", points[1].velocity.x, points[1].velocity.y, points[1].velocity.z);
    
    // show spring length
    float currentLength = glm::length(points[1].position - points[0].position);
    ImGui::Text("spring length: %.3f (rest: %.3f)", currentLength, restLength);
}

