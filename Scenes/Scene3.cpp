#include "Scene3.h"
#include <imgui.h>
#include <iostream>

glm::vec3 Scene3::computeSpringForce(const MassPoint& p0, const MassPoint& p1, float k, float L)
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

void Scene3::midpointStep(float timestep)
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
    
    mid0.position = initialState[0].position + 0.5f * timestep * initialState[0].velocity;
    mid0.velocity = initialState[0].velocity + 0.5f * timestep * accel0_initial;
    
    mid1.position = initialState[1].position + 0.5f * timestep * initialState[1].velocity;
    mid1.velocity = initialState[1].velocity + 0.5f * timestep * accel1_initial;
    
    // recalc forces at midpoint - this makes it way more accurate
    glm::vec3 force0_mid = computeSpringForce(mid0, mid1, springStiffness, restLength);
    glm::vec3 force1_mid = -force0_mid;
    
    glm::vec3 accel0_mid = force0_mid / mid0.mass;
    glm::vec3 accel1_mid = force1_mid / mid1.mass;
    
    // now do full step with the midpoint velocity and accel
    points[0].position = initialState[0].position + timestep * mid0.velocity;
    points[0].velocity = initialState[0].velocity + timestep * accel0_mid;
    
    points[1].position = initialState[1].position + timestep * mid1.velocity;
    points[1].velocity = initialState[1].velocity + timestep * accel1_mid;
}

void Scene3::init()
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

void Scene3::onDraw(Renderer &renderer)
{
    // draw the wire cube boundary
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    
    // draw the two mass points as spheres
    renderer.drawSphere(points[0].position, 0.1f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));  // red
    renderer.drawSphere(points[1].position, 0.1f, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));  // blue
    
    // draw the spring as a line between them
    renderer.drawLine(points[0].position, points[1].position, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));  // green
}

void Scene3::simulateStep()
{
    // do one midpoint step with current timestep
    midpointStep(dt);
}

void Scene3::onGUI()
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

