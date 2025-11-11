#include "Scene4.h"
#include <imgui.h>
#include <iostream>

glm::vec3 Scene4::computeSpringForce(const MassPoint& p0, const MassPoint& p1, float k, float L)
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

void Scene4::eulerStep(float timestep)
{
    // calc all forces first
    std::vector<glm::vec3> forces(points.size(), glm::vec3(0.0f));
    
    // spring forces
    for (const auto& spring : springs) {
        glm::vec3 force = computeSpringForce(points[spring.point0], points[spring.point1], 
                                             spring.stiffness, spring.restLength);
        forces[spring.point0] += force;
        forces[spring.point1] -= force;
    }
    
    // integrate all points
    for (size_t i = 0; i < points.size(); i++) {
        glm::vec3 accel = forces[i] / points[i].mass;
        points[i].position += timestep * points[i].velocity;
        points[i].velocity += timestep * accel;
    }
}

void Scene4::midpointStep(float timestep)
{
    // save initial state
    std::vector<MassPoint> initialState = points;
    
    // calc forces at current state
    std::vector<glm::vec3> forces_initial(points.size(), glm::vec3(0.0f));
    for (const auto& spring : springs) {
        glm::vec3 force = computeSpringForce(points[spring.point0], points[spring.point1],
                                             spring.stiffness, spring.restLength);
        forces_initial[spring.point0] += force;
        forces_initial[spring.point1] -= force;
    }
    
    // do half step to get midpoint state
    for (size_t i = 0; i < points.size(); i++) {
        glm::vec3 accel = forces_initial[i] / points[i].mass;
        points[i].position = initialState[i].position + 0.5f * timestep * initialState[i].velocity;
        points[i].velocity = initialState[i].velocity + 0.5f * timestep * accel;
    }
    
    // recalc forces at midpoint
    std::vector<glm::vec3> forces_mid(points.size(), glm::vec3(0.0f));
    for (const auto& spring : springs) {
        glm::vec3 force = computeSpringForce(points[spring.point0], points[spring.point1],
                                             spring.stiffness, spring.restLength);
        forces_mid[spring.point0] += force;
        forces_mid[spring.point1] -= force;
    }
    
    // full step using midpoint derivatives
    for (size_t i = 0; i < points.size(); i++) {
        glm::vec3 accel_mid = forces_mid[i] / points[i].mass;
        points[i].position = initialState[i].position + timestep * points[i].velocity;
        points[i].velocity = initialState[i].velocity + timestep * accel_mid;
    }
}

void Scene4::leapfrogStep(float timestep)
{
    // calc all forces
    std::vector<glm::vec3> forces(points.size(), glm::vec3(0.0f));
    for (const auto& spring : springs) {
        glm::vec3 force = computeSpringForce(points[spring.point0], points[spring.point1],
                                             spring.stiffness, spring.restLength);
        forces[spring.point0] += force;
        forces[spring.point1] -= force;
    }
    
    // leapfrog integration - velocity at half steps
    for (size_t i = 0; i < points.size(); i++) {
        glm::vec3 accel = forces[i] / points[i].mass;
        points[i].velocity += timestep * accel;  // v(t+dt/2) using current positions
        points[i].position += timestep * points[i].velocity;  // x(t+dt) using new velocity
    }
}

void Scene4::handleCollisions()
{
    for (auto& point : points) {
        // ground collision
        if (point.position.z < groundLevel) {
            point.position.z = groundLevel;
            point.velocity.z = -point.velocity.z * damping;  // bounce with damping
        }
        
        // wall collisions (x and y bounds)
        if (point.position.x < -wallBounds) {
            point.position.x = -wallBounds;
            point.velocity.x = -point.velocity.x * damping;
        }
        if (point.position.x > wallBounds) {
            point.position.x = wallBounds;
            point.velocity.x = -point.velocity.x * damping;
        }
        if (point.position.y < -wallBounds) {
            point.position.y = -wallBounds;
            point.velocity.y = -point.velocity.y * damping;
        }
        if (point.position.y > wallBounds) {
            point.position.y = wallBounds;
            point.velocity.y = -point.velocity.y * damping;
        }
    }
}

void Scene4::applyGravity(float timestep)
{
    if (!gravityEnabled) return;
    
    for (auto& point : points) {
        point.velocity += gravity * timestep;
    }
}

void Scene4::applyGlobalParams()
{
    // update all mass points
    for (auto& point : points) {
        point.mass = globalMass;
    }
    
    // update all springs
    for (auto& spring : springs) {
        spring.stiffness = globalStiffness;
    }
}

void Scene4::applyWindForce()
{
    if (!applyWind) return;
    
    for (auto& point : points) {
        point.velocity += windForce * dt;
    }
}

void Scene4::applyVelocityDamping()
{
    if (globalDamping <= 0.0f) return;
    
    for (auto& point : points) {
        point.velocity *= (1.0f - globalDamping * dt);
    }
}

void Scene4::init()
{
    // create a hanging cloth-like structure (4x3 grid of points)
    int rows = 3;
    int cols = 4;
    float spacing = 0.5f;
    
    // create mass points in a grid
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            MassPoint p;
            p.position = glm::vec3(c * spacing - (cols-1) * spacing * 0.5f, 
                                   r * spacing - (rows-1) * spacing * 0.5f, 
                                   1.0f);
            p.velocity = glm::vec3(0.0f);
            p.mass = 1.0f;
            points.push_back(p);
        }
    }
    
    // create springs - horizontal connections
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols - 1; c++) {
            Spring s;
            s.point0 = r * cols + c;
            s.point1 = r * cols + c + 1;
            s.stiffness = 100.0f;
            s.restLength = spacing;
            springs.push_back(s);
        }
    }
    
    // vertical connections
    for (int r = 0; r < rows - 1; r++) {
        for (int c = 0; c < cols; c++) {
            Spring s;
            s.point0 = r * cols + c;
            s.point1 = (r + 1) * cols + c;
            s.stiffness = 100.0f;
            s.restLength = spacing;
            springs.push_back(s);
        }
    }
    
    // diagonal connections for stability
    for (int r = 0; r < rows - 1; r++) {
        for (int c = 0; c < cols - 1; c++) {
            // diagonal 1
            Spring s1;
            s1.point0 = r * cols + c;
            s1.point1 = (r + 1) * cols + c + 1;
            s1.stiffness = 100.0f;
            s1.restLength = spacing * sqrtf(2.0f);
            springs.push_back(s1);
            
            // diagonal 2
            Spring s2;
            s2.point0 = r * cols + c + 1;
            s2.point1 = (r + 1) * cols + c;
            s2.stiffness = 100.0f;
            s2.restLength = spacing * sqrtf(2.0f);
            springs.push_back(s2);
        }
    }
    
    std::cout << "created " << points.size() << " points and " << springs.size() << " springs" << std::endl;
}

void Scene4::onDraw(Renderer &renderer)
{
    // draw boundary box
    renderer.drawWireCube(glm::vec3(0, 0, 0), glm::vec3(wallBounds * 2.0f, wallBounds * 2.0f, 5.0f), 
                         glm::vec3(0.5f, 0.5f, 0.5f));
    
    // draw ground plane
    renderer.drawLine(glm::vec3(-wallBounds, -wallBounds, groundLevel), 
                     glm::vec3(wallBounds, -wallBounds, groundLevel), 
                     glm::vec4(0.3f, 0.3f, 0.3f, 1.0f));
    renderer.drawLine(glm::vec3(-wallBounds, wallBounds, groundLevel), 
                     glm::vec3(wallBounds, wallBounds, groundLevel), 
                     glm::vec4(0.3f, 0.3f, 0.3f, 1.0f));
    
    // draw springs
    for (const auto& spring : springs) {
        glm::vec4 color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);  // green
        
        // color based on extension
        float currentLen = glm::length(points[spring.point1].position - points[spring.point0].position);
        float strain = abs(currentLen - spring.restLength) / spring.restLength;
        if (strain > 0.1f) {
            color = glm::vec4(1.0f, 0.5f, 0.0f, 1.0f);  // orange if stretched
        }
        if (strain > 0.3f) {
            color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);  // red if very stretched
        }
        
        renderer.drawLine(points[spring.point0].position, points[spring.point1].position, color);
    }
    
    // draw mass points
    for (size_t i = 0; i < points.size(); i++) {
        glm::vec4 color = glm::vec4(0.2f, 0.5f, 1.0f, 1.0f);  // blue
        if ((int)i == selectedPoint) {
            color = glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);  // yellow if selected
        }
        renderer.drawSphere(points[i].position, 0.05f, color);
    }
}

void Scene4::simulateStep()
{
    if (isPaused) return;
    
    // apply global params if they changed
    applyGlobalParams();
    
    // apply forces before integration
    applyGravity(dt);
    applyWindForce();
    
    // do integration step based on selected method
    switch (integrationMethod) {
        case 0:
            eulerStep(dt);
            break;
        case 1:
            midpointStep(dt);
            break;
        case 2:
            leapfrogStep(dt);
            break;
    }
    
    // apply damping and handle collisions after integration
    applyVelocityDamping();
    handleCollisions();
}

void Scene4::onGUI()
{
    // simulation control
    if (ImGui::Button(isPaused ? "resume" : "pause")) {
        isPaused = !isPaused;
    }
    ImGui::SameLine();
    if (ImGui::Button("reset")) {
        points.clear();
        springs.clear();
        init();
    }
    
    ImGui::Separator();
    ImGui::Text("integration settings");
    
    // integration method selector
    const char* methods[] = { "euler", "midpoint", "leapfrog" };
    ImGui::Combo("method", &integrationMethod, methods, 3);
    
    // timestep slider
    ImGui::SliderFloat("timestep dt", &dt, 0.0001f, 0.02f, "%.4f", ImGuiSliderFlags_Logarithmic);
    
    ImGui::Separator();
    ImGui::Text("physics parameters");
    
    // mass and stiffness
    if (ImGui::SliderFloat("global mass", &globalMass, 0.1f, 10.0f)) {
        applyGlobalParams();
    }
    if (ImGui::SliderFloat("spring stiffness", &globalStiffness, 10.0f, 500.0f)) {
        applyGlobalParams();
    }
    ImGui::SliderFloat("velocity damping", &globalDamping, 0.0f, 2.0f);
    
    ImGui::Separator();
    ImGui::Text("forces");
    
    // gravity toggle and settings
    ImGui::Checkbox("gravity", &gravityEnabled);
    if (gravityEnabled) {
        ImGui::SliderFloat("gravity strength", &gravity.z, -20.0f, 0.0f);
    }
    
    // wind force
    ImGui::Checkbox("wind", &applyWind);
    if (applyWind) {
        ImGui::SliderFloat("wind x", &windForce.x, -5.0f, 5.0f);
        ImGui::SliderFloat("wind y", &windForce.y, -5.0f, 5.0f);
        ImGui::SliderFloat("wind z", &windForce.z, -5.0f, 5.0f);
    }
    
    ImGui::Separator();
    ImGui::Text("collision settings");
    
    // collision settings
    ImGui::SliderFloat("ground level", &groundLevel, -3.0f, 0.0f);
    ImGui::SliderFloat("bounce damping", &damping, 0.0f, 1.0f);
    ImGui::SliderFloat("wall bounds", &wallBounds, 1.0f, 5.0f);
    
    ImGui::Separator();
    ImGui::Text("interaction");
    
    // point manipulation
    ImGui::SliderInt("selected point", &selectedPoint, -1, (int)points.size() - 1);
    if (selectedPoint >= 0 && selectedPoint < (int)points.size()) {
        ImGui::Text("point %d:", selectedPoint);
        ImGui::DragFloat3("position", &points[selectedPoint].position.x, 0.01f);
        ImGui::DragFloat3("velocity", &points[selectedPoint].velocity.x, 0.1f);
        if (ImGui::Button("freeze point")) {
            points[selectedPoint].velocity = glm::vec3(0.0f);
        }
        ImGui::SameLine();
        if (ImGui::Button("launch up")) {
            points[selectedPoint].velocity.z = 5.0f;
        }
    }
    
    // quick actions
    if (ImGui::Button("shake all")) {
        for (auto& point : points) {
            point.velocity += glm::vec3(
                (rand() % 200 - 100) / 100.0f,
                (rand() % 200 - 100) / 100.0f,
                (rand() % 200 - 100) / 100.0f
            );
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("stop all")) {
        for (auto& point : points) {
            point.velocity = glm::vec3(0.0f);
        }
    }
    
    ImGui::Separator();
    ImGui::Text("stats");
    ImGui::Text("mass points: %d", (int)points.size());
    ImGui::Text("springs: %d", (int)springs.size());
    
    // calc total energy
    float kineticEnergy = 0.0f;
    for (const auto& point : points) {
        kineticEnergy += 0.5f * point.mass * glm::dot(point.velocity, point.velocity);
    }
    ImGui::Text("kinetic energy: %.2f", kineticEnergy);
}

