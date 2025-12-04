#include "Simulation.h"
#include <imgui.h>

void Simulation::init() {
    resetBody();
}

void Simulation::resetBody() {
    // setup rigid body as specified in exercise
    body.position = glm::vec3(0, 0, 0);
    body.extent = glm::vec3(1.0f, 0.6f, 0.5f);
    body.mass = 2.0f;
    body.orientation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1));
    body.velocity = glm::vec3(0);
    body.angularVelocity = glm::vec3(0);
}

void Simulation::handleInput() {
    userForce = glm::vec3(0);
    
    // keyboard forces - apply force at center of mass
    if (ImGui::IsKeyDown(ImGuiKey_I)) userForce.y += forceMagnitude;  // forward
    if (ImGui::IsKeyDown(ImGuiKey_K)) userForce.y -= forceMagnitude;  // backward
    if (ImGui::IsKeyDown(ImGuiKey_J)) userForce.x -= forceMagnitude;  // left
    if (ImGui::IsKeyDown(ImGuiKey_L)) userForce.x += forceMagnitude;  // right
    if (ImGui::IsKeyDown(ImGuiKey_U)) userForce.z += forceMagnitude;  // up
    if (ImGui::IsKeyDown(ImGuiKey_O)) userForce.z -= forceMagnitude;  // down
    
    // mouse drag force
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        auto drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        if (drag.x != 0 || drag.y != 0) {
            userForce += camRight * drag.x * 0.1f;
            userForce -= camUp * drag.y * 0.1f;
        }
    }
}

void Simulation::simulateStep() {
    if (paused) return;
    
    handleInput();
    
    // compute total force and torque
    glm::vec3 totalForce = userForce;
    glm::vec3 totalTorque = glm::vec3(0);
    
    // add exercise force if enabled
    if (applyInitialForce) {
        totalForce += externalForce;
        totalTorque += body.computeTorque(externalForce, forcePosition);
    }
    
    // add torque from user force (applied at center, so no torque)
    // if you want torque from user force, apply it at a corner:
    // totalTorque += body.computeTorque(userForce, body.position + someOffset);
    
    body.integrate(dt, totalForce, totalTorque);
}

void Simulation::onDraw(Renderer& renderer) {
    // save camera info for mouse interaction
    cameraMatrix = renderer.camera.viewMatrix;
    camRight = glm::vec3(glm::inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0));
    camUp = glm::vec3(glm::inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0));
    
    // draw world bounds
    renderer.drawWireCube(glm::vec3(0), glm::vec3(10), glm::vec3(0.3f));
    
    // draw the rigid body
    renderer.drawCube(body.position, body.orientation, body.extent, glm::vec4(0.8f, 0.2f, 0.2f, 1.0f));
    
    // draw force application point if force is enabled
    if (applyInitialForce) {
        renderer.drawSphere(forcePosition, 0.05f, glm::vec4(1, 1, 0, 1));
        renderer.drawLine(forcePosition, forcePosition + externalForce * 0.5f, glm::vec3(1, 1, 0));
    }
    
    // draw velocity vector
    renderer.drawLine(body.position, body.position + body.velocity * 0.3f, glm::vec3(0, 1, 0));
    
    // draw angular velocity
    renderer.drawLine(body.position, body.position + body.angularVelocity * 0.1f, glm::vec3(0, 0.5f, 1));
}

void Simulation::onGUI() {
    ImGui::Text("=== Simulation Controls ===");
    
    if (ImGui::Button(paused ? "Play" : "Pause")) {
        paused = !paused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        resetBody();
    }
    
    ImGui::SliderFloat("Time Step (dt)", &dt, 0.001f, 0.1f, "%.4f");
    ImGui::Checkbox("Apply Exercise Force", &applyInitialForce);
    
    ImGui::Separator();
    ImGui::Text("=== External Force ===");
    ImGui::SliderFloat3("Force", &externalForce.x, -10.0f, 10.0f);
    ImGui::SliderFloat3("Force Position", &forcePosition.x, -2.0f, 2.0f);
    
    ImGui::Separator();
    ImGui::Text("=== User Interaction ===");
    ImGui::SliderFloat("Force Magnitude", &forceMagnitude, 1.0f, 20.0f);
    ImGui::Text("Keys: I/K (Y), J/L (X), U/O (Z)");
    ImGui::Text("Right-click drag to apply force");
    
    ImGui::Separator();
    ImGui::Text("=== State ===");
    ImGui::Text("Position: [%.2f, %.2f, %.2f]", body.position.x, body.position.y, body.position.z);
    ImGui::Text("Velocity: [%.2f, %.2f, %.2f]", body.velocity.x, body.velocity.y, body.velocity.z);
    ImGui::Text("Angular Vel: [%.2f, %.2f, %.2f]", body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z);
}

