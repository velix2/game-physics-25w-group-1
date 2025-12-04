#include "Complex.h"
#include <imgui.h>
#include <util/CollisionDetection.h>
#include <algorithm>

void Complex::init() {
    resetScene();
}

void Complex::addBox(glm::vec3 pos, glm::vec3 size, float mass, glm::quat rot) {
    RigidBody body;
    body.position = pos;
    body.extent = size;
    body.mass = mass;
    body.orientation = rot;
    body.velocity = glm::vec3(0);
    body.angularVelocity = glm::vec3(0);
    bodies.push_back(body);
}

void Complex::resetScene() {
    bodies.clear();
    activeCollisions.clear();
    
    // Ground (static heavy box)
    RigidBody ground;
    ground.position = glm::vec3(0, 0, -2.0f);
    ground.extent = glm::vec3(20.0f, 20.0f, 1.0f);
    ground.mass = 100000.0f; // practically infinite
    ground.orientation = glm::quat(1, 0, 0, 0);
    bodies.push_back(ground);
    
    // Wall 1
    addBox(glm::vec3(-5, 0, 0), glm::vec3(1, 10, 5), 10000.0f);
    // Wall 2
    addBox(glm::vec3(5, 0, 0), glm::vec3(1, 10, 5), 10000.0f);
    // Wall 3
    addBox(glm::vec3(0, 5, 0), glm::vec3(10, 1, 5), 10000.0f);
    // Wall 4
    addBox(glm::vec3(0, -5, 0), glm::vec3(10, 1, 5), 10000.0f);
    
    // Stack of boxes
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                addBox(glm::vec3(i*1.2f - 1.2f, j*1.2f - 1.2f, k*1.2f + 2.0f), 
                       glm::vec3(1.0f), 
                       1.0f);
            }
        }
    }
    
    spawnTimer = 0.0f;
}

void Complex::spawnRandomCube() {
    std::uniform_real_distribution<float> posDist(-3.0f, 3.0f);
    std::uniform_real_distribution<float> heightDist(5.0f, 10.0f);
    std::uniform_real_distribution<float> sizeDist(0.3f, 1.2f);
    std::uniform_real_distribution<float> angleDist(0.0f, glm::two_pi<float>());
    std::uniform_real_distribution<float> massDist(0.5f, 3.0f);
    
    // random position above the arena
    glm::vec3 pos(posDist(rng), posDist(rng), heightDist(rng));
    
    // random size (can be non-uniform for weird shapes)
    glm::vec3 size(sizeDist(rng), sizeDist(rng), sizeDist(rng));
    
    // random rotation - create from random euler angles
    glm::vec3 euler(angleDist(rng), angleDist(rng), angleDist(rng));
    glm::quat rot = glm::quat(euler);
    
    // random mass
    float mass = massDist(rng);
    
    // random initial angular velocity for extra chaos
    std::uniform_real_distribution<float> spinDist(-2.0f, 2.0f);
    
    RigidBody body;
    body.position = pos;
    body.extent = size;
    body.mass = mass;
    body.orientation = rot;
    body.velocity = glm::vec3(0, 0, -1.0f); // slight downward velocity
    body.angularVelocity = glm::vec3(spinDist(rng), spinDist(rng), spinDist(rng));
    bodies.push_back(body);
}

bool Complex::handleCollision(RigidBody& a, RigidBody& b, bool& wasColliding) {
    // If both are static, ignore collision
    if (a.mass > 5000.0f && b.mass > 5000.0f) return false;

    glm::mat4 worldFromA = a.getWorldFromObj();
    glm::mat4 worldFromB = b.getWorldFromObj();
    
    // Check both ways since SAT only detects B hitting A
    CollisionInfo infoAB = collisionTools::checkCollisionSAT(worldFromA, worldFromB);
    CollisionInfo infoBA = collisionTools::checkCollisionSAT(worldFromB, worldFromA);
    
    bool isColliding = infoAB.isColliding || infoBA.isColliding;
    
    if (!isColliding) {
        wasColliding = false;
        return false;
    }
    
    // Use the valid collision info
    CollisionInfo& info = infoAB.isColliding ? infoAB : infoBA;
    glm::vec3 n = infoAB.isColliding ? info.normalWorld : -info.normalWorld;
    glm::vec3 p = info.collisionPointWorld;
    float depth = info.depth;
    
    // FIRST: Determine correct normal direction using center-to-center
    // Normal should point from B towards A (separation direction)
    glm::vec3 centerDir = a.position - b.position;
    if (glm::dot(n, centerDir) < 0) {
        n = -n; // Flip so n points from B to A
    }
    
    // Position correction - strong to prevent sinking
    if (depth > 0.001f) {
        float correction = depth * 0.8f; // 80% correction
        
        // Only move dynamic bodies
        if (a.mass < 5000.0f) a.position += n * correction;
        if (b.mass < 5000.0f) b.position -= n * correction;
    }
    
    // Impulse calculation
    glm::vec3 xA = p - a.position;
    glm::vec3 xB = p - b.position;
    
    glm::vec3 vRel = a.velocity - b.velocity;
    float vRelDotN = glm::dot(vRel, n);
    
    // Only apply impulse if approaching (vRelDotN < 0 means A approaching B along n)
    if (vRelDotN < 0) {
        float effectiveRestitution = wasColliding ? 0.0f : restitution;
        float numerator = -(1.0f + effectiveRestitution) * vRelDotN;
        
        glm::mat3 IA_inv = a.mass > 5000.0f ? glm::mat3(0) : a.getWorldInverseInertiaTensor();
        glm::mat3 IB_inv = b.mass > 5000.0f ? glm::mat3(0) : b.getWorldInverseInertiaTensor();
        
        float invMassA = a.mass > 5000.0f ? 0 : 1.0f/a.mass;
        float invMassB = b.mass > 5000.0f ? 0 : 1.0f/b.mass;
        
        float denomA = invMassA + glm::dot(IA_inv * glm::cross(glm::cross(xA, n), xA), n);
        float denomB = invMassB + glm::dot(IB_inv * glm::cross(glm::cross(xB, n), xB), n);
        
        float denominator = denomA + denomB;
        if (denominator < 1e-6f) {
            wasColliding = true;
            return true;
        }
        
        float J = numerator / denominator;
        
        // Apply impulse: A gets pushed in +n direction, B in -n direction
        if (a.mass < 5000.0f) {
            a.velocity += J * n * invMassA;
            a.angularVelocity += IA_inv * glm::cross(xA, J * n);
        }
        if (b.mass < 5000.0f) {
            b.velocity -= J * n * invMassB;
            b.angularVelocity -= IB_inv * glm::cross(xB, J * n);
        }
    }
    
    wasColliding = true;
    return true;
}

void Complex::handleInteraction() {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        auto drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        if (drag.x != 0 || drag.y != 0) {
            glm::vec3 force = camRight * drag.x * 0.5f - camUp * drag.y * 0.5f;
            
            // Apply to all non-static bodies
            for (auto& body : bodies) {
                if (body.mass < 5000.0f) {
                    body.velocity += force * 0.01f;
                }
            }
            ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
        }
    }
}

void Complex::simulateStep() {
    if (paused) return;
    
    handleInteraction();
    
    // Spawner - spawn random cubes every spawnInterval seconds
    if (autoSpawn) {
        spawnTimer += dt;
        if (spawnTimer >= spawnInterval) {
            spawnRandomCube();
            spawnTimer = 0.0f;
        }
    }
    
    // Integration
    for (auto& body : bodies) {
        if (body.mass < 5000.0f) { // Don't move walls/ground
            glm::vec3 force = gravityEnabled ? gravity * body.mass : glm::vec3(0);
            body.integrate(dt, force, glm::vec3(0));
        }
    }
    
    // Collision detection O(N^2) - simple broadphase
    // Track active collisions to maintain wasColliding state
    std::vector<Pair> newCollisions;
    
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            // Simple AABB check first optimization could go here
            
            // Find if we were colliding
            bool wasColliding = false;
            for (const auto& pair : activeCollisions) {
                if (pair.a == i && pair.b == j) {
                    wasColliding = true;
                    break;
                }
            }
            
            if (handleCollision(bodies[i], bodies[j], wasColliding)) {
                newCollisions.push_back({(int)i, (int)j});
            }
        }
    }
    activeCollisions = newCollisions;
}

void Complex::onDraw(Renderer& renderer) {
    // save camera info
    cameraMatrix = renderer.camera.viewMatrix;
    camRight = glm::vec3(glm::inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0));
    camUp = glm::vec3(glm::inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0));
    
    renderer.drawWireCube(glm::vec3(0), glm::vec3(20), glm::vec3(0.1f));
    
    for (const auto& body : bodies) {
        glm::vec4 color;
        if (body.mass > 5000.0f) {
            color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f); // Static = Gray
        } else {
            // Color based on velocity magnitude
            float speed = glm::length(body.velocity);
            color = glm::vec4(0.2f + std::min(speed*0.1f, 0.8f), 0.3f, 0.8f - std::min(speed*0.1f, 0.6f), 1.0f);
        }
        renderer.drawCube(body.position, body.orientation, body.extent, color);
    }
}

void Complex::onGUI() {
    ImGui::Text("=== Complex Scene ===");
    
    if (ImGui::Button(paused ? "Play" : "Pause")) paused = !paused;
    ImGui::SameLine();
    if (ImGui::Button("Reset")) resetScene();
    
    ImGui::SliderFloat("Time Step", &dt, 0.001f, 0.05f);
    ImGui::SliderFloat("Restitution", &restitution, 0.0f, 1.0f);
    ImGui::Checkbox("Gravity", &gravityEnabled);
    
    ImGui::Separator();
    ImGui::Text("=== Spawner ===");
    ImGui::Checkbox("Auto Spawn", &autoSpawn);
    ImGui::SliderFloat("Spawn Interval (s)", &spawnInterval, 1.0f, 10.0f);
    if (ImGui::Button("Spawn Now!")) {
        spawnRandomCube();
    }
    ImGui::Text("Next spawn in: %.1fs", spawnInterval - spawnTimer);
    
    ImGui::Separator();
    ImGui::Text("Right-click drag to stir the boxes!");
    ImGui::Text("Bodies: %d", (int)bodies.size());
}

