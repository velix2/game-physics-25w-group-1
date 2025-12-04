#include "Collision.h"
#include <imgui.h>
#include <util/CollisionDetection.h>
#include <iostream>

void Collision::init() {
    resetBodies();
}

void Collision::resetBodies() {
    // simpler setup: X-Y plane only, one cube rotated 45 degrees
    
    // body A - on the left, moving right, rotated 45 deg around Z
    bodyA.position = glm::vec3(-2.0f, 0, 0);
    bodyA.extent = glm::vec3(1.0f, 1.0f, 1.0f);  // unit cube
    bodyA.mass = 2.0f;
    bodyA.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 0, 1));  // 45 deg rotation
    bodyA.velocity = glm::vec3(2, 0, 0);        // moving right
    bodyA.angularVelocity = glm::vec3(0);
    
    // body B - on the right, no rotation, stationary
    bodyB.position = glm::vec3(2.0f, 0, 0);
    bodyB.extent = glm::vec3(1.0f, 1.0f, 1.0f);  // unit cube
    bodyB.mass = 2.0f;
    bodyB.orientation = glm::quat(1, 0, 0, 0);   // no rotation
    bodyB.velocity = glm::vec3(0, 0, 0);         // stationary
    bodyB.angularVelocity = glm::vec3(0);
    
    // reset collision state
    wasColliding = false;
    
    std::cout << "\n=== RESET ===" << std::endl;
    std::cout << "A: pos=[" << bodyA.position.x << "," << bodyA.position.y << "], vel=[" << bodyA.velocity.x << "," << bodyA.velocity.y << "], rotated 45deg" << std::endl;
    std::cout << "B: pos=[" << bodyB.position.x << "," << bodyB.position.y << "], vel=[" << bodyB.velocity.x << "," << bodyB.velocity.y << "], no rotation" << std::endl;
}

bool Collision::handleCollision(RigidBody& a, RigidBody& b) {
    // get transformation matrices
    glm::mat4 worldFromA = a.getWorldFromObj();
    glm::mat4 worldFromB = b.getWorldFromObj();
    
    // check for collision
    CollisionInfo info = collisionTools::checkCollisionSAT(worldFromA, worldFromB);
    
    if (!info.isColliding) return false;
    
    // collision data
    glm::vec3 n = info.normalWorld;
    glm::vec3 xA = info.collisionPointWorld - a.position;
    glm::vec3 xB = info.collisionPointWorld - b.position;
    
    // relative velocity
    glm::vec3 vRel = a.velocity - b.velocity;
    float vRelDotN = glm::dot(vRel, n);
    
    // FIX: if vRelDotN > 0, normal points wrong way - flip it!
    // this ensures the impulse pushes bodies apart correctly
    if (vRelDotN > 0) {
        n = -n;
        vRelDotN = -vRelDotN;
    }
    
    std::cout << "\n=== COLLISION (A,B) ===" << std::endl;
    std::cout << "normal (corrected): [" << n.x << ", " << n.y << ", " << n.z << "]" << std::endl;
    std::cout << "vRel dot n: " << vRelDotN << std::endl;
    std::cout << "A vel before: [" << a.velocity.x << ", " << a.velocity.y << ", " << a.velocity.z << "]" << std::endl;
    std::cout << "B vel before: [" << b.velocity.x << ", " << b.velocity.y << ", " << b.velocity.z << "]" << std::endl;
    
    // impulse formula
    float numerator = -(1.0f + restitution) * vRelDotN;
    
    glm::mat3 IA_inv = a.getWorldInverseInertiaTensor();
    glm::mat3 IB_inv = b.getWorldInverseInertiaTensor();
    
    float denomA = 1.0f / a.mass + glm::dot(IA_inv * glm::cross(glm::cross(xA, n), xA), n);
    float denomB = 1.0f / b.mass + glm::dot(IB_inv * glm::cross(glm::cross(xB, n), xB), n);
    
    float J = numerator / (denomA + denomB);
    
    std::cout << "J (impulse): " << J << std::endl;
    
    // apply impulse
    a.velocity += J * n / a.mass;
    b.velocity -= J * n / b.mass;
    a.angularVelocity += IA_inv * glm::cross(xA, J * n);
    b.angularVelocity -= IB_inv * glm::cross(xB, J * n);
    
    std::cout << "A vel after: [" << a.velocity.x << ", " << a.velocity.y << ", " << a.velocity.z << "]" << std::endl;
    std::cout << "B vel after: [" << b.velocity.x << ", " << b.velocity.y << ", " << b.velocity.z << "]" << std::endl;
    
    return true;
}

void Collision::handleCollisionSwapped(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    // when called with swapped order, normal points from A to B
    glm::vec3 n = -info.normalWorld;
    glm::vec3 xA = info.collisionPointWorld - a.position;
    glm::vec3 xB = info.collisionPointWorld - b.position;
    
    // relative velocity
    glm::vec3 vRel = a.velocity - b.velocity;
    float vRelDotN = glm::dot(vRel, n);
    
    // FIX: if vRelDotN > 0, normal points wrong way - flip it!
    if (vRelDotN > 0) {
        n = -n;
        vRelDotN = -vRelDotN;
    }
    
    std::cout << "\n=== COLLISION SWAPPED ===" << std::endl;
    std::cout << "normal (corrected): [" << n.x << ", " << n.y << ", " << n.z << "]" << std::endl;
    std::cout << "vRel dot n: " << vRelDotN << std::endl;
    std::cout << "A vel before: [" << a.velocity.x << ", " << a.velocity.y << ", " << a.velocity.z << "]" << std::endl;
    std::cout << "B vel before: [" << b.velocity.x << ", " << b.velocity.y << ", " << b.velocity.z << "]" << std::endl;
    
    // impulse
    float numerator = -(1.0f + restitution) * vRelDotN;
    
    glm::mat3 IA_inv = a.getWorldInverseInertiaTensor();
    glm::mat3 IB_inv = b.getWorldInverseInertiaTensor();
    
    float denomA = 1.0f / a.mass + glm::dot(IA_inv * glm::cross(glm::cross(xA, n), xA), n);
    float denomB = 1.0f / b.mass + glm::dot(IB_inv * glm::cross(glm::cross(xB, n), xB), n);
    
    float J = numerator / (denomA + denomB);
    
    std::cout << "J (impulse): " << J << std::endl;
    
    // apply impulse
    a.velocity += J * n / a.mass;
    b.velocity -= J * n / b.mass;
    a.angularVelocity += IA_inv * glm::cross(xA, J * n);
    b.angularVelocity -= IB_inv * glm::cross(xB, J * n);
    
    std::cout << "A vel after: [" << a.velocity.x << ", " << a.velocity.y << ", " << a.velocity.z << "]" << std::endl;
    std::cout << "B vel after: [" << b.velocity.x << ", " << b.velocity.y << ", " << b.velocity.z << "]" << std::endl;
}

void Collision::simulateStep() {
    if (paused) return;
    
    // integrate both bodies (no external forces)
    bodyA.integrate(dt, glm::vec3(0), glm::vec3(0));
    bodyB.integrate(dt, glm::vec3(0), glm::vec3(0));
    
    // check collision both ways (SAT only detects B's vertex hitting A's face)
    glm::mat4 worldFromA = bodyA.getWorldFromObj();
    glm::mat4 worldFromB = bodyB.getWorldFromObj();
    
    CollisionInfo infoAB = collisionTools::checkCollisionSAT(worldFromA, worldFromB);
    CollisionInfo infoBA = collisionTools::checkCollisionSAT(worldFromB, worldFromA);
    
    bool isColliding = infoAB.isColliding || infoBA.isColliding;
    
    if (isColliding) {
        // always apply position correction if overlapping
        CollisionInfo& info = infoAB.isColliding ? infoAB : infoBA;
        glm::vec3 n = infoAB.isColliding ? info.normalWorld : -info.normalWorld;
        float depth = info.depth;
        
        if (depth > 0) {
            // gentle correction: push apart 20% of depth per frame to avoid jitter
            float totalMass = bodyA.mass + bodyB.mass;
            float correction = depth * 0.2f;
            bodyA.position += n * correction * (bodyB.mass / totalMass);
            bodyB.position -= n * correction * (bodyA.mass / totalMass);
        }

        if (!wasColliding) {
            std::cout << "\n*** FIRST COLLISION FRAME ***" << std::endl;
            
            // first frame of collision - apply impulse
            if (infoAB.isColliding) {
                handleCollision(bodyA, bodyB);
            } else {
                handleCollisionSwapped(bodyB, bodyA, infoBA);
            }
            wasColliding = true;
        }
    } else {
        if (wasColliding) {
            std::cout << "\n*** COLLISION ENDED ***" << std::endl;
        }
        wasColliding = false;
    }
}

void Collision::onDraw(Renderer& renderer) {
    // save camera info
    cameraMatrix = renderer.camera.viewMatrix;
    camRight = glm::vec3(glm::inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0));
    camUp = glm::vec3(glm::inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0));
    
    // draw world bounds
    renderer.drawWireCube(glm::vec3(0), glm::vec3(10), glm::vec3(0.3f));
    
    // draw body A (red)
    renderer.drawCube(bodyA.position, bodyA.orientation, bodyA.extent, glm::vec4(0.9f, 0.2f, 0.2f, 1.0f));
    
    // draw body B (blue)
    renderer.drawCube(bodyB.position, bodyB.orientation, bodyB.extent, glm::vec4(0.2f, 0.4f, 0.9f, 1.0f));
    
    // draw velocity vectors
    renderer.drawLine(bodyA.position, bodyA.position + bodyA.velocity * 0.3f, glm::vec3(1, 0.5f, 0.5f));
    renderer.drawLine(bodyB.position, bodyB.position + bodyB.velocity * 0.3f, glm::vec3(0.5f, 0.5f, 1));
    
    // check collision and draw collision point
    glm::mat4 worldFromA = bodyA.getWorldFromObj();
    glm::mat4 worldFromB = bodyB.getWorldFromObj();
    CollisionInfo info = collisionTools::checkCollisionSAT(worldFromA, worldFromB);
    
    if (info.isColliding) {
        renderer.drawSphere(info.collisionPointWorld, 0.08f, glm::vec4(1, 1, 0, 1));
        renderer.drawLine(info.collisionPointWorld, 
                         info.collisionPointWorld + info.normalWorld * 0.5f, 
                         glm::vec3(1, 1, 0));
    }
}

void Collision::onGUI() {
    ImGui::Text("=== Collision Scene ===");
    
    if (ImGui::Button(paused ? "Play" : "Pause")) {
        paused = !paused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        resetBodies();
    }
    
    ImGui::SliderFloat("Time Step (dt)", &dt, 0.001f, 0.05f, "%.4f");
    ImGui::SliderFloat("Restitution (c)", &restitution, 0.0f, 1.0f, "%.2f");
    ImGui::Text("  0 = plastic, 1 = elastic");
    
    ImGui::Separator();
    ImGui::Text("=== Body A (Red) ===");
    ImGui::Text("Position: [%.2f, %.2f, %.2f]", bodyA.position.x, bodyA.position.y, bodyA.position.z);
    ImGui::Text("Velocity: [%.2f, %.2f, %.2f]", bodyA.velocity.x, bodyA.velocity.y, bodyA.velocity.z);
    ImGui::Text("Angular: [%.2f, %.2f, %.2f]", bodyA.angularVelocity.x, bodyA.angularVelocity.y, bodyA.angularVelocity.z);
    
    ImGui::Separator();
    ImGui::Text("=== Body B (Blue) ===");
    ImGui::Text("Position: [%.2f, %.2f, %.2f]", bodyB.position.x, bodyB.position.y, bodyB.position.z);
    ImGui::Text("Velocity: [%.2f, %.2f, %.2f]", bodyB.velocity.x, bodyB.velocity.y, bodyB.velocity.z);
    ImGui::Text("Angular: [%.2f, %.2f, %.2f]", bodyB.angularVelocity.x, bodyB.angularVelocity.y, bodyB.angularVelocity.z);
    
    // show collision status
    glm::mat4 worldFromA = bodyA.getWorldFromObj();
    glm::mat4 worldFromB = bodyB.getWorldFromObj();
    CollisionInfo info = collisionTools::checkCollisionSAT(worldFromA, worldFromB);
    
    ImGui::Separator();
    if (info.isColliding) {
        ImGui::TextColored(ImVec4(1, 1, 0, 1), "COLLISION DETECTED");
        ImGui::Text("Point: [%.2f, %.2f, %.2f]", 
                   info.collisionPointWorld.x, info.collisionPointWorld.y, info.collisionPointWorld.z);
        ImGui::Text("Normal: [%.2f, %.2f, %.2f]", 
                   info.normalWorld.x, info.normalWorld.y, info.normalWorld.z);
    } else {
        ImGui::Text("No collision");
    }
}

