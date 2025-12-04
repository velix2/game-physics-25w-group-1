#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>

// simple rigid body for 3d boxes
struct RigidBody {
    // state
    glm::vec3 position = glm::vec3(0);         // center of mass
    glm::vec3 velocity = glm::vec3(0);         // linear velocity
    glm::quat orientation = glm::quat(1,0,0,0); // rotation quaternion (w,x,y,z)
    glm::vec3 angularVelocity = glm::vec3(0);  // angular velocity in world space
    
    // properties
    float mass = 1.0f;
    glm::vec3 extent = glm::vec3(1);           // size in x, y, z
    
    // computed from extent and mass - inertia tensor in body space (diagonal for box)
    glm::mat3 getBodyInertiaTensor() const {
        float x2 = extent.x * extent.x;
        float y2 = extent.y * extent.y;
        float z2 = extent.z * extent.z;
        
        // inertia tensor for a box: I = (1/12) * M * diag(y^2+z^2, x^2+z^2, x^2+y^2)
        glm::mat3 I(0);
        I[0][0] = (mass / 12.0f) * (y2 + z2);
        I[1][1] = (mass / 12.0f) * (x2 + z2);
        I[2][2] = (mass / 12.0f) * (x2 + y2);
        return I;
    }
    
    // get inertia tensor in world space: I_world = R * I_body * R^T
    glm::mat3 getWorldInertiaTensor() const {
        glm::mat3 R = glm::toMat3(orientation);
        glm::mat3 I_body = getBodyInertiaTensor();
        return R * I_body * glm::transpose(R);
    }
    
    // get inverse inertia tensor in world space
    glm::mat3 getWorldInverseInertiaTensor() const {
        return glm::inverse(getWorldInertiaTensor());
    }
    
    // apply force at world position, returns torque
    glm::vec3 computeTorque(const glm::vec3& force, const glm::vec3& worldPos) const {
        glm::vec3 r = worldPos - position; // lever arm
        return glm::cross(r, force);
    }
    
    // integrate one time step using explicit euler
    void integrate(float dt, const glm::vec3& force, const glm::vec3& torque) {
        // linear motion
        glm::vec3 linearAccel = force / mass;
        velocity += linearAccel * dt;
        position += velocity * dt;
        
        // angular motion
        glm::mat3 I_inv = getWorldInverseInertiaTensor();
        glm::vec3 angularAccel = I_inv * torque;
        angularVelocity += angularAccel * dt;
        
        // update orientation: q_new = q + 0.5 * dt * omega_quat * q
        // where omega_quat = (0, omega.x, omega.y, omega.z)
        glm::quat omegaQuat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
        orientation = orientation + 0.5f * dt * omegaQuat * orientation;
        orientation = glm::normalize(orientation); // keep unit length
    }
    
    // get world space velocity of a point on the rigid body (given in body-local coords)
    glm::vec3 getPointVelocity(const glm::vec3& bodyLocalPoint) const {
        glm::vec3 r = glm::toMat3(orientation) * bodyLocalPoint;
        return velocity + glm::cross(angularVelocity, r);
    }
    
    // get velocity at a world-space offset from center of mass (for collision response)
    glm::vec3 getVelocityAtOffset(const glm::vec3& worldOffset) const {
        return velocity + glm::cross(angularVelocity, worldOffset);
    }
    
    // get world position of a body-local point
    glm::vec3 getWorldPosition(const glm::vec3& bodyLocalPoint) const {
        return position + glm::toMat3(orientation) * bodyLocalPoint;
    }
    
    // get transformation matrix from object space to world space (for rendering/collision)
    glm::mat4 getWorldFromObj() const {
        glm::mat4 T = glm::translate(glm::mat4(1), position);
        glm::mat4 R = glm::toMat4(orientation);
        glm::mat4 S = glm::scale(glm::mat4(1), extent);
        return T * R * S;
    }
    
    void print(const std::string& label = "") const {
        if (!label.empty()) std::cout << "=== " << label << " ===" << std::endl;
        std::cout << "position: [" << position.x << ", " << position.y << ", " << position.z << "]" << std::endl;
        std::cout << "velocity: [" << velocity.x << ", " << velocity.y << ", " << velocity.z << "]" << std::endl;
        std::cout << "orientation (w,x,y,z): [" << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z << "]" << std::endl;
        std::cout << "angularVelocity: [" << angularVelocity.x << ", " << angularVelocity.y << ", " << angularVelocity.z << "]" << std::endl;
    }
};
