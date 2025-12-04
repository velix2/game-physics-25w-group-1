#include "Renderer.h"
#include "util/CollisionDetection.h"
#include <array>
#pragma once

glm::vec3 screenToWorldRay(
    const glm::mat4 &proj,
    const glm::mat4 &view,
    float mx, float my,
    float screenW, float screenH);
glm::mat3 compInitialInertia(glm::vec3 extent, float mass);
std::array<glm::vec3, 8> compOffsets(glm::vec3 extent);

struct Body
{
    bool fixed;
    glm::vec3 cm;
    const glm::vec3 extent;
    const std::array<glm::vec3, 8> offsets; // x_i
    glm::vec3 force;                        // F
    glm::vec3 torque;                       // q
    glm::quat orientation;                  // r
    const glm::mat3 initialInertia;         // I_0^-1
    glm::mat3 inertia;                      // I^-1
    glm::vec3 angularMomentum;              // L
    glm::vec3 angularVelocity;              // w
    glm::vec3 linearVelocity;               // v
    float mass;                             // M
    float inverseMass;                      // M^-1

    Body(glm::vec3 cm, glm::vec3 linearVelocity, glm::quat orientation, glm::vec3 angularMomentum, float mass, glm::vec3 extent, bool fixed)
        : fixed(fixed), cm(cm), offsets(compOffsets(extent)), extent(extent), force(glm::vec3(0)), torque(glm::vec3(0)), orientation(orientation),
          initialInertia(fixed ? glm::mat3(0) : compInitialInertia(extent, mass)), angularMomentum(angularMomentum), linearVelocity(linearVelocity), mass(mass),
          inverseMass(fixed ? 0 : 1.0 / mass)
    {
        glm::mat3 rot = static_cast<glm::mat3>(this->orientation);
        this->inertia = rot * this->initialInertia * glm::transpose(rot);
        this->angularVelocity = this->inertia * angularMomentum;
    }

    glm::mat4 getWorldFromObj();
    glm::vec3 getLocalPos(glm::vec3 worldPos);
    glm::vec3 getVelocityAt(glm::vec3 worldPos);
    void clearForce();
    void applyForceAt(glm::vec3 pos, glm::vec3 force);
    void applyDirectForce(glm::vec3 force);
    void integrate(float dt);
    void draw(Renderer &renderer);
    bool intersectRay(glm::vec3 origin, glm::vec3 direction, glm::vec3 &hitPoint);
    // If this body collides with provided body, apply collision force
    bool doCollide(Body &other, float c);
    void print();
    void printPoint(glm::vec3 pos);
};
