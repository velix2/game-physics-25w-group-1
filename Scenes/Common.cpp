#include "Common.h"

glm::vec3 screenToWorldRay(
    const glm::mat4 &proj,
    const glm::mat4 &view,
    float mx, float my,
    float screenW, float screenH)
{
    float x = (2.0f * mx) / screenW - 1.0f;
    float y = 1.0f - (2.0f * my) / screenH;

    // float zn = isDirectX ? 0.0f : -1.0f;
    float zn = -1.0f;
    float zf = 1.0f;

    glm::vec4 clipNear(x, y, zn, 1.0f);
    glm::vec4 clipFar(x, y, zf, 1.0f);

    glm::mat4 invProj = glm::inverse(proj);
    glm::mat4 invView = glm::inverse(view);

    glm::vec4 eyeNear = invProj * clipNear;
    eyeNear /= eyeNear.w;
    glm::vec4 eyeFar = invProj * clipFar;
    eyeFar /= eyeFar.w;

    glm::vec4 worldNear = invView * eyeNear;
    worldNear /= worldNear.w;
    glm::vec4 worldFar = invView * eyeFar;
    worldFar /= worldFar.w;

    glm::vec3 dir = glm::normalize(glm::vec3(worldFar - worldNear));
    return dir;
}

glm::mat3 compInitialInertia(glm::vec3 extent, float mass)
{
    glm::mat3 inertia = glm::mat3(0.0f);
    inertia[0][0] = mass * (extent.y * extent.y + extent.z * extent.z) / 12.0f;
    inertia[1][1] = mass * (extent.x * extent.x + extent.z * extent.z) / 12.0f;
    inertia[2][2] = mass * (extent.y * extent.y + extent.x * extent.x) / 12.0f;
    return glm::inverse(inertia);
}

// ppp, ppn, pnp, pnn, npp, npn, nnp, nnn
std::array<glm::vec3, 8> compOffsets(glm::vec3 extent)
{
    std::array<glm::vec3, 8> points;
    glm::vec3 x = extent * glm::vec3(0.5, 0, 0);
    glm::vec3 y = extent * glm::vec3(0, 0.5, 0);
    glm::vec3 z = extent * glm::vec3(0, 0, 0.5);
    points[0] = x + y + z;
    points[1] = x + y - z;
    points[2] = x - y + z;
    points[3] = x - y - z;
    points[4] = -x + y + z;
    points[5] = -x + y - z;
    points[6] = -x - y + z;
    points[7] = -x - y - z;
    return points;
}

glm::mat4 Body::getWorldFromObj()
{
    glm::mat4 rotationMatrix = static_cast<glm::mat4>(this->orientation);
    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1), this->extent);
    glm::mat4 translationMatrix = glm::translate(glm::mat4(1), this->cm);
    return translationMatrix * rotationMatrix * scaleMatrix;
}

glm::vec3 Body::getLocalPos(glm::vec3 worldPos)
{
    return glm::inverse(static_cast<glm::mat3>(this->orientation)) * (worldPos - this->cm);
}

glm::vec3 Body::getVelocityAt(glm::vec3 worldPos)
{
    return this->linearVelocity + glm::cross(this->angularVelocity, getLocalPos(worldPos)); // TODO check if only translation is enough
    // return this->linearVelocity + glm::cross(this->angularVelocity, worldPos - this->cm); // TODO check if only translation is enough
}

void Body::clearForce()
{
    this->force = glm::vec3(0);
    this->torque = glm::vec3(0);
}

void Body::applyForceAt(glm::vec3 worldPos, glm::vec3 force)
{
    // glm::vec3 localPos = getLocalPos(worldPos);
    glm::vec3 localPos = worldPos - this->cm; // Only translation, no rotation
    this->force += force;
    this->torque += glm::cross(localPos, force);
}

void Body::applyDirectForce(glm::vec3 force)
{
    this->force += force;
}

void Body::integrate(float dt)
{
    if (this->fixed)
    {
        return;
    }
    this->cm += dt * this->linearVelocity;
    this->linearVelocity += (dt / this->mass) * (this->force);
    this->orientation = glm::normalize(this->orientation + (dt / 2) * (glm::quat(0, this->angularVelocity) * this->orientation));
    this->angularMomentum += dt * this->torque;
    glm::mat3 rot = static_cast<glm::mat3>(this->orientation);
    this->inertia = rot * this->initialInertia * glm::transpose(rot);
    this->angularVelocity = this->inertia * this->angularMomentum;
    clearForce();
}

void Body::draw(Renderer &renderer)
{
    glm::mat3 rot = static_cast<glm::mat3>(this->orientation);
    std::array<glm::vec3, 8> worldPoints;
    // Vertices
    for (size_t i = 0; i < 8; i++)
    {
        worldPoints[i] = cm + rot * offsets[i];
        renderer.drawSphere(worldPoints[i], 0.01, glm::vec4(1, 1, 1, 1));
    }
    // Edges
    for (size_t i = 0; i < 4; i++)
    {
        renderer.drawLine(worldPoints[2 * i], worldPoints[2 * i + 1], glm::vec4(0.5, 0.5, 0.5, 1));
    }
    for (size_t i = 0; i < 2; i++)
    {
        renderer.drawLine(worldPoints[i], worldPoints[i + 2], glm::vec4(0.5, 0.5, 0.5, 1));
    }
    for (size_t i = 4; i < 6; i++)
    {
        renderer.drawLine(worldPoints[i], worldPoints[i + 2], glm::vec4(0.5, 0.5, 0.5, 1));
    }
    for (size_t i = 0; i < 4; i++)
    {
        renderer.drawLine(worldPoints[i], worldPoints[i + 4], glm::vec4(0.5, 0.5, 0.5, 1));
    }
    // return; // Debug
    // Faces
    glm::mat3 rotx = rot * static_cast<glm::mat3>(glm::quat(glm::vec3(0, glm::pi<float>() / 2, 0)));
    glm::vec2 xs = glm::vec2(extent.z, extent.y);
    glm::vec3 xp = cm + rot * (extent * glm::vec3(0.5, 0, 0));
    glm::vec3 xn = cm + rot * (extent * glm::vec3(-0.5, 0, 0));
    renderer.drawQuad(xp, rotx, xs, glm::vec4(0.3, 0.3, 0.3, 1));
    renderer.drawQuad(xn, rotx, xs, glm::vec4(0.3, 0.3, 0.3, 1));
    glm::mat3 roty = rot * static_cast<glm::mat3>(glm::quat(glm::vec3(glm::pi<float>() / 2, 0, 0)));
    glm::vec2 ys = glm::vec2(extent.x, extent.z);
    glm::vec3 yp = cm + rot * (extent * glm::vec3(0, 0.5, 0));
    glm::vec3 yn = cm + rot * (extent * glm::vec3(0, -0.5, 0));
    renderer.drawQuad(yp, roty, ys, glm::vec4(0.3, 0.3, 0.3, 1));
    renderer.drawQuad(yn, roty, ys, glm::vec4(0.3, 0.3, 0.3, 1));
    glm::vec2 zs = glm::vec2(extent.x, extent.y);
    glm::vec3 zp = cm + rot * (extent * glm::vec3(0, 0, 0.5));
    glm::vec3 zn = cm + rot * (extent * glm::vec3(0, 0, -0.5));
    renderer.drawQuad(zp, rot, zs, glm::vec4(0.3, 0.3, 0.3, 1));
    renderer.drawQuad(zn, rot, zs, glm::vec4(0.3, 0.3, 0.3, 1));
}

float max(float a, float b)
{
    return a > b ? a : b;
}

float min(float a, float b)
{
    return a < b ? a : b;
}

// https://gamedev.stackexchange.com/a/18459
bool Body::intersectRay(glm::vec3 origin, glm::vec3 direction, glm::vec3 &hitPoint)
{
    glm::mat3 rot = static_cast<glm::mat3>(this->orientation);
    glm::mat3 unrot = glm::inverse(rot);
    glm::vec3 rorg = unrot * (origin - cm);
    glm::vec3 localDirection = unrot * direction;
    glm::vec3 lb = -extent / 2.0f;
    glm::vec3 rt = extent / 2.0f;
    glm::vec3 dirfrac = 1.0f / localDirection;
    float t1 = (lb.x - rorg.x) * dirfrac.x;
    float t2 = (rt.x - rorg.x) * dirfrac.x;
    float t3 = (lb.y - rorg.y) * dirfrac.y;
    float t4 = (rt.y - rorg.y) * dirfrac.y;
    float t5 = (lb.z - rorg.z) * dirfrac.z;
    float t6 = (rt.z - rorg.z) * dirfrac.z;
    float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
    float t;
    // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
    if (tmax < 0)
    {
        t = tmax;
        return false;
    }
    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        t = tmax;
        return false;
    }

    t = tmin;

    hitPoint = rorg + t * localDirection;
    hitPoint = cm + rot * hitPoint;
    return true;
}

void printMatrix(const glm::mat3 &mat)
{
    printf("| % 5.2f % 5.2f % 5.2f |\n", mat[0][0], mat[0][1], mat[0][2]);
    printf("| % 5.2f % 5.2f % 5.2f |\n", mat[1][0], mat[1][1], mat[1][2]);
    printf("| % 5.2f % 5.2f % 5.2f |\n", mat[2][0], mat[2][1], mat[2][2]);
}

bool Body::doCollide(Body &rbb, float c)
{
    if (this->fixed && rbb.fixed)
    {
        return false;
    }
    Body &rba = *this;
    // this is A, other is B
    auto info = collisionTools::checkCollisionSAT(rba.getWorldFromObj(), rbb.getWorldFromObj());
    if (!info.isColliding)
    {
        return false;
    }

    auto n = info.normalWorld; // impulse from B to A
    // Normal should be correct, may need to flip
    glm::vec3 vrel = rba.getVelocityAt(info.collisionPointWorld) - rbb.getVelocityAt(info.collisionPointWorld);
    // glm::vec3 vrel = rba.linearVelocity - rbb.linearVelocity;
    if (glm::dot(vrel, n) > 0)
    {
        return false;
    }
    // FIRST: Determine correct normal direction using center-to-center
    // Normal should point from B towards A (separation direction)
    glm::vec3 centerDir = rba.cm - rbb.cm;
    if (glm::dot(n, centerDir) < 0)
    {
        n = -n; // Flip so n points from B to A
    }
    // Position correction - strong to prevent sinking
    if (info.depth > 0.001f)
    {
        float correction = info.depth * 0.8f; // 80% correction

        // Only move dynamic bodies
        if (rba.inverseMass > 0)
            rba.cm += n * correction;
        if (rbb.inverseMass > 0)
            rbb.cm -= n * correction;
    }
    glm::vec3 xa = info.collisionPointWorld - rba.cm;
    glm::vec3 xb = info.collisionPointWorld - rbb.cm;
    float numerator = -(1 + c) * glm::dot(vrel, n);
    glm::vec3 parta = glm::vec3(0);
    if (!rba.fixed)
    {
        parta = glm::cross(rba.inertia * glm::cross(xa, n), xa);
    }
    glm::vec3 partb = glm::vec3(0);
    if (!rbb.fixed)
    {
        partb = glm::cross(rbb.inertia * glm::cross(xb, n), xb);
    }
    float denominator = rba.inverseMass + rbb.inverseMass + glm::dot(parta + partb, n);
    float j = numerator / denominator;
    if (!rba.fixed)
    {
        rba.linearVelocity += (j * rba.inverseMass) * n;
        rba.angularMomentum += glm::cross(xa, j * n);
    }
    if (!rbb.fixed)
    {
        rbb.linearVelocity -= (j * rbb.inverseMass) * n;
        rbb.angularMomentum -= glm::cross(xb, j * n);
    }

    return true;
}

void Body::print()
{
    printf("================ Rigidbody state: ================\n");
    printf("Is fixed?               %s\n", this->fixed ? "yes" : "no");
    printf("CM Position:            (%.2f, %.2f, %.2f)\n", this->cm.x, this->cm.y, this->cm.z);
    printf("Velocity:               (%.2f, %.2f, %.2f)\n", this->linearVelocity.x, this->linearVelocity.y, this->linearVelocity.z);
    printf("Mass:                   %.2f\n", this->mass);
    printf("Rotation:               (%.2f, %.2f, %.2f, %.2f)\n", this->orientation.w, this->orientation.x, this->orientation.y, this->orientation.z);
    printf("Rotation Matrix:\n");
    printMatrix(glm::mat3_cast(this->orientation));
    printf("Angular Momentum:       (%.2f, %.2f, %.2f)\n", this->angularMomentum.x, this->angularMomentum.y, this->angularMomentum.z);
    printf("Angular Velocity:       (%.2f, %.2f, %.2f)\n", this->angularVelocity.x, this->angularVelocity.y, this->angularVelocity.z);
    printf("Current Inverted Inertia Tensor:\n");
    printMatrix(this->inertia);
    printf("Initial Inverted Inertia Tensor:\n");
    printMatrix(this->initialInertia);
    printf("==================================================\n");
}

void Body::printPoint(glm::vec3 pos)
{
    printf("================== Point state: ==================\n");
    printf("World Position:         (%.2f, %.2f, %.2f)\n", pos.x, pos.y, pos.z);
    glm::vec3 localPos = glm::inverse(static_cast<glm::mat3>(this->orientation)) * (pos - this->cm);
    printf("Local Position:         (%.2f, %.2f, %.2f)\n", localPos.x, localPos.y, localPos.z);
    glm::vec3 velocity = this->linearVelocity + glm::cross(this->angularVelocity, localPos);
    printf("Velocity:               (%.2f, %.2f, %.2f)\n", velocity.x, velocity.y, velocity.z);
    printf("==================================================\n");
}