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

void Spring::computeElasticForces(float dt, bool doDamping)
{
    glm::vec3 lVec = (point1->cm - point2->cm);
    double l = sqrt(lVec.x * lVec.x + lVec.y * lVec.y + lVec.z * lVec.z);
    // If two points have the same position, the computed force would be infinite.
    // In this case, the assumed force is based on the velocities of the points, as if the points collided.
    if (l == 0)
    {
        glm::vec3 vrel = point2->linearVelocity - point1->linearVelocity;
        float normFactor = sqrt(dot(vrel, vrel));
        if (normFactor == 0)
        {
            vrel = glm::vec3(0, 0, 1);
            normFactor = 1;
        }
        glm::vec3 n = vrel / normFactor;
        glm::vec3 f = 1000000 * 0.001f * n;
        point1->force += f;
        point2->force += -f;
        return;
    }
    float scaleFactor = (-stiffness * (l - restLength)) / l;
    glm::vec3 force = lVec * scaleFactor;

    point1->force += force;
    if (doDamping)
    {
        point1->force -= point1->damping * point1->linearVelocity;
    }
    point2->force += -force;
    if (doDamping)
    {
        point2->force -= point2->damping * point2->linearVelocity;
    }
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
    // Use world-space offset from center of mass (not local-space position)
    return this->linearVelocity + glm::cross(this->angularVelocity, worldPos - this->cm);
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

    this->linearVelocity += (dt / this->mass) * (this->force);
    this->angularMomentum += dt * this->torque;
    this->angularVelocity = this->inertia * this->angularMomentum;
    this->cm += dt * this->linearVelocity;
    this->orientation = glm::normalize(this->orientation + (dt / 2) * (glm::quat(0, this->angularVelocity) * this->orientation));
    glm::mat3 rot = static_cast<glm::mat3>(this->orientation);
    this->inertia = rot * this->initialInertia * glm::transpose(rot);
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
    printf("| % 5.3f % 5.3f % 5.3f |\n", mat[0][0], mat[1][0], mat[2][0]);
    printf("| % 5.3f % 5.3f % 5.3f |\n", mat[0][1], mat[1][1], mat[2][1]);
    printf("| % 5.3f % 5.3f % 5.3f |\n", mat[0][2], mat[1][2], mat[2][2]);
}
bool Body::doCollide(Body &rbb, float c, float friction)
{
    if (this->fixed && rbb.fixed)
        return false;
    Body &rba = *this;

    auto rbaMat = rba.getWorldFromObj();
    auto rbbMat = rbb.getWorldFromObj();

    // Broad collision check with the templates SAT
    auto infoAB = collisionTools::checkCollisionSAT(rbaMat, rbbMat);
    auto infoBA = collisionTools::checkCollisionSAT(rbbMat, rbaMat);

    if (!infoAB.isColliding && !infoBA.isColliding)
        return false;

    CollisionInfo info = infoAB.isColliding ? (infoBA.isColliding ? (infoAB.depth > infoBA.depth ? infoAB : infoBA) : infoAB) : infoBA;

    glm::vec3 n = info.normalWorld;
    if (glm::dot(n, rbb.cm - rba.cm) < 0)
        n = -n;

    // Generate contact polygon; this polygon describes the area where the two cubes overlap
    // e.g. is a full square when perfectly aligned, or a smaller rectangle when the cubes are shifted 
    std::vector<glm::vec3> faceA, faceB; // The colliding faces of the two cubes, we use the normal to find the best one
    getBestFace(rba, n, faceA);
    getBestFace(rbb, -n, faceB);

    std::vector<glm::vec3> contactPoly = faceB;
    glm::vec3 centerA = (faceA[0] + faceA[1] + faceA[2] + faceA[3]) * 0.25f;

    for (int i = 0; i < 4; i++)
    {
        glm::vec3 p1 = faceA[i];
        glm::vec3 p2 = faceA[(i + 1) % 4];
        glm::vec3 sideNormal = glm::normalize(glm::cross(p2 - p1, n));
        if (glm::dot(sideNormal, centerA - p1) < 0)
            sideNormal = -sideNormal;

        contactPoly = clip(contactPoly, p1, sideNormal);
        if (contactPoly.empty())
            break;
    }

    // Create manifolds; these are kinda similar to the collision point from the original SAT
    // but we use multiple ones, to apply a force to each point. Makes it more stable
    std::vector<glm::vec3> manifolds;
    float maxDepth = 0.0f;
    for (const auto &p : contactPoly)
    {
        float d = glm::dot(p - centerA, -n);
        if (d >= -0.01f)
        {
            manifolds.push_back(p);
            if (d > maxDepth)
                maxDepth = d;
        }
    }

    if (manifolds.empty())
    {
        manifolds.push_back(info.collisionPointWorld);
        maxDepth = info.depth;
    }

    // Position Correction, when blocks intersect. prevents huge forces etc.
    if (maxDepth > 0.001f)
    {
        float totalInvMass = rba.inverseMass + rbb.inverseMass;
        float percent = 0.8f;
        glm::vec3 correctionVec = n * (maxDepth * percent / totalInvMass);
        if (!rba.fixed)
            rba.cm -= correctionVec * rba.inverseMass;
        if (!rbb.fixed)
            rbb.cm += correctionVec * rbb.inverseMass;
    }

    // Apply Impulses Iteratively (Normal + Friction)
    int iterations = 32;

    for (int k = 0; k < iterations; k++)
    {
        for (const auto &p : manifolds)
        {
            glm::vec3 r1 = p - rba.cm;
            glm::vec3 r2 = p - rbb.cm;

            // Normal impulse (from the collision as we know it from before)
            glm::vec3 v1 = rba.linearVelocity + glm::cross(rba.angularVelocity, r1);
            glm::vec3 v2 = rbb.linearVelocity + glm::cross(rbb.angularVelocity, r2);
            glm::vec3 vrel = v2 - v1;

            float vn = glm::dot(vrel, n);
            if (vn > 0.0f)
                continue;

            glm::vec3 t1 = glm::cross(rba.inertia * glm::cross(r1, n), r1);
            glm::vec3 t2 = glm::cross(rbb.inertia * glm::cross(r2, n), r2);

            float denom = rba.inverseMass + rbb.inverseMass + glm::dot(t1 + t2, n);
            float j = -(1.0f + c) * vn / denom;

            glm::vec3 impulse = n * j;

            // skip fixed bodies
            if (!rba.fixed)
            {
                rba.linearVelocity -= impulse * rba.inverseMass;
                rba.angularMomentum -= glm::cross(r1, impulse);
                rba.angularVelocity = rba.inertia * rba.angularMomentum;
            }
            if (!rbb.fixed)
            {
                rbb.linearVelocity += impulse * rbb.inverseMass;
                rbb.angularMomentum += glm::cross(r2, impulse);
                rbb.angularVelocity = rbb.inertia * rbb.angularMomentum;
            }

            // Friction
            // relative velocity changed due to impluse, we calculate again
            v1 = rba.linearVelocity + glm::cross(rba.angularVelocity, r1);
            v2 = rbb.linearVelocity + glm::cross(rbb.angularVelocity, r2);
            vrel = v2 - v1;

            // Get tangent direction (velocity along the surface), i.e. sliding dir
            glm::vec3 tangent = vrel - n * glm::dot(vrel, n);
            float tangentLen = glm::length(tangent);

            if (tangentLen > 0.75f) // lower limit
            {
                tangent /= tangentLen; // Normalize tangent

                // fricition tangents on both blocks
                glm::vec3 ft1 = glm::cross(rba.inertia * glm::cross(r1, tangent), r1);
                glm::vec3 ft2 = glm::cross(rbb.inertia * glm::cross(r2, tangent), r2);
                float fDenom = rba.inverseMass + rbb.inverseMass + glm::dot(ft1 + ft2, tangent);

                float jt = -glm::dot(vrel, tangent) / fDenom;

                // Clamp friction
                float maxJt = friction * j;
                jt = glm::clamp(jt, -maxJt, maxJt);

                glm::vec3 frictionImpulse = tangent * jt;

                // Again ignore fixed bodies
                if (!rba.fixed)
                {
                    rba.linearVelocity -= frictionImpulse * rba.inverseMass;
                    rba.angularMomentum -= glm::cross(r1, frictionImpulse);
                    rba.angularVelocity = rba.inertia * rba.angularMomentum;
                }
                if (!rbb.fixed)
                {
                    rbb.linearVelocity += frictionImpulse * rbb.inverseMass;
                    rbb.angularMomentum += glm::cross(r2, frictionImpulse);
                    rbb.angularVelocity = rbb.inertia * rbb.angularMomentum;
                }
            }
        }
    }

    return true;
}

void Body::print()
{
    printf("================ Rigidbody state: ================\n");
    printf("Is fixed?               %s\n", this->fixed ? "yes" : "no");
    printf("CM Position:            (%.3f, %.3f, %.3f)\n", this->cm.x, this->cm.y, this->cm.z);
    printf("Velocity:               (%.3f, %.3f, %.3f)\n", this->linearVelocity.x, this->linearVelocity.y, this->linearVelocity.z);
    printf("Mass:                   %.3f\n", this->mass);
    printf("Rotation:               (%.3f, %.3f, %.3f, %.3f)\n", this->orientation.w, this->orientation.x, this->orientation.y, this->orientation.z);
    printf("Rotation Matrix:\n");
    printMatrix(glm::mat3_cast(this->orientation));
    printf("Angular Momentum:       (%.3f, %.3f, %.3f)\n", this->angularMomentum.x, this->angularMomentum.y, this->angularMomentum.z);
    printf("Angular Velocity:       (%.3f, %.3f, %.3f)\n", this->angularVelocity.x, this->angularVelocity.y, this->angularVelocity.z);
    printf("Current Inverted Inertia Tensor:\n");
    printMatrix(this->inertia);
    printf("Initial Inverted Inertia Tensor:\n");
    printMatrix(this->initialInertia);
    printf("==================================================\n");
}

void Body::printPoint(glm::vec3 pos)
{
    printf("================== Point state: ==================\n");
    printf("World Position:         (%.3f, %.3f, %.3f)\n", pos.x, pos.y, pos.z);
    glm::vec3 localPos = glm::inverse(static_cast<glm::mat3>(this->orientation)) * (pos - this->cm);
    printf("Local Position:         (%.3f, %.3f, %.3f)\n", localPos.x, localPos.y, localPos.z);
    glm::vec3 velocity = this->linearVelocity + glm::cross(this->angularVelocity, localPos);
    printf("Velocity:               (%.3f, %.3f, %.3f)\n", velocity.x, velocity.y, velocity.z);
    printf("==================================================\n");
}

// Collision helpers

// Helper: Clip polygon against a plane
std::vector<glm::vec3> clip(const std::vector<glm::vec3> &vertices, glm::vec3 planePos, glm::vec3 planeNormal)
{
    std::vector<glm::vec3> result;
    if (vertices.empty())
        return result;

    for (size_t i = 0; i < vertices.size(); i++)
    {
        size_t next = (i + 1) % vertices.size();
        glm::vec3 v1 = vertices[i];
        glm::vec3 v2 = vertices[next];

        float d1 = glm::dot(v1 - planePos, planeNormal);
        float d2 = glm::dot(v2 - planePos, planeNormal);

        if (d1 >= 0)
            result.push_back(v1); // Keep points inside/on plane

        if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
        {
            float t = d1 / (d1 - d2);
            result.push_back(v1 + t * (v2 - v1));
        }
    }
    return result;
}

// Helper: Get face most perpendicular to normal
void getBestFace(Body &body, glm::vec3 normal, std::vector<glm::vec3> &outVertices)
{
    glm::mat3 rot = static_cast<glm::mat3>(body.orientation);
    glm::vec3 localNormal = glm::transpose(rot) * normal;

    glm::vec3 absN = glm::abs(localNormal);
    int axis = 0;
    if (absN.y > absN.x)
        axis = 1;
    if (absN.z > absN[axis])
        axis = 2;

    glm::vec3 c = body.extent * 0.5f;
    glm::vec3 v[4];

    // Build the face in local space based on the dominant axis
    if (axis == 0)
    {
        float sign = (localNormal.x > 0) ? 1.0f : -1.0f;
        v[0] = glm::vec3(c.x * sign, c.y, c.z);
        v[1] = glm::vec3(c.x * sign, -c.y, c.z);
        v[2] = glm::vec3(c.x * sign, -c.y, -c.z);
        v[3] = glm::vec3(c.x * sign, c.y, -c.z);
    }
    else if (axis == 1)
    {
        float sign = (localNormal.y > 0) ? 1.0f : -1.0f;
        v[0] = glm::vec3(c.x, c.y * sign, c.z);
        v[1] = glm::vec3(c.x, c.y * sign, -c.z);
        v[2] = glm::vec3(-c.x, c.y * sign, -c.z);
        v[3] = glm::vec3(-c.x, c.y * sign, c.z);
    }
    else
    {
        float sign = (localNormal.z > 0) ? 1.0f : -1.0f;
        v[0] = glm::vec3(c.x, c.y, c.z * sign);
        v[1] = glm::vec3(-c.x, c.y, c.z * sign);
        v[2] = glm::vec3(-c.x, -c.y, c.z * sign);
        v[3] = glm::vec3(c.x, -c.y, c.z * sign);
    }

    // Transform to world space
    for (int i = 0; i < 4; i++)
        outVertices.push_back(body.cm + rot * v[i]);
}