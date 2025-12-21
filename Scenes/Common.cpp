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

float max(float a, float b)
{
    return a > b ? a : b;
}

float min(float a, float b)
{
    return a < b ? a : b;
}

// https://gamedev.stackexchange.com/a/18459
bool intersectRay(glm::vec3 pos, glm::vec3 scale, glm::vec3 origin, glm::vec3 direction, glm::vec3 &hitPoint, float &t)
{
    glm::vec3 rorg = (origin - pos);
    glm::vec3 localDirection = direction;
    glm::vec3 lb = -scale / 2.0f;
    glm::vec3 rt = scale / 2.0f;
    glm::vec3 dirfrac = 1.0f / localDirection;
    float t1 = (lb.x - rorg.x) * dirfrac.x;
    float t2 = (rt.x - rorg.x) * dirfrac.x;
    float t3 = (lb.y - rorg.y) * dirfrac.y;
    float t4 = (rt.y - rorg.y) * dirfrac.y;
    float t5 = (lb.z - rorg.z) * dirfrac.z;
    float t6 = (rt.z - rorg.z) * dirfrac.z;
    float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
    // float t;
    //  if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
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
    hitPoint = pos + hitPoint;
    return true;
}
