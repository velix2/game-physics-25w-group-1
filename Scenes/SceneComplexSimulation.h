#include "Scene.h"

class SceneComplexSimulation : public Scene
{
    virtual void init() override;

    // Math

    float delta_t = 0.005f;

    float gravity_accel = 9.81f;

    virtual void simulateStep();

    typedef struct masspoint
    {
        uint id;
        glm::vec3 position;
        glm::vec3 velocity;
        float mass;
    } masspoint_t;

    typedef struct spring
    {
        float stiffness;
        float rest_length;
        masspoint_t &p1;
        masspoint_t &p2;
    } spring_t;

    typedef struct collision_plane 
    {
        glm::vec3 surfaceNormal;
        float offsetAlongNormal;
    } collision_plane_t;

    std::vector<masspoint_t> masspoints;
    std::vector<spring_t> springs;
    std::vector<collision_plane_t> collision_planes;

    void handleCollision(collision_plane_t &plane, masspoint_t &point);

    void calculateElasticForcesWithGravity(spring_t &spring, std::vector<glm::vec3> &masspointForces);

    void calculateElasticForcesAtMidpointWithGravity(spring_t &spring, glm::vec3 p1_midpoint_position, glm::vec3 p2_midpoint_position, std::vector<glm::vec3> &masspointForces);

    glm::vec3 calculateAcceleration(glm::vec3 force, float mass);

    glm::vec3 eulerStep(glm::vec3 x_n, glm::vec3 xPrime_n, float h);

    void runSimulationStep(float stepsize);
    
    void runSimulationStepWithMidpoint(float stepsize);

    void runSimulationStepWithEuler(float stepsize);

    // Visuals

    const float POINT_RADIUS = 0.25f;
    const glm::vec4 POINT_COLOR = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    const glm::vec4 SPRING_COLOR = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);

    bool isSimulationRunning = false;
    bool isUsingMidpoint = false;

    virtual void onDraw(Renderer &renderer) override;

    virtual void onGUI() override;
};