#include "Scene.h"

class MidpointSimulation : public Scene
{
    public:
        void init() override;
        void simulateStep() override;
        void onDraw(Renderer &renderer) override;
        void onGUI() override;
    
    private:
        glm::vec3 x0, v0;
        glm::vec3 x1, v1;

        float mass = 10.0f;
        float springRestLength = 1.0f;
        float stiffness = 40.0f;
        float deltaT = 0.005f;

        glm::vec3 calculateSpringForce( float stiffness, float springCurrentLength, float springRestLength,  
                                        glm::vec3 &x0, glm::vec3 &x1);
};