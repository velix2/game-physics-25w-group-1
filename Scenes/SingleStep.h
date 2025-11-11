#include "Scene.h"

class SingleStep : public Scene
{
    public:
        void init() override;
    
    private:
        void integrateStepEuler(    glm::vec3 &x0, glm::vec3 &v0, float m0, 
                                    glm::vec3 &x1, glm::vec3 &v1, float m1,
                                    float springRestLength, float stiffness, float deltaT);
        
        void integrateStepMidpoint( glm::vec3 &x0, glm::vec3 &v0, float m0, 
                                    glm::vec3 &x1, glm::vec3 &v1, float m1,
                                    float springRestLength, float stiffness, float deltaT);

        glm::vec3 calculateSpringForce( float stiffness, float springCurrentLength, float springRestLength,  
                                        glm::vec3 &x0, glm::vec3 &x1);

        void printMassPoint(const std::string &name, const glm::vec3 &x, const glm::vec3 &v);
};