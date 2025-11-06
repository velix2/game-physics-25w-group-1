#include "Scene.h"

class SceneSingleStep : public Scene
{
    struct mss_setup
    {
        glm::vec3 x0;
        glm::vec3 x1;
        glm::vec3 v0;
        glm::vec3 v1;
        float k;
        float L;
        float m0;
        float m1;
        int steps;
        float delta_t;
    };

    struct mss_setup mss_data;

    inline glm::vec3 eulerStep(glm::vec3 x_n, glm::vec3 xPrime_n, float h);

    glm::vec3 calculateForce_ab(float k, float L, glm::vec3 x_a, glm::vec3 x_b);

    inline glm::vec3 calculateAcceleration(glm::vec3 F, float m);

    void mssWithEuler();

    virtual void onGUI() override;

    public:
        SceneSingleStep() : mss_data(
            {
                    // x0
                    { 0.0f, 0.0f, 0.0f }, 
                    // x1
                    { 0.0f, 2.0f, 0.0f }, 
                    // v0
                    { -1.0f, 0.0f, 0.0f }, 
                    // v1
                    { 1.0f, 0.0f, 0.0f }, 
                    // k
                    40.0f, 
                    // L
                    1.0f, 
                    // m0
                    10.0f, 
                    // m1
                    10.0f, 
                    // steps
                    2, 
                    // delta_t                    
                    0.1f  
                }
        ) {}
};