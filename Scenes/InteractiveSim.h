#pragma once
#include "Scene.h"
#include "Common.h"

static const char *schemeNames[] = {"Explicit", "Implicit"};
enum IntegrationScheme
{
    EXPLICIT,
    IMPLICIT,
};

class InteractiveSim : public Scene
{
    std::vector<std::vector<double>> field;
    std::vector<std::vector<double>> buffer;

    float dt = 0.01;
    float nu = 0.1;
    float xdomain = 1;
    int xgrid = 16;
    float ydomain = 1;
    int ygrid = 16;
    bool paused = true;
    bool oneStep = false;
    float xScale = 1;
    float yScale = 1;
    float zScale = 4;
    IntegrationScheme scheme = IMPLICIT;

    float distfactor = 5;
    float clickStrength = 5;
    glm::vec3 lastcast1;
    glm::vec3 lastcast2;
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::mat4 projMatrix = glm::mat4(1);
    glm::vec3 cameraPosition = glm::vec3(0);
    float cameraNear = 0;
    glm::vec2 windowSize = glm::vec2(0);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);
    virtual void init() override;
    /// This is where you should update the physics of the scene.
    virtual void simulateStep() override;
    /// @brief Draw the scene. Gets called every frame after simulateStep.
    ///
    /// This is where you should call the Renderer draw functions.
    virtual void onDraw(Renderer &renderer) override;
    /// @brief Define the GUI for the scene. Gets called every frame after onDraw.
    virtual void onGUI() override;
};