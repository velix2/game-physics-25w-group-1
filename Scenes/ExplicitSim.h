#pragma once
#include "Scene.h"
#include "Common.h"

class ExplicitSim : public Scene
{
    std::array<std::array<double, 16>, 16> field = {0};
    std::array<std::array<double, 16>, 16> buffer = {0};

    float dt = 0.01;
    float nu = 0.1;
    float xdomain = 1;
    int xgrid = 16;
    float ydomain = 1;
    int ygrid = 16;
    bool paused = true;
    bool oneStep = false;
    float yScale = 4;

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