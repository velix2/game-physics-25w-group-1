#include "SceneEuler.h"
#include <imgui.h>

void SceneEuler::simulateStep()
{
    p1.force = glm::vec3(0);
    p2.force = glm::vec3(0);
    s.computeElasticForces(dt);
    p1.integrateEuler(dt);
    p2.integrateEuler(dt);
}

void SceneEuler::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    renderer.drawLine(p1.position, p2.position, glm::vec4(1));
    renderer.drawSphere(p1.position, 0.1);
    renderer.drawSphere(p2.position, 0.1);
}

void SceneEuler::onGUI()
{
    ImGui::SliderFloat("Dt", &dt, 0, 0.1f);
}