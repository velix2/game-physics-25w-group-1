#include "ExplicitSim.h"
#include <imgui.h>

void ExplicitSim::init()
{
    double xr = xdomain / xgrid;
    double yr = ydomain / ygrid;
    double xpart, ypart;
    for (size_t i = 0; i < ygrid; i++)
    {
        for (size_t j = 0; j < xgrid; j++)
        {
            xpart = xr * j - (xdomain / 2);
            ypart = yr * i - (ydomain / 2);
            field[i][j] = std::exp(-10 * (xpart * xpart + ypart * ypart));
        }
    }
    // printField(field);
}

void ExplicitSim::simulateStep()
{
    if (!paused || oneStep)
    {
        // if (applyForce)
        // {
        // }
        explicitStep(field, buffer, nu, dt, xdomain, ydomain);
        std::swap(field, buffer);
        oneStep = false;
    }
}

void ExplicitSim::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(xdomain * xgrid, ydomain * ygrid, yScale));
    float corrx = xgrid % 2 == 0 ? 0.5f : 0;
    float corry = ygrid % 2 == 0 ? 0.5f : 0;
    float val;
    auto cmap = Colormap("terrain");
    for (int i = 0; i < ygrid; i++)
    {
        for (int j = 0; j < xgrid; j++)
        {
            val = field[i][j] * yScale;
            renderer.drawCube(glm::vec3(float(j - (xgrid / 2)) + corrx, float(i - (ygrid / 2)) + corry, -(yScale / 2.0) + val / 2),
                              glm::quat(glm::vec3(0)),
                              glm::vec3(1, 1, val),
                              glm::vec4(cmap(glm::clamp(field[i][j], 0.0, 1.0)), 1.0));
        }
    }
}

void ExplicitSim::onGUI()
{
    ImGui::SliderFloat("Dt", &dt, 0, 0.1f);
    ImGui::SliderFloat("Diffusivity", &nu, 0, 1);
    ImGui::Checkbox("Paused", &paused);
    auto btnOneStep = ImGui::Button("Step");
    if (btnOneStep)
    {
        oneStep = true;
    }
}
