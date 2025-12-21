#include "InteractiveSim.h"
#include <imgui.h>

void InteractiveSim::init()
{
    double xr = xdomain / xgrid;
    double yr = ydomain / ygrid;
    double xpart, ypart;
    field.clear();
    buffer.clear();
    for (size_t i = 0; i < ygrid; i++)
    {
        auto fvec = std::vector<double>();
        auto bvec = std::vector<double>();
        for (size_t j = 0; j < xgrid; j++)
        {
            xpart = xr * j - (xdomain / 2);
            ypart = yr * i - (ydomain / 2);
            fvec.push_back(glm::clamp(glm::exp(-distfactor * (xpart * xpart + ypart * ypart)), 0.0, 1.0));
            bvec.push_back(0);
        }
        field.push_back(fvec);
        buffer.push_back(bvec);
    }
    // printField(field);
}

void InteractiveSim::simulateStep()
{
    if (!paused || oneStep)
    {
        // if (applyForce)
        // {
        // }
        switch (scheme)
        {
        case IMPLICIT:
            implicitStep(field, buffer, ygrid, xgrid, nu, dt, xdomain, ydomain);
            break;
        case EXPLICIT:
            explicitStep(field, buffer, ygrid, xgrid, nu, dt, xdomain, ydomain);
            break;

        default:
            break;
        }

        std::swap(field, buffer);
        oneStep = false;
    }
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right))
    {
        auto pos = ImGui::GetMousePos();
        // printf("heyi");
        //  auto size = ImGui::GetWindowSize();
        // printf("heyo");
        // glm::vec3 rel = -glm::normalize((pos.x / windowSize.x - 0.5f) * right - (pos.y / windowSize.y - 0.5f) * up + cameraNear * fwd);
        glm::vec3 hitPoint;
        glm::vec3 rel = screenToWorldRay(projMatrix, cameraMatrix, pos.x, pos.y, windowSize.x, windowSize.y);
        float nearest = -10;
        int nearest_i = 0, nearest_j = 0;
        int i = 0;
        float corrx = xgrid % 2 == 0 ? xScale / 2 : 0;
        float corry = ygrid % 2 == 0 ? yScale / 2 : 0;
        for (auto &&row : field)
        {
            int j = 0;
            for (auto &&fval : row)
            {
                float val = fval * zScale;
                float t;
                glm::vec3 pos = glm::vec3(float(j - (xgrid / 2)) * xScale + corrx, float(i - (ygrid / 2)) * yScale + corry, -(zScale / 2.0) + val / 2);
                glm::vec3 scale = glm::vec3(xScale, yScale, val);
                if (intersectRay(pos, scale, cameraPosition, rel, hitPoint, t))
                {
                    if (nearest < -5 || t < nearest)
                    {
                        // printf("Got at t=%f, i=%d, j=%d\n", t, i, j);
                        nearest = t;
                        nearest_i = i;
                        nearest_j = j;
                        lastcast1 = hitPoint;
                        lastcast2 = hitPoint + rel;
                    }
                }
                j++;
            }
            i++;
        }
        // printf("\n");
        if (nearest >= 0)
        {
            field[nearest_i][nearest_j] += dt * clickStrength;
        }

        // printf("t %f, %f, %f\n", rel.x, rel.y, rel.z);
        // printf("r %f, %f, %f\n", fwd.x, fwd.y, fwd.z);
        // printf("");
    }
}

void InteractiveSim::onDraw(Renderer &renderer)
{
    renderer.drawLine(lastcast1, lastcast2, glm::vec4(1, 1, 0, 1));
    renderer.drawWireCube(glm::vec3(0), glm::vec3(xScale * xgrid, yScale * ygrid, zScale));
    float corrx = xgrid % 2 == 0 ? xScale / 2 : 0;
    float corry = ygrid % 2 == 0 ? yScale / 2 : 0;
    float val;
    auto cmap = Colormap("terrain");
    for (int i = 0; i < ygrid; i++)
    {
        for (int j = 0; j < xgrid; j++)
        {
            val = field[i][j] * zScale;
            renderer.drawCube(glm::vec3(float(j - (xgrid / 2)) * xScale + corrx, float(i - (ygrid / 2)) * yScale + corry, -(zScale / 2.0) + val / 2),
                              glm::quat(glm::vec3(0)),
                              glm::vec3(xScale, yScale, val),
                              glm::vec4(cmap(glm::clamp(field[i][j], 0.0, 1.0)), 1.0));
        }
    }
    projMatrix = renderer.camera.projectionMatrix();
    cameraMatrix = renderer.camera.viewMatrix;
    cameraPosition = renderer.camera.position;
    // cameraNear = renderer.camera.near;
    windowSize = glm::vec2(renderer.camera.width, renderer.camera.height);
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, -1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);
}

void InteractiveSim::onGUI()
{
    ImGui::SliderFloat("Dt", &dt, 0, 0.1f);
    ImGui::SliderFloat("Diffusivity", &nu, 0, 1);
    ImGui::Checkbox("Paused", &paused);
    auto btnOneStep = ImGui::Button("Step");
    if (btnOneStep)
    {
        oneStep = true;
    }
    int current = static_cast<int>(scheme);
    if (ImGui::Combo("Integration Scheme", &current, schemeNames, IM_ARRAYSIZE(schemeNames)))
    {
        scheme = static_cast<IntegrationScheme>(current);
    }
    if (ImGui::SliderInt("M", &ygrid, 1, 32))
    {
        init();
    }
    if (ImGui::SliderInt("N", &xgrid, 1, 32))
    {
        init();
    }
    if (ImGui::SliderFloat("Y domain", &ydomain, 0.1, 50))
    {
        init();
    }
    if (ImGui::SliderFloat("X domain", &xdomain, 0.1, 50))
    {
        init();
    }
    if (ImGui::SliderFloat("Distribution factor", &distfactor, -20, 20))
    {
        init();
    }
    ImGui::SliderFloat("Click Strength", &clickStrength, 0.1, 100);
    ImGui::Text("Right click a column to add temperature");
}
