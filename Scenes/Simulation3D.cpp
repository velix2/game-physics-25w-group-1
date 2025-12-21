#include "Simulation3D.h"
#include <imgui.h>
#include <cmath>

void Simulation3D::init()
{
    T.resize(m * n * p);
    initGaussian();
    stepCount = 0;
    running = false;
    sliceIndex = p / 2;
}

void Simulation3D::initZero()
{
    T.assign(m * n * p, 0.0f);
}

void Simulation3D::initRandom()
{
    for (int i = 0; i < m * n * p; ++i)
        T[i] = dis(gen);
}

void Simulation3D::initGaussian()
{
    float cx = domainX * 0.5f;
    float cy = domainY * 0.5f;
    float cz = domainZ * 0.5f;
    float sigma = std::min({domainX, domainY, domainZ}) * 0.15f;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            for (int k = 0; k < p; ++k)
            {
                float x = (i + 1) * dx();
                float y = (j + 1) * dy();
                float z = (k + 1) * dz();
                float r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy) + (z - cz) * (z - cz);
                T[idx(i, j, k)] = std::exp(-r2 / (2.0f * sigma * sigma));
            }
        }
    }
}

void Simulation3D::explicitStep()
{
    std::vector<float> Tnew(m * n * p);
    float dx2 = dx() * dx();
    float dy2 = dy() * dy();
    float dz2 = dz() * dz();

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            for (int k = 0; k < p; ++k)
            {
                float Tijk = getT(i, j, k);
                float lap_x = (getT(i + 1, j, k) - 2.0f * Tijk + getT(i - 1, j, k)) / dx2;
                float lap_y = (getT(i, j + 1, k) - 2.0f * Tijk + getT(i, j - 1, k)) / dy2;
                float lap_z = (getT(i, j, k + 1) - 2.0f * Tijk + getT(i, j, k - 1)) / dz2;
                Tnew[idx(i, j, k)] = Tijk + nu * dt * (lap_x + lap_y + lap_z);
            }
        }
    }
    T = Tnew;
    stepCount++;
}

void Simulation3D::implicitStep()
{
    int N = m * n * p;
    double dx2 = dx() * dx();
    double dy2 = dy() * dy();
    double dz2 = dz() * dz();
    double alpha_x = nu * dt / dx2;
    double alpha_y = nu * dt / dy2;
    double alpha_z = nu * dt / dz2;
    double diag = 1.0 + 2.0 * (alpha_x + alpha_y + alpha_z);

    SparseMatrix<double> A(N);
    std::vector<double> b(N);
    std::vector<double> x(N, 0.0);

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            for (int k = 0; k < p; ++k)
            {
                int id = idx(i, j, k);
                b[id] = T[id];
                A.set_element(id, id, diag);

                if (i + 1 < m) A.set_element(id, idx(i + 1, j, k), -alpha_x);
                if (i - 1 >= 0) A.set_element(id, idx(i - 1, j, k), -alpha_x);
                if (j + 1 < n) A.set_element(id, idx(i, j + 1, k), -alpha_y);
                if (j - 1 >= 0) A.set_element(id, idx(i, j - 1, k), -alpha_y);
                if (k + 1 < p) A.set_element(id, idx(i, j, k + 1), -alpha_z);
                if (k - 1 >= 0) A.set_element(id, idx(i, j, k - 1), -alpha_z);
            }
        }
    }

    double residual;
    int iterations;
    solver.solve(A, b, x, residual, iterations);

    lastIterations = iterations;
    lastResidual = residual;

    for (int id = 0; id < N; ++id)
        T[id] = static_cast<float>(x[id]);

    stepCount++;
}

void Simulation3D::simulateStep()
{
    if (running)
    {
        if (useImplicit)
            implicitStep();
        else
            explicitStep();
    }
}

void Simulation3D::onDraw(Renderer& renderer)
{
    // draw bounding box (z is up in this framework)
    glm::vec3 center(domainX * 0.5f, domainY * 0.5f, domainZ * 0.5f + heightScale * 0.5f);
    renderer.drawWireCube(center, glm::vec3(domainX, domainY, domainZ + heightScale), glm::vec3(0.5f));

    Colormap cmap("coolwarm");

    if (showVolume)
    {
        // volume rendering - show all points above threshold with height displacement
        for (int i = 0; i < m; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                for (int k = 0; k < p; ++k)
                {
                    float temp = T[idx(i, j, k)];
                    if (std::abs(temp) > tempThreshold)
                    {
                        float x = (i + 1) * dx();
                        float y = (j + 1) * dy();
                        float z = (k + 1) * dz() + temp * heightScale; // displace z (up) by temperature
                        float normTemp = (temp + 1.0f) * 0.5f;
                        glm::vec3 col = cmap(normTemp);
                        float alpha = std::min(1.0f, std::abs(temp));
                        renderer.drawSphere(glm::vec3(x, y, z), sphereRadius, glm::vec4(col, alpha));
                    }
                }
            }
        }
    }
    else
    {
        // slice view with height displacement - looks like a surface plot
        for (int a = 0; a < (sliceAxis == 0 ? n : m); ++a)
        {
            for (int b = 0; b < (sliceAxis == 2 ? n : p); ++b)
            {
                int i, j, k;
                if (sliceAxis == 0) { i = sliceIndex; j = a; k = b; }
                else if (sliceAxis == 1) { i = a; j = sliceIndex; k = b; }
                else { i = a; j = b; k = sliceIndex; }

                if (i < 0 || i >= m || j < 0 || j >= n || k < 0 || k >= p)
                    continue;

                float temp = T[idx(i, j, k)];
                float x = (i + 1) * dx();
                float y = (j + 1) * dy();
                float z = (k + 1) * dz() + temp * heightScale; // displace z (up) by temperature
                float normTemp = (temp + 1.0f) * 0.5f;
                glm::vec3 col = cmap(normTemp);
                renderer.drawSphere(glm::vec3(x, y, z), sphereRadius * 1.5f, glm::vec4(col, 1.0f));
            }
        }
    }
}

void Simulation3D::onGUI()
{
    ImGui::Text("3D Heat Simulation");
    ImGui::Separator();

    if (ImGui::Button(running ? "Pause" : "Start"))
        running = !running;

    ImGui::SameLine();
    if (ImGui::Button("Step"))
    {
        if (useImplicit) implicitStep();
        else explicitStep();
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset"))
        init();

    ImGui::Text("Step: %d", stepCount);
    ImGui::Separator();

    // scheme
    if (ImGui::RadioButton("Explicit", !useImplicit)) useImplicit = false;
    ImGui::SameLine();
    if (ImGui::RadioButton("Implicit", useImplicit)) useImplicit = true;

    ImGui::Separator();

    ImGui::SliderFloat("dt", &dt, 0.0001f, 0.1f, "%.4f");
    ImGui::SliderFloat("nu", &nu, 0.001f, 0.1f, "%.4f");

    if (!useImplicit)
    {
        float cfl = nu * dt * (1.0f / (dx() * dx()) + 1.0f / (dy() * dy()) + 1.0f / (dz() * dz()));
        ImGui::Text("CFL: %.4f (< 0.5)", cfl);
        if (cfl > 0.5f) ImGui::TextColored(ImVec4(1, 0, 0, 1), "Unstable!");
    }
    else
    {
        ImGui::Text("PCG iters: %d, res: %.2e", lastIterations, lastResidual);
    }

    ImGui::Separator();
    ImGui::Text("Grid: %d x %d x %d = %d", m, n, p, m * n * p);

    ImGui::Separator();
    ImGui::Text("Visualization:");
    ImGui::Checkbox("Volume View", &showVolume);

    if (showVolume)
    {
        ImGui::SliderFloat("Threshold", &tempThreshold, 0.01f, 1.0f);
    }
    else
    {
        const char* axisNames[] = {"X", "Y", "Z"};
        ImGui::Combo("Slice Axis", &sliceAxis, axisNames, 3);
        int maxSlice = (sliceAxis == 0 ? m : (sliceAxis == 1 ? n : p)) - 1;
        ImGui::SliderInt("Slice Index", &sliceIndex, 0, maxSlice);
    }

    ImGui::SliderFloat("Sphere Size", &sphereRadius, 0.01f, 0.1f);
    ImGui::SliderFloat("Height Scale", &heightScale, 0.0f, 2.0f);

    ImGui::Separator();
    ImGui::Text("Initialize:");
    if (ImGui::Button("Zero")) { initZero(); stepCount = 0; }
    ImGui::SameLine();
    if (ImGui::Button("Random")) { initRandom(); stepCount = 0; }
    ImGui::SameLine();
    if (ImGui::Button("Gaussian")) { initGaussian(); stepCount = 0; }
}

