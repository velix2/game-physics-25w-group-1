#include "InteractiveSimulation.h"
#include <imgui.h>
#include <cmath>

void InteractiveSimulation::init()
{
    resizeGrid();
    initZero();
    stepCount = 0;
    running = false;
}

void InteractiveSimulation::resizeGrid()
{
    T.resize(m * n, 0.0f);
}

void InteractiveSimulation::initZero()
{
    T.assign(m * n, 0.0f);
}

void InteractiveSimulation::initRandom()
{
    T.resize(m * n);
    for (int i = 0; i < m * n; ++i)
        T[i] = dis(gen);
}

void InteractiveSimulation::initGaussian()
{
    T.resize(m * n);
    float cx = domainWidth * 0.5f;
    float cy = domainHeight * 0.5f;
    float sigma = std::min(domainWidth, domainHeight) * 0.1f;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            float x = (i + 1) * dx();
            float y = (j + 1) * dy();
            float r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            T[i * n + j] = std::exp(-r2 / (2.0f * sigma * sigma));
        }
    }
}

void InteractiveSimulation::initSineWave()
{
    T.resize(m * n);
    const float pi = 3.14159265f;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            float x = (i + 1) * dx() / domainWidth;
            float y = (j + 1) * dy() / domainHeight;
            T[i * n + j] = std::sin(pi * x) * std::sin(pi * y);
        }
    }
}

void InteractiveSimulation::paintAtPosition(float normX, float normY)
{
    // normX, normY are in [0, 1] range for the image area
    // grid indexing: T[i * n + j] where i is row (y), j is column (x)
    // image: m rows (height), n columns (width)
    // so normX -> j direction, normY -> i direction

    float brushPhysRadius = brushRadius * std::max(domainWidth, domainHeight);

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            // cell position in normalized coords [0,1]
            float cellNormX = (float)(j + 1) / (n + 1);
            float cellNormY = (float)(i + 1) / (m + 1);

            float distX = cellNormX - normX;
            float distY = cellNormY - normY;
            float dist = std::sqrt(distX * distX + distY * distY);

            if (dist < brushRadius)
            {
                float falloff = 1.0f - (dist / brushRadius);
                T[i * n + j] = brushTemperature * falloff + T[i * n + j] * (1.0f - falloff);
            }
        }
    }
}

void InteractiveSimulation::explicitStep()
{
    std::vector<float> Tnew(m * n);
    float dx2 = dx() * dx();
    float dy2 = dy() * dy();

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            float Tij = getT(i, j);
            float lap_x = (getT(i + 1, j) - 2.0f * Tij + getT(i - 1, j)) / dx2;
            float lap_y = (getT(i, j + 1) - 2.0f * Tij + getT(i, j - 1)) / dy2;
            Tnew[i * n + j] = Tij + nu * dt * (lap_x + lap_y);
        }
    }
    T = Tnew;
    stepCount++;
}

void InteractiveSimulation::implicitStep()
{
    int N = m * n;
    double dx2 = dx() * dx();
    double dy2 = dy() * dy();
    double alpha_x = nu * dt / dx2;
    double alpha_y = nu * dt / dy2;
    double diag = 1.0 + 2.0 * alpha_x + 2.0 * alpha_y;

    SparseMatrix<double> A(N);
    std::vector<double> b(N);
    std::vector<double> x(N, 0.0);

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            int k = idx(i, j);
            b[k] = T[k];
            A.set_element(k, k, diag);

            if (i + 1 < m)
                A.set_element(k, idx(i + 1, j), -alpha_x);
            if (i - 1 >= 0)
                A.set_element(k, idx(i - 1, j), -alpha_x);
            if (j + 1 < n)
                A.set_element(k, idx(i, j + 1), -alpha_y);
            if (j - 1 >= 0)
                A.set_element(k, idx(i, j - 1), -alpha_y);
        }
    }

    double residual;
    int iterations;
    solver.solve(A, b, x, residual, iterations);

    lastIterations = iterations;
    lastResidual = residual;

    for (int k = 0; k < N; ++k)
        T[k] = static_cast<float>(x[k]);

    stepCount++;
}

void InteractiveSimulation::simulateStep()
{
    // handle mouse painting with left click
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse)
    {
        ImVec2 mousePos = ImGui::GetMousePos();
        ImVec2 displaySize = ImGui::GetIO().DisplaySize;

        // image is fullscreen, mouse (0,0) is top-left
        // normX: left=0, right=1
        // normY: top=0, bottom=1 (matching row indexing where row 0 is at top)
        float normX = mousePos.x / displaySize.x;
        float normY = mousePos.y / displaySize.y;

        paintAtPosition(normX, normY);
        painting = true;
    }
    else
    {
        painting = false;
    }

    if (running)
    {
        if (useImplicit)
            implicitStep();
        else
            explicitStep();
    }
}

void InteractiveSimulation::onDraw(Renderer& renderer)
{
    if (!T.empty())
    {
        renderer.drawImage(T, m, n, Colormap("coolwarm"), {0.0f, 0.0f}, {1.0f, 1.0f});
    }
}

void InteractiveSimulation::onGUI()
{
    ImGui::Text("Interactive Heat Simulation");
    ImGui::Separator();

    // simulation controls
    if (ImGui::Button(running ? "Pause" : "Start"))
        running = !running;

    ImGui::SameLine();
    if (ImGui::Button("Step"))
    {
        if (useImplicit)
            implicitStep();
        else
            explicitStep();
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset"))
        init();

    ImGui::Text("Step: %d", stepCount);
    ImGui::Separator();

    // scheme selection
    ImGui::Text("Integration Scheme:");
    if (ImGui::RadioButton("Explicit (FTCS)", !useImplicit))
        useImplicit = false;
    ImGui::SameLine();
    if (ImGui::RadioButton("Implicit (BTCS)", useImplicit))
        useImplicit = true;

    ImGui::Separator();

    // parameters
    ImGui::SliderFloat("dt", &dt, 0.0001f, 0.5f, "%.4f");
    ImGui::SliderFloat("nu (diffusivity)", &nu, 0.001f, 1.0f, "%.3f");

    // stability indicator for explicit
    if (!useImplicit)
    {
        float cfl = nu * dt * (1.0f / (dx() * dx()) + 1.0f / (dy() * dy()));
        ImGui::Text("CFL: %.4f (should be < 0.5)", cfl);
        if (cfl > 0.5f)
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "Warning: Unstable!");
    }
    else
    {
        ImGui::Text("Implicit - unconditionally stable");
        ImGui::Text("PCG iters: %d, residual: %.2e", lastIterations, lastResidual);
    }

    ImGui::Separator();

    // grid and domain settings
    ImGui::Text("Grid & Domain:");
    int newM = m, newN = n;
    bool gridChanged = false;

    if (ImGui::SliderInt("m (x resolution)", &newM, 4, 128))
        gridChanged = true;
    if (ImGui::SliderInt("n (y resolution)", &newN, 4, 128))
        gridChanged = true;

    if (gridChanged && (newM != m || newN != n))
    {
        m = newM;
        n = newN;
        resizeGrid();
        initZero();
        stepCount = 0;
    }

    ImGui::SliderFloat("Domain Width", &domainWidth, 0.1f, 10.0f);
    ImGui::SliderFloat("Domain Height", &domainHeight, 0.1f, 10.0f);

    ImGui::Text("dx = %.4f, dy = %.4f", dx(), dy());

    ImGui::Separator();

    // brush settings
    ImGui::Text("Mouse Brush (Left Click to Paint):");
    ImGui::SliderFloat("Brush Temp", &brushTemperature, -2.0f, 2.0f);
    ImGui::SliderFloat("Brush Radius", &brushRadius, 0.01f, 0.2f);
    if (painting)
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Painting...");

    ImGui::Separator();

    // initialization options
    ImGui::Text("Initialize:");
    if (ImGui::Button("Zero"))
    {
        initZero();
        stepCount = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Random"))
    {
        initRandom();
        stepCount = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Gaussian"))
    {
        initGaussian();
        stepCount = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Sine"))
    {
        initSineWave();
        stepCount = 0;
    }
}

