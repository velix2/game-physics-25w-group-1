#pragma once
#include "Renderer.h"
#include <array>
#include <vector>
#include <stdio.h>
#include "util/pcgsolver.h"

glm::vec3 screenToWorldRay(
    const glm::mat4 &proj,
    const glm::mat4 &view,
    float mx, float my,
    float screenW, float screenH);

bool intersectRay(glm::vec3 pos, glm::vec3 scale, glm::vec3 origin, glm::vec3 direction, glm::vec3 &hitPoint, float &t);

template <int m, int n>
inline void explicitStep(std::array<std::array<double, n>, m> &current, std::array<std::array<double, n>, m> &next,
                         float nu, float dt, float xdomain, float ydomain)
{
    float dx = xdomain / (float)n;
    float dy = ydomain / (float)m;
    float dxi = 1 / (dx * dx);
    float dyi = 1 / (dy * dy);
    float xpart, ypart;
    float currentT;
    for (size_t i = 0; i < m; i++)
    {
        for (size_t j = 0; j < n; j++)
        {
            currentT = current[i][j];
            xpart = -2 * currentT;
            if (i < (m - 1))
            {
                xpart += current[i + 1][j];
            }
            if (i > 0)
            {
                xpart += current[i - 1][j];
            }
            xpart *= dxi;
            ypart = -2 * currentT;
            if (j < (n - 1))
            {
                ypart += current[i][j + 1];
            }
            if (j > 0)
            {
                ypart += current[i][j - 1];
            }
            ypart *= dyi;
            next[i][j] = dt * nu * (xpart + ypart) + currentT;
        }
    }
}

inline void explicitStep(std::vector<std::vector<double>> &current, std::vector<std::vector<double>> &next,
                         int m, int n, float nu, float dt, float xdomain, float ydomain)
{
    float dx = xdomain / (float)n;
    float dy = ydomain / (float)m;
    float dxi = 1 / (dx * dx);
    float dyi = 1 / (dy * dy);
    float xpart, ypart;
    float currentT;
    for (size_t i = 0; i < m; i++)
    {
        for (size_t j = 0; j < n; j++)
        {
            currentT = current[i][j];
            xpart = -2 * currentT;
            if (i < (m - 1))
            {
                xpart += current[i + 1][j];
            }
            if (i > 0)
            {
                xpart += current[i - 1][j];
            }
            xpart *= dxi;
            ypart = -2 * currentT;
            if (j < (n - 1))
            {
                ypart += current[i][j + 1];
            }
            if (j > 0)
            {
                ypart += current[i][j - 1];
            }
            ypart *= dyi;
            next[i][j] = dt * nu * (xpart + ypart) + currentT;
        }
    }
}

template <int m, int n>
inline void implicitStep(std::array<std::array<double, n>, m> &current, std::array<std::array<double, n>, m> &next,
                         float nu, float dt, float xdomain, float ydomain)
{
    float sign = -1;
    float dx = xdomain / (float)n;
    float dy = ydomain / (float)m;
    float dxi = 1 / (dx * dx);
    float dyi = 1 / (dy * dy);
    std::vector<float> b;
    for (int j = 0; j < n; j++)
    {
        for (int i = 0; i < m; i++)
        {
            b.push_back(-sign * current[i][j]);
        }
    }
    float diagval = sign * ((-2 * dt * nu * dxi) + (-2 * dt * nu * dyi) - 1);
    float xval = sign * dt * nu * dxi;
    float yval = sign * dt * nu * dyi;
    SparseMatrix<float> A(n * m, 5);
    for (int j = 0; j < n; j++)
    {
        for (int i = 0; i < m; i++)
        {
            int curIdx = j * m + i;
            A.set_element(curIdx, curIdx, diagval);
            if (i < (m - 1))
            {
                A.set_element(curIdx, curIdx + 1, xval);
            }
            if (i > 0)
            {
                A.set_element(curIdx, curIdx - 1, xval);
            }
            if (j < (n - 1))
            {
                A.set_element(curIdx, curIdx + m, yval);
            }
            if (j > 0)
            {
                A.set_element(curIdx, curIdx - m, yval);
            }
        }
    }
    SparsePCGSolver<float> solver;
    std::vector<float> x;
    for (size_t i = 0; i < m * n; i++)
    {
        x.push_back(0);
    }

    float relativeResidualsOut;
    int iterationsOut;
    solver.solve(A, b, x, relativeResidualsOut, iterationsOut);
    int i = 0, j = 0;
    for (auto &&val : x)
    {
        next[i][j] = val;
        i = (i + 1) % m;
        if (i == 0)
        {
            j++;
        }
    }
}
inline void implicitStep(std::vector<std::vector<double>> &current, std::vector<std::vector<double>> &next,
                         int m, int n, float nu, float dt, float xdomain, float ydomain)
{
    float sign = -1;
    float dx = xdomain / (float)n;
    float dy = ydomain / (float)m;
    float dxi = 1 / (dx * dx);
    float dyi = 1 / (dy * dy);
    std::vector<float> b;
    for (int j = 0; j < n; j++)
    {
        for (int i = 0; i < m; i++)
        {
            b.push_back(-sign * current[i][j]);
        }
    }
    float diagval = sign * ((-2 * dt * nu * dxi) + (-2 * dt * nu * dyi) - 1);
    float xval = sign * dt * nu * dxi;
    float yval = sign * dt * nu * dyi;
    SparseMatrix<float> A(n * m, 5);
    for (int j = 0; j < n; j++)
    {
        for (int i = 0; i < m; i++)
        {
            int curIdx = j * m + i;
            A.set_element(curIdx, curIdx, diagval);
            if (i < (m - 1))
            {
                A.set_element(curIdx, curIdx + 1, xval);
            }
            if (i > 0)
            {
                A.set_element(curIdx, curIdx - 1, xval);
            }
            if (j < (n - 1))
            {
                A.set_element(curIdx, curIdx + m, yval);
            }
            if (j > 0)
            {
                A.set_element(curIdx, curIdx - m, yval);
            }
        }
    }
    SparsePCGSolver<float> solver;
    std::vector<float> x = std::vector<float>(m * n);
    // for (size_t i = 0; i < m * n; i++)
    // {
    //     x.push_back(0);
    // }

    float relativeResidualsOut;
    int iterationsOut;
    solver.solve(A, b, x, relativeResidualsOut, iterationsOut);
    int i = 0, j = 0;
    for (auto &&val : x)
    {
        next[i][j] = val;
        i = (i + 1) % m;
        if (i == 0)
        {
            j++;
        }
    }
}
template <int m, int n>
inline void printField(std::array<std::array<double, n>, m> field)
{
    printf("=====");
    for (size_t i = 0; i < n; i++)
    {
        printf("=======");
    }
    printf("\n");
    printf("i,j  ");
    for (int i = 0; i < n; i++)
    {
        printf(" %-2d    ", i);
    }
    printf("\n");
    printf("-----");
    for (size_t i = 0; i < n; i++)
    {
        printf("-------");
    }
    printf("\n");
    for (int i = 0; i < m; i++)
    {
        printf("%-2d   ", i);
        for (int j = 0; j < n; j++)
        {
            printf("% 5.2f  ", field[i][j]);
        }
        printf("\n");
    }
    printf("=====");
    for (size_t i = 0; i < n; i++)
    {
        printf("=======");
    }
    printf("\n");
}