#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SceneEuler.h"
#include "SceneMidpoint.h"
#include "SceneComplex.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<Scene1>()},
    {"Euler Simulation", creator<SceneEuler>()},
    {"Midpoint Simulation", creator<SceneMidpoint>()},
    {"Complex Simulation", creator<SceneComplex>()},
    // add more Scene types here
};
