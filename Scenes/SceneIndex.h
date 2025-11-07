#include "Scene.h"
#include <map>

#include "SceneSingleStep.h"
#include "SceneEuler.h"
#include "SceneMidpoint.h"
#include "SceneComplexSimulation.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<SceneSingleStep>()},
    {"Euler Simulation", creator<SceneEuler>()},
    {"Midpoint Simulation", creator<SceneMidpoint>()},
    {"Complex Simulation", creator<SceneComplexSimulation>()},
    // add more Scene types here
};
