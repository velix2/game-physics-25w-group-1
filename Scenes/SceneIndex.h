#include "Scene.h"
#include <map>

#include "SingleStep.h"
#include "EulerSimulation.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    // add more Scene types here
    {"Single Step", creator<SingleStep>()},
    {"Euler Simulation", creator<EulerSimulation>()},
};
