#include "Scene.h"
#include <map>

#include "SceneSingleStep.h"
#include "SceneSimulation.h"
#include "SceneCollision.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<SceneSingleStep>()},
    {"Simulation", creator<SceneSimulation>()},
    {"Collision", creator<SceneCollision>()},
    // add more Scene types here
};
