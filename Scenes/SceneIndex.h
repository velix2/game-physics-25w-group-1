#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "Scene2.h"
#include "Scene3.h"
#include "Scene4.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<Scene1>()},
    {"Euler Simulation", creator<Scene2>()},
    {"Midpoint Simulation", creator<Scene3>()},
    {"Complex Simulation", creator<Scene4>()},
    // add more Scene types here
};
