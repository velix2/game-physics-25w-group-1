#include "Scene.h"
#include "SingleStep.h"
#include "Simulation.h"
#include "Collision.h"
#include "Complex.h"
#include <map>

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<SingleStep>()},
    {"Simulation", creator<Simulation>()},
    {"Collision", creator<Collision>()},
    {"Complex", creator<Complex>()},
    // add more Scene types here
};
