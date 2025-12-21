#include "Scene.h"
#include "SingleStep.h"
#include "ExplicitSimulation.h"
#include "ImplicitSimulation.h"
#include "InteractiveSimulation.h"
#include "Simulation3D.h"
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
    {"Explicit Simulation", creator<ExplicitSimulation>()},
    {"Implicit Simulation", creator<ImplicitSimulation>()},
    {"Interactive Simulation", creator<InteractiveSimulation>()},
    {"3D Simulation", creator<Simulation3D>()},
    // add more Scene types here
};
