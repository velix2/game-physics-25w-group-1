#include "Scene.h"
#include <map>

#include "SingleStep.h"
#include "ExplicitSim.h"
#include "ImplicitSim.h"
#include "InteractiveSim.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<SingleStep>()},
    {"Explicit Simulation", creator<ExplicitSim>()},
    {"Implicit Simulation", creator<ImplicitSim>()},
    {"Interactive Simulation", creator<InteractiveSim>()},
    // add more Scene types here
};
