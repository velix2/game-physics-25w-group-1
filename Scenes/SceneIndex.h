#include "Scene.h"
#include <map>

#include "SingleStep.h"
#include "SingleBody.h"
#include "Collision.h"
#include "SceneComplex.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<SingleStep>()},
    {"Simulation", creator<SingleBody>()},
    {"Collision", creator<Collision>()},
    {"Complex", creator<SceneComplex>()},
    // add more Scene types here
};
