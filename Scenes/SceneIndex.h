#include "Scene.h"
#include <map>

#include "SceneSingleStep.h"
#include "SceneExplicit.h"
#include "SceneImplicit.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<SceneSingleStep>()},
    {"Explicit Simluation", creator<SceneExplicit>()},
    {"Implicit Simluation", creator<SceneImplicit>()},
    // add more Scene types here
};
