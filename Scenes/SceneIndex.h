#include "Scene.h"
#include <map>

#include "Collision.h"
#include "Complex.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Collision", creator<Collision>()},
    {"Complex", creator<Complex>()},
    // add more Scene types here
};
