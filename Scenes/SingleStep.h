#pragma once
#include "Scene.h"
#include "RigidBody.h"

class SingleStep : public Scene {
    RigidBody body;
    
public:
    virtual void init() override;
    virtual void onDraw(Renderer& renderer) override;
};

