#pragma once
#include "Cala/Utility/BaseApplication.h"
#include "ModelOptimizer.h"
#include "Cala/Rendering/Renderers/LightRenderer.h"
#include "Cala/Rendering/Renderers/SimpleRenderer.h"
#include "Cala/Rendering/Mesh.h"
#include <array>

using namespace Cala;

class MyApplication : public Cala::BaseApplication {
public:
    MyApplication();
    void loop() override;

private:
    ModelOptimizer modelOptimizer;
    LightRenderer lightRenderer;
    SimpleRenderer simpleRenderer;
    Mesh sphereMesh;
    SimpleRenderer::Renderable lightRenderable;
    LightRenderer::Light lightSource;
    std::vector<Mesh> lodMeshes;
    std::vector<LightRenderer::Renderable> renderables;
    bool polygonizedRendering = false;
    bool lod = false;
    std::array<float, 50> frameTimes;
    int frame = 0;
};