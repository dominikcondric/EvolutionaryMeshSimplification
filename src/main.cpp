#include "Utility/Window.h"
#include "Rendering/Mesh.h"
#include <iostream>
#include "Rendering/Shader.h"
#include "Rendering/GraphicsAPI.h"
#include "Rendering/Camera.h"
#include "Utility/Transformation.h"
#include "Rendering/Renderers/HelperGridRenderer.h"
#include "Rendering/Renderers/LightRenderer.h"
#include "Rendering/Renderers/SimpleRenderer.h"
#include "Rendering/Renderers/SkyboxRenderer.h"
#include <glm/gtc/random.hpp>
#include "ModelLoader.h"
#include "ModelOptimizer.h"

using namespace Cala;

void update(GraphicsAPI* api, PerspectiveCamera& camera, Window& window) 
{
    const IOSystem& ioSystem = window.getIO();

    if (ioSystem.isMouseButtonPressed(IOSystem::MOUSE_BUTTON_RIGHT))
        camera.rotateCamera(ioSystem.getCursorOffset());

    if (ioSystem.isKeyPressed(IOSystem::KEY_W))
        camera.moveCamera(Camera::Directions::FORWARD, 0.01f);

    if (ioSystem.isKeyPressed(IOSystem::KEY_S))
        camera.moveCamera(Camera::Directions::BACKWARD, 0.01f);

    if (ioSystem.isKeyPressed(IOSystem::KEY_A))
        camera.moveCamera(Camera::Directions::LEFT, 0.01f);

    if (ioSystem.isKeyPressed(IOSystem::KEY_D))
        camera.moveCamera(Camera::Directions::RIGHT, 0.01f);

    if (ioSystem.isKeyPressed(IOSystem::KEY_SPACE))
        camera.moveCamera(Camera::Directions::UP, 0.01f);

    if (ioSystem.isKeyPressed(IOSystem::KEY_LEFT_SHIFT))
        camera.moveCamera(Camera::Directions::DOWN, 0.01f);

    window.update();
    if (window.isResized())
    {
        auto winSize = window.getWindowSize();
        api->setViewport(glm::uvec4(0, 0, winSize.x, winSize.y));
        camera.setProjectionAspectRatio((float)winSize.x / winSize.y);
    }
}

int main(void)
{
    // Window
    Window::WindowSpecification windowSpecification;
    windowSpecification.width = 1920;
    windowSpecification.height = 1080;
    windowSpecification.sampleCount = 4;
    windowSpecification.windowName = "EvolutionaryMeshOptimization";
    Window window(windowSpecification);
    window.createOpenGLContext();

    // Initializing OpenGL API
    GraphicsAPI* api = GraphicsAPI::construct();
    api->setBufferClearingColor(glm::vec4(0.1f, 0.1f, 0.1f, 1.f));
    api->setBufferClearingBits(true, true, false);
    api->setViewport(glm::ivec4(0, 0, windowSpecification.width, windowSpecification.height));

    // Setting up camera
    PerspectiveCamera camera;
    camera.setPosition(glm::vec3(0.f, 15.f, 25.f));
    camera.setCenter(glm::vec3(0.f));
    camera.setProjectionFarPlane(200.f);
    camera.setProjectionAspectRatio((float)windowSpecification.width / windowSpecification.height);

    std::filesystem::path assetsDir(ASSETS_DIR);

    // Renderers
    LightRenderer lightRenderer;
    SimpleRenderer simpleRenderer;

    // Models
    Mesh lightMesh(Model().loadSphere());
    SimpleRenderer::Renderable lightRenderable(lightMesh, Transformation().translate(glm::vec3(0.f, 15.f, 5.f)).scale(0.5f),
        glm::vec4(1.f));
    
    LightRenderer::Light lightSource(LightRenderer::Light::Type::Point, lightRenderable.transformation, 1.f, 
        lightRenderable.color, 0.f);

    // Model optimizer
    ModelOptimizer modelOptimizer(30, 20, 5, 0.1f, 5);

    // Meshes
    std::vector<Mesh> meshes;
    uint32_t counter = 1;
    std::vector<Cala::Model> models = ModelLoader::loadOBJModels(assetsDir / "airplane_v2_L2.123c71795678-4b63-46c4-b2c6-549c45f4c806/11805_airplane_v2_L2.obj");
    for (auto& model : models)
    {
        std::cout << "Optimizing model " << counter++ << '/' << models.size() << '\n';
        modelOptimizer.optimize(&model);
        meshes.emplace_back(model, false);
    }

    // Renderables
    std::vector<LightRenderer::ColoredRenderable> renderables;
    int i = 0;
    for (const Mesh& mesh : meshes)
    {
        renderables.emplace_back(mesh, Transformation().scale(0.01f).rotate(-90.f, glm::vec3(1.f, 0.f, 0.f)),
            glm::vec4(0.8f, 0.8f, 0.8f, 1.f), 0.2f, 0.7f, 0.9f, 10.f);
    }

    api->enableSetting(GraphicsAPI::Multisampling);
    api->enableSetting(GraphicsAPI::DepthTesting);

    // Rendering loop
    while (!window.exitTriggered())
    {
        api->clearFramebuffer();

        lightRenderer.pushLight(lightSource);
        for (const auto& renderable : renderables)
            lightRenderer.pushColoredRenderable(renderable);

        //api->setPolygonFillingMode(GraphicsAPI::Front, GraphicsAPI::Lines);
        lightRenderer.render(api, camera);
        //api->setPolygonFillingMode(GraphicsAPI::FrontAndBack, GraphicsAPI::Fill);

        simpleRenderer.pushRenderable(lightRenderable);
        simpleRenderer.render(api, camera);

        update(api, camera, window);
    }

    delete api;
    window.shutdown();
}