#include "MyApplication.h"
#include "Cala/Utility/Logger.h"
#include "Cala/Utility/GLFWWindow.h"
#include "Cala/Utility/ModelLoader.h"
#include <iostream>
#include <filesystem>
#include <numeric>

using namespace Cala;

MyApplication::MyApplication() : 
        BaseApplication(IWindow::Specification("EvolutionaryMeshSimplification", 1024, 768, 4)), 
        sphereMesh(Model().loadSphere()),
        lightRenderable(sphereMesh, Transformation().translate(glm::vec3(0.f, 30.f, 0.f)).scale(0.5f), glm::vec4(1.f)),
        lightSource(LightRenderer::Light::Type::Point, lightRenderable.transformation, 1.f, lightRenderable.color, 0.f, false)
{
    camera.setPosition(glm::vec3(0.f, 15.f, 25.f));
    camera.setCenter(glm::vec3(0.f));
    camera.setProjectionFarPlane(200.f);
    camera.setProjectionAspectRatio((float)window->getWindowSize().x / window->getWindowSize().y);

    std::filesystem::path assetsDir(ASSETS_DIR);

    std::string modelPath;
    do {
        std::cout << "Enter a mesh to optimize in Assets directory: ";
        std::cin >> modelPath;
    } while (!std::filesystem::exists(assetsDir / modelPath));

    // Input
    std::cout << "Use default values (G: 20, P: 50, O: 5, M: 0.1, TS: 5), Y/N: ";
    char defaultInput;
    do {
        std::cin >> defaultInput;
    } while (defaultInput != 'Y' && defaultInput != 'N');

    uint32_t generationCount = 0, populationCount = 0, offspringCount = 0, tournamentSelectionParticipantsCount = 0;
    float mutationProbability = 0;

    if (defaultInput == 'Y') 
    {
        generationCount = 20;
        populationCount = 50;
        offspringCount = 5;
        mutationProbability = 0.1f;
        tournamentSelectionParticipantsCount = 5;
    }
    else
    {
        do {
            std::cout << "Enter generation count: ";
            std::cin >> generationCount;
        } while (generationCount == 0 || generationCount > 200);

        do {
            std::cout << "Enter population count: ";
            std::cin >> populationCount;
        } while (populationCount == 0 || populationCount > 500);

        do {
            std::cout << "Enter offspring count: ";
            std::cin >> offspringCount;
        } while (offspringCount == 0 || offspringCount > populationCount);

        do {
            std::cout << "Enter mutation probablity: ";
            std::cin >> mutationProbability;
        } while (mutationProbability < 0.f || mutationProbability > 1.f);

        do {
            std::cout << "Enter tournament selection participants count: ";
            std::cin >> tournamentSelectionParticipantsCount;
        } while (tournamentSelectionParticipantsCount == 0 || tournamentSelectionParticipantsCount > 10);
    }

    uint32_t counter = 1;
    Cala::ModelLoader modelLoader(false);
    modelLoader.loadFromObj(assetsDir / modelPath);
    Cala::Model originalModel = modelLoader.getModels()[0];
    auto lodLevels = modelOptimizer.optimize(&originalModel, generationCount, populationCount, offspringCount, mutationProbability, tournamentSelectionParticipantsCount);
    for (auto& level : lodLevels)
    {
        level.createGPUVertexData();
        lodMeshes.emplace_back(level, false, true);
    }

    api->enableSetting(GraphicsAPI::Multisampling);
    api->enableSetting(GraphicsAPI::DepthTesting);
}

void MyApplication::loop()
{
    if (window->getIO().isKeyTapped(IIOSystem::KEY_P))
    {
        polygonizedRendering = !polygonizedRendering;
        if (polygonizedRendering)
            api->setPolygonFillingMode(GraphicsAPI::Front, GraphicsAPI::Lines);
        else
            api->setPolygonFillingMode(GraphicsAPI::Front, GraphicsAPI::Fill);
    }

    if (window->getIO().isKeyTapped(IIOSystem::KEY_L))
        lod = !lod;

    api->clearFramebuffer();

    lightRenderer.pushLight(lightSource);

    for (int i = 0; i <= 20; i++)
    {
        for (int j = 0; j <= 20; j++)
        {
            Mesh* mesh;
            glm::vec3 meshPosition(-80.f + i * 8.f, 0.f, -80.f + j * 8.f);

            if (!lod)
            {
                mesh = &lodMeshes[0];
            }
            else
            {
                float distance = glm::distance(meshPosition, camera.getPosition());
                int threshold = (int)(100.f / lodMeshes.size());
                int index = glm::clamp((int)(distance / threshold), 0, (int)lodMeshes.size() - 1);
                mesh = &lodMeshes[index];
            }

            lightRenderer.pushRenderable(
                Cala::LightRenderer::Renderable(
                    *mesh, 
                    Cala::Transformation().translate(meshPosition),
                    glm::vec4(1.f),
                    nullptr,
                    nullptr,
                    nullptr,
                    0.2f,
                    0.8f,
                    0.8f,
                    20.f
                )
            );
        }
    }

    lightRenderer.setupCamera(camera);
    lightRenderer.render(api.get(), nullptr);

    simpleRenderer.setupCamera(camera);
    simpleRenderer.pushRenderable(lightRenderable);
    simpleRenderer.render(api.get(), nullptr);
    
    frameTimes[frame] = time.getFrameRate();
    frame = (frame+1) % 50;
    std::cout << std::accumulate(frameTimes.begin(), frameTimes.end(), 0.f) / 50 << std::endl; 
 }