cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(EvolutionaryMeshOptimization)

add_executable(EvolutionaryMeshOptimization
    src/main.cpp
    src/ModelLoader.h src/ModelLoader.cpp
    src/ModelOptimizer.h src/ModelOptimizer.cpp
)

add_subdirectory(Dependencies)

target_compile_definitions(EvolutionaryMeshOptimization
    PRIVATE 
        CALA_API=CALA_API_OPENGL
        ASSETS_DIR="${CMAKE_SOURCE_DIR}/Assets/"
)

target_link_libraries(EvolutionaryMeshOptimization
    PRIVATE
        Cala
)