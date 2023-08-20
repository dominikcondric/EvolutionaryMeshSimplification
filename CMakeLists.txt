cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
set(ProjectName EvolutionaryMeshSimplification)
project(${ProjectName})

add_executable(${ProjectName}
    src/main.cpp
    src/MyApplication.h src/MyApplication.cpp
    src/ModelOptimizer.h src/ModelOptimizer.cpp
    src/MathUtils.h src/MathUtils.cpp
)

add_subdirectory(Dependencies)

target_compile_definitions(${ProjectName}
    PRIVATE 
        CALA_API=CALA_API_OPENGL
        ASSETS_DIR="${CMAKE_SOURCE_DIR}/Assets/"
)

set(DependenciesDir ${PROJECT_SOURCE_DIR}/Dependencies)

target_include_directories(${ProjectName}
	PRIVATE
		${DependenciesDir}/OBJLoader/Source/
        ${DependenciesDir}/matplotlib/
)

set(CGAL_DIR ${DependenciesDir}/cgal/)
set(CGAL_DISABLE_GMP ON)
set(Boost_INCLUDE_DIR ${DependenciesDir}/boost_1_83_0/)

find_package(CGAL REQUIRED)

target_link_libraries(${ProjectName}
    PRIVATE
        Cala
        matplotlib_cpp
        CGAL::CGAL
)