cmake_minimum_required(VERSION 3.5)

project(3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Vulkan REQUIRED)
find_package(glm
             PATHS D:/glm/cmake/glm
             REQUIRED)

set(GLFW D:/glfw-3.3.2.bin.WIN64)
set(GLM D:/glm)

add_library(geometry STATIC geometry_base.cpp geometry.cpp intersection_finder.cpp triangles_generator.cpp)
add_library(vulkan_visualization STATIC vulkan_drawing.cpp)

target_include_directories(vulkan_visualization PRIVATE
    $ENV{VULKAN_SDK}/Include
    ${GLFW}/include
    ${GLM})

target_link_libraries(vulkan_visualization
    $ENV{VULKAN_SDK}/Lib/vulkan-1.lib
    ${GLFW}/lib-mingw-w64/libglfw3.a)

add_custom_command(TARGET vulkan_visualization
                   COMMAND $ENV{VULKAN_SDK}/Bin/glslc.exe
                   ARGS shaders/shader.frag -o frag.spv
                   COMMAND $ENV{VULKAN_SDK}/Bin/glslc.exe
                   ARGS shaders/shader.vert -o vert.spv)

add_executable(3 main.cpp)

target_link_libraries(3 geometry vulkan_visualization)
