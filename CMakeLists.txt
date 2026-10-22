cmake_minimum_required(VERSION 3.20)

project(Hesitation
    VERSION 0.0.0
    LANGUAGES CXX C
)

# ==============================================================================
# PROJECT CONFIGURATION
# ==============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep all executables in a predictable location.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# ==============================================================================
# DEPENDENCIES
# ==============================================================================

include(FetchContent)

# ------------------------------------------------------------------------------
# GLFW
# ------------------------------------------------------------------------------

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(glfw)

# ------------------------------------------------------------------------------
# Dear ImGui (with docking)
# ------------------------------------------------------------------------------

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8-docking
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(imgui)

# Build Dear ImGui as a static library.
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp

    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui_lib
    PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui_lib
    PUBLIC
        glfw
)

# ------------------------------------------------------------------------------
# ImPlot
# ------------------------------------------------------------------------------

FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.16
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(implot)

add_library(implot_lib STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

target_include_directories(implot_lib
    PUBLIC
        ${implot_SOURCE_DIR}
)

target_link_libraries(implot_lib
    PUBLIC
        imgui_lib
)

# ==============================================================================
# SYSTEM DEPENDENCIES
# ==============================================================================

find_package(OpenGL REQUIRED)

# ------------------------------------------------------------------------------
# Native File Dialog Extended (nfd)
# ------------------------------------------------------------------------------

FetchContent_Declare(
    nfd
    GIT_REPOSITORY https://github.com/btzy/nativefiledialog-extended.git
    GIT_TAG        v1.2.1
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(nfd)

# ==============================================================================
# APPLICATION
# ==============================================================================

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/app/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/src/*.h"
)

add_executable(${PROJECT_NAME}
    "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/main.h"
    ${PROJECT_SOURCES}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/"
        ${CMAKE_CURRENT_SOURCE_DIR}/app/src
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        imgui_lib
        implot_lib
        OpenGL::GL
        nfd
)

# ==============================================================================
# COMPILER WARNINGS
# ==============================================================================

if (MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            /W4
            /permissive-
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()

# ==============================================================================
# ASSETS
# ==============================================================================

add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/app/assets"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/app/assets"

    COMMENT "Copying assets..."
)
