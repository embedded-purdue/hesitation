include(FetchContent)

# ==============================================================================
# GLFW
# ==============================================================================

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(glfw)

# ==============================================================================
# DEAR IMGUI
# ==============================================================================

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8-docking
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(imgui)

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

target_link_libraries(imgui_lib PUBLIC glfw)

# ==============================================================================
# IMPLOT
# ==============================================================================

FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
    GIT_SHALLOW TRUE
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

target_link_libraries(implot_lib PUBLIC imgui_lib)

# ==============================================================================
# SYSTEM DEPENDENCIES
# ==============================================================================

find_package(OpenGL REQUIRED)

# ==============================================================================
# NATIVE FILE DIALOG
# ==============================================================================

FetchContent_Declare(
    nfd
    GIT_REPOSITORY https://github.com/btzy/nativefiledialog-extended.git
    GIT_TAG v1.2.1
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(nfd)

# ==============================================================================
# APPLICATION SOURCES
# ==============================================================================

file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
)

list(FILTER APP_SOURCES EXCLUDE REGEX "/python/")

add_executable(nep_controls
    "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/main.h"
    ${APP_SOURCES}
)

# ==============================================================================
# INCLUDE DIRECTORIES
# ==============================================================================

target_include_directories(nep_controls
    PRIVATE
        "${PROJECT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# ==============================================================================
# LINK LIBRARIES
# ==============================================================================

target_link_libraries(nep_controls
    PRIVATE
        imgui_lib
        implot_lib
        OpenGL::GL
        messaging
        nfd
)

# Optional engineering integrations
if(TARGET thrust_vectoring)
    target_link_libraries(nep_controls PRIVATE thrust_vectoring)
endif()

# ==============================================================================
# COMPILER WARNINGS
# ==============================================================================

if(MSVC)
    target_compile_options(nep_controls PRIVATE
        /W4
        /permissive-
    )
else()
    target_compile_options(nep_controls PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()

# ==============================================================================
# ASSETS
# ==============================================================================

add_custom_command(
    TARGET nep_controls
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:nep_controls>/app/assets"

    COMMENT "Copying assets..."
)
