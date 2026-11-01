#include "main.h"

#include "messaging/channels/environment/EnvironmentMessages.h"
#include "messaging/channels/environment/EnvironmentChannel.h"

#include <GLFW/glfw3.h>

Application::Application() : m_window(nullptr) {}

Application::~Application() {
    Shutdown();
}

bool Application::Initialize() {
    if (!glfwInit())
        return false;

    // Window / OpenGL Init
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    m_window = glfwCreateWindow(1280, 720, "Hesitation", nullptr, nullptr);

    if (!m_window) {
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);

    // Enable VSync to synchronize with the display refresh rate
    glfwSwapInterval(1);

    // Start maximized
    glfwMaximizeWindow(m_window);

    // Update the viewport when the framebuffer is resized
    glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow*, int width, int height) {
        if (width > 0 && height > 0)
            glViewport(0, 0, width, height);
    });

    // GUI Init
    if (!m_gui.Initialize(m_window)) {
        Shutdown();
        return false;
    }

    // Register all known message types.
    Hesitation::Environment::EnvironmentMessages::RegisterAll();

    // Register OrbitSim listeners.
    if (!m_orbitSim.Initialize())
        return false;

    // Start the central messaging network.
    if (!Hesitation::Environment::EnvironmentChannel::StartHub("tcp://*:5555", "tcp://*:5556")) {
        return false;
    }

    // Connect application modules to the network.
    if (!Hesitation::Environment::EnvironmentChannel::Start("tcp://127.0.0.1:5555",
                                                            "tcp://127.0.0.1:5556")) {
        Hesitation::Environment::EnvironmentChannel::Stop();
        return false;
    }

    // Start background tasks
    m_orbitSim.Start();

    return true;
}

void Application::Run() {
    // Main application loop
    while (!glfwWindowShouldClose(m_window)) {
        glfwPollEvents();

        m_gui.Render();

        // Present the completed frame
        glfwSwapBuffers(m_window);
    }
}

void Application::Shutdown() {
    // Stop background tasks first
    m_orbitSim.Stop();

    // Shut down messaging channels
    Hesitation::Environment::EnvironmentChannel::Stop();

    // Shut down modules before dependencies
    m_gui.Shutdown();

    // Destroy the window and OpenGL context
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }

    glfwTerminate();
}

int main() {
    Application app;

    if (!app.Initialize())
        return 1;

    app.Run();

    return 0;
}
