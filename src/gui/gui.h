#pragma once

class GLFWwindow;

// The main class for the simulation GUI.
class GUI {
  public:
    GUI() = default;
    ~GUI();

    bool Initialize(GLFWwindow* window);
    void Render();
    void Shutdown();

  private:
    void ApplyStyle(float scale);

    GLFWwindow* m_window = nullptr;
};