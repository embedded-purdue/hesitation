#pragma once

#include "gui/gui.h"

class GLFWwindow;

// Process manager to launch the application and own the GUI, Physics Sim, and Hardware modules.
// This should own the modules and launch them.
// This should NOT facilitate communication between the modules or perform tasks that should take
// place in the modules.
class Application {
  public:
    Application();
    ~Application();

    bool Initialize();
    void Run();
    void Shutdown();

  private:
    GLFWwindow* m_window;
    GUI m_gui;
};
