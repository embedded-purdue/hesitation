#pragma once

#include <vector>
#include <string>
#include <memory>

#include "gui/tab.h"

struct GLFWwindow;

class GUI {
  public:
    bool Initialize(GLFWwindow* window);
    void Render();
    void Shutdown();
    ~GUI();

    void SaveStateToFile(const std::string& filepath = "layout.json") const;
    void LoadStateFromFile(const std::string& filepath = "layout.json");

    void SaveLayoutDialog();
    void LoadLayoutDialog();

    // Helper template to instantiate new tabs
    template <typename T, typename... Args> T* CreateTab(Args&&... args) {
        auto tab = std::make_unique<T>(std::forward<Args>(args)...);
        T* ptr   = tab.get();
        m_tabs.push_back(std::move(tab));
        return ptr;
    }

  private:
    void ApplyStyle(float scale);
    void RenderMainMenuBar();

    GLFWwindow* m_window = nullptr;
    std::vector<std::unique_ptr<ITab>> m_tabs;
};