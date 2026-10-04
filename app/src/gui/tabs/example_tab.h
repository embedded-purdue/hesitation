#pragma once

#include "../tab.h" // or "gui/tab.h"
#include "gui/tab_registry.h"

class ExampleTab : public ITab {
  public:
    explicit ExampleTab(std::string instanceId = "0")
        : ITab("Example Tab", std::move(instanceId)) {}

  protected:
    void OnRender() override {
        ImGui::Text("Hello from my custom tab!");

        static float val = 0.5f;
        ImGui::SliderFloat("Speed", &val, 0.0f, 1.0f);

        if (ImGui::Button("Action")) {
        }
    }
};

REGISTER_TAB(ExampleTab, "General", "Example Tab");