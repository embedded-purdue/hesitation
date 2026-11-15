#pragma once

#include <array>
#include <vector>
#include <string>

#include "gui/tab.h"
#include "gui/tab_registry.h"

class ConsoleTab : public ITab {
  public:
    explicit ConsoleTab(std::string instanceId = "0") : ITab("Console", std::move(instanceId)) {
        m_logs.push_back("App initialized!");
    }

  protected:
    void OnRender() override {
        // Scrollable log area
        const float footerHeight =
            ImGui::GetStyle().ItemSpacing.y + ImGui::GetFrameHeightWithSpacing();
        if (ImGui::BeginChild("LogRegion", ImVec2(0, -footerHeight), true)) {
            for (const auto& log : m_logs) {
                ImGui::TextUnformatted(log.c_str());
            }
        }
        ImGui::EndChild();

        // Input field
        ImGui::PushItemWidth(-1.0f);
        if (ImGui::InputText("##Input", m_inputBuffer.data(), m_inputBuffer.size(),
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            if (m_inputBuffer[0] != '\0') {
                m_logs.push_back(std::string("> ") + m_inputBuffer.data());
                m_inputBuffer.fill(0);
            }
            ImGui::SetKeyboardFocusHere(-1);
        }
        ImGui::PopItemWidth();
    }

  private:
    std::vector<std::string> m_logs;
    std::array<char, 256> m_inputBuffer{};
};

REGISTER_TAB(ConsoleTab, "Tools/Debugging", "Console");