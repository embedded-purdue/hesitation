#pragma once

#include <string>
#include <utility>
#include "imgui.h"

class ITab {
  public:
    // Pass a title and unique insance ID
    explicit ITab(std::string title, std::string instanceId = "0")
        : m_title(std::move(title)), m_instanceId(std::move(instanceId)) {}

    virtual ~ITab() = default;

    void Render() {
        if (!m_open)
            return;

        // Combine title + instance ID after '###'
        // Result: "Console###Tab_Console_1", "Console###Tab_Console_2"
        std::string uniqueTitle = m_title + "###Tab_" + m_title + "_" + m_instanceId;

        if (ImGui::Begin(uniqueTitle.c_str(), &m_open, GetFlags())) {
            OnRender();
        }
        ImGui::End();
    }

    bool IsOpen() const {
        return m_open;
    }
    void Close() {
        m_open = false;
    }
    const std::string& GetTitle() const {
        return m_title;
    }
    const std::string& GetInstanceId() const {
        return m_instanceId;
    }

  protected:
    virtual void OnRender() = 0;
    virtual ImGuiWindowFlags GetFlags() const {
        return 0;
    }

  private:
    std::string m_title;
    std::string m_instanceId;
    bool m_open = true;
};