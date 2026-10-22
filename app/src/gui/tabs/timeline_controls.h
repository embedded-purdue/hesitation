#pragma once

#include <string>
#include <utility>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "../tab.h"
#include "gui/tab_registry.h"
#include "imgui.h"

class TimelineControlsTab : public ITab {
  public:
    explicit TimelineControlsTab(std::string instanceId = "0")
        : ITab("Timeline Controls", std::move(instanceId)) {}

  protected:
    void OnRender() override {
        // UNIX epoch timestamp (seconds since 1970-01-01 00:00:00 UTC)
        static double currentEpoch = 1700000000.0;
        static float runningTime   = 0.0f;
        static bool isRunning      = false;

        // Convert Unix timestamp to UTC date/time.
        const auto epochSeconds = static_cast<std::time_t>(currentEpoch);

        const std::tm* utcTime = std::gmtime(&epochSeconds);

        std::string formattedEpoch = "Invalid timestamp";

        if (utcTime != nullptr) {
            std::ostringstream oss;

            // Format date and whole seconds first.
            oss << std::put_time(utcTime, "%Y-%m-%d %H:%M:%S");

            // Append fractional seconds with a decimal point.
            const double fractional = currentEpoch - static_cast<double>(epochSeconds);

            const int fractionalHundredths = static_cast<int>(fractional * 100.0);

            oss << '.' << std::setfill('0') << std::setw(2) << fractionalHundredths << " UTC";

            formattedEpoch = oss.str();
        }

        // Status displays
        ImGui::Text("Current Epoch: %s", formattedEpoch.c_str());
        ImGui::Text("Running Time:  %.2f s", runningTime);

        ImGui::Separator();

        // Transport controls
        const float buttonHeight = ImGui::GetFrameHeight();

        if (ImGui::Button(isRunning ? "Stop" : "Start", ImVec2(0.0f, buttonHeight))) {
            isRunning = !isRunning;
        }

        ImGui::SameLine();

        if (ImGui::Button("Reset", ImVec2(0.0f, buttonHeight))) {
            currentEpoch = 1700000000.0;
            runningTime  = 0.0f;
            isRunning    = false;
        }
    }
};

REGISTER_TAB(TimelineControlsTab, "Settings", "Timeline Controls");