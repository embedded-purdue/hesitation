#pragma once

#include <string>
#include <utility>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cmath>

#include "../tab.h"
#include "gui/tab_registry.h"
#include "imgui.h"
#include "../../AppData.h"
#include "environment.pb.h"

#include "messaging/channels/environment/EnvironmentChannel.h"
#include "messaging/channels/environment/EnvironmentMessages.h"

class TimelineControlsTab : public ITab {
  public:
    explicit TimelineControlsTab(std::string instanceId = "0")
        : ITab("Timeline Controls", std::move(instanceId)) {}

  protected:
    void OnRender() override {
        static double currentEpoch = 1700000000.0;
        static float runningTime   = 0.0f;
        static bool isRunning      = false;

        // Convert Unix timestamp to UTC date/time.
        const auto epochSeconds = static_cast<std::time_t>(currentEpoch);
        const std::tm* utcTime  = std::gmtime(&epochSeconds);

        std::string formattedEpoch = "Invalid timestamp";

        if (utcTime != nullptr) {
            std::ostringstream oss;

            oss << std::put_time(utcTime, "%Y-%m-%d %H:%M:%S");

            const double fractional = currentEpoch - static_cast<double>(epochSeconds);

            const int fractionalHundredths = static_cast<int>(fractional * 100.0);

            oss << '.' << std::setfill('0') << std::setw(2) << fractionalHundredths << " UTC";

            formattedEpoch = oss.str();
        }

        // Status displays.
        ImGui::Text("Current Epoch: %s", formattedEpoch.c_str());
        ImGui::Text("Running Time: %.2f s", runningTime);

        ImGui::Separator();

        // Transport controls.
        const float buttonHeight = ImGui::GetFrameHeight();

        // Start / Stop.
        if (ImGui::Button(isRunning ? "Stop" : "Start", ImVec2(0.0f, buttonHeight))) {

            auto message = isRunning
                               ? Hesitation::Environment::EnvironmentMessages::SimulationStop()
                               : Hesitation::Environment::EnvironmentMessages::SimulationStart();

            if (Hesitation::Environment::EnvironmentChannel::Send(message)) {
                isRunning = !isRunning;
            }
        }

        ImGui::SameLine();

        // Reset.
        if (ImGui::Button("Reset", ImVec2(0.0f, buttonHeight))) {

            auto& config = GetApplicationData().simulationConfig;

            hesitation::environment::SimulationReset reset;

            // Run type.
            reset.set_run_type(config.runType);

            // Convert Unix epoch into UTC calendar time.
            const std::time_t timestamp = static_cast<std::time_t>(std::floor(config.epoch));

            std::tm* utcTime = std::gmtime(&timestamp);

            if (utcTime) {
                reset.set_year(utcTime->tm_year + 1900);
                reset.set_month(utcTime->tm_mon + 1);
                reset.set_day(utcTime->tm_mday);
                reset.set_hour(utcTime->tm_hour);
                reset.set_minute(utcTime->tm_min);

                reset.set_second(static_cast<double>(utcTime->tm_sec) +
                                 (config.epoch - std::floor(config.epoch)));
            }

            // Position.
            reset.set_position_x(config.position[0]);
            reset.set_position_y(config.position[1]);
            reset.set_position_z(config.position[2]);

            // Velocity.
            reset.set_velocity_x(config.velocity[0]);
            reset.set_velocity_y(config.velocity[1]);
            reset.set_velocity_z(config.velocity[2]);

            // Attitude (MRP).
            reset.set_mrp_x(config.sigmaBN[0]);
            reset.set_mrp_y(config.sigmaBN[1]);
            reset.set_mrp_z(config.sigmaBN[2]);

            // Angular velocity.
            reset.set_omega_x(config.omegaBNB[0]);
            reset.set_omega_y(config.omegaBNB[1]);
            reset.set_omega_z(config.omegaBNB[2]);

            // Build and send message envelope.
            auto message = Hesitation::Environment::EnvironmentMessages::SimulationReset(reset);

            if (Hesitation::Environment::EnvironmentChannel::Send(message)) {
                runningTime = 0.0f;
                isRunning   = false;
            }
        }
    }
};

REGISTER_TAB(TimelineControlsTab, "Settings", "Timeline Controls");
