#pragma once

#include <string>
#include <utility>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "../tab.h"
#include "gui/tab_registry.h"
#include "imgui.h"

#include "../../AppData.h"

class InitialStateTab : public ITab {
  public:
    explicit InitialStateTab(std::string instanceId = "0")
        : ITab("Initial State", std::move(instanceId)) {}

  protected:
    void OnRender() override {
        constexpr double PI         = 3.14159265358979323846;
        constexpr double DEG_TO_RAD = PI / 180.0;
        constexpr double EARTH_MU   = 3.986004418e14;

        auto& config = GetApplicationData().simulationConfig;

        ImGui::TextDisabled("Applied on reset");
        ImGui::Spacing();

        // Run Type
        const char* runTypes[] = {"Regular", "Orbit Sim Test"};

        int runTypeIndex = config.runType == "Orbit Sim Test" ? 1 : 0;

        ImGui::Text("Run Type");
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);

        if (ImGui::Combo("##RunType", &runTypeIndex, runTypes, IM_ARRAYSIZE(runTypes))) {
            config.runType = runTypes[runTypeIndex];
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Epoch
        std::time_t timestamp = static_cast<std::time_t>(std::floor(config.epoch));

        const std::tm* utcTime = std::gmtime(&timestamp);

        static int year        = 2023;
        static int month       = 11;
        static int day         = 14;
        static int hour        = 22;
        static int minute      = 13;
        static int second      = 20;
        static int millisecond = 0;

        if (utcTime) {
            year   = utcTime->tm_year + 1900;
            month  = utcTime->tm_mon + 1;
            day    = utcTime->tm_mday;
            hour   = utcTime->tm_hour;
            minute = utcTime->tm_min;
            second = utcTime->tm_sec;

            millisecond =
                static_cast<int>(std::round((config.epoch - std::floor(config.epoch)) * 1000.0));
        }

        ImGui::Text("Epoch");
        ImGui::TextDisabled("UTC date and time | Unix seconds");
        ImGui::Spacing();

        const float spacing   = ImGui::GetStyle().ItemSpacing.x;
        const float available = ImGui::GetContentRegionAvail().x;

        bool epochEdited = false;

        const float dateWidth = std::max(1.0f, (available - spacing * 2.0f) / 3.0f);

        ImGui::TextUnformatted("Year");
        ImGui::SameLine(dateWidth + spacing);
        ImGui::TextUnformatted("Month");
        ImGui::SameLine(dateWidth * 2.0f + spacing * 2.0f);
        ImGui::TextUnformatted("Day");

        ImGui::SetNextItemWidth(dateWidth);
        if (ImGui::InputInt("##EpochYear", &year, 0, 0))
            epochEdited = true;

        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(dateWidth);
        if (ImGui::InputInt("##EpochMonth", &month, 0, 0))
            epochEdited = true;

        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(dateWidth);
        if (ImGui::InputInt("##EpochDay", &day, 0, 0))
            epochEdited = true;

        ImGui::Spacing();

        const float timeWidth = std::max(1.0f, (available - spacing * 3.0f) / 4.0f);

        ImGui::TextUnformatted("Hour");
        ImGui::SameLine(timeWidth + spacing);
        ImGui::TextUnformatted("Minute");
        ImGui::SameLine(timeWidth * 2.0f + spacing * 2.0f);
        ImGui::TextUnformatted("Second");
        ImGui::SameLine(timeWidth * 3.0f + spacing * 3.0f);
        ImGui::TextUnformatted("Millis");

        ImGui::SetNextItemWidth(timeWidth);
        if (ImGui::InputInt("##EpochHour", &hour, 0, 0))
            epochEdited = true;

        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(timeWidth);
        if (ImGui::InputInt("##EpochMinute", &minute, 0, 0))
            epochEdited = true;

        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(timeWidth);
        if (ImGui::InputInt("##EpochSecond", &second, 0, 0))
            epochEdited = true;

        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(timeWidth);
        if (ImGui::InputInt("##EpochMillis", &millisecond, 0, 0))
            epochEdited = true;

        month       = std::clamp(month, 1, 12);
        day         = std::clamp(day, 1, 31);
        hour        = std::clamp(hour, 0, 23);
        minute      = std::clamp(minute, 0, 59);
        second      = std::clamp(second, 0, 59);
        millisecond = std::clamp(millisecond, 0, 999);

        if (epochEdited) {
            std::tm edited = {};
            edited.tm_year = year - 1900;
            edited.tm_mon  = month - 1;
            edited.tm_mday = day;
            edited.tm_hour = hour;
            edited.tm_min  = minute;
            edited.tm_sec  = second;

#ifdef _WIN32
            const std::time_t t = _mkgmtime(&edited);
#else
            const std::time_t t = timegm(&edited);
#endif

            config.epoch = static_cast<double>(t) + static_cast<double>(millisecond) / 1000.0;
        }

        // Translational State
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        const char* stateModes[] = {"Orbital Elements", "Cartesian Position / Velocity"};

        ImGui::Text("Translational State");
        ImGui::TextDisabled("Orbital elements are converted to inertial state vectors");
        ImGui::Spacing();

        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::Combo("##InputMode", &config.stateMode, stateModes, IM_ARRAYSIZE(stateModes));

        if (config.stateMode == 0) {
            ImGui::Spacing();
            ImGui::Text("Classical Orbital Elements");
            ImGui::TextDisabled("Semi-major axis in m | Angles in degrees");
            ImGui::Spacing();

            const float w3 =
                std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f);

            ImGui::TextUnformatted("Semi-major axis (m)");
            ImGui::SameLine(w3 + spacing);
            ImGui::TextUnformatted("Eccentricity");
            ImGui::SameLine(w3 * 2.0f + spacing * 2.0f);
            ImGui::TextUnformatted("Inclination (deg)");

            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COEA", ImGuiDataType_Double, &config.semiMajorAxis, nullptr,
                               nullptr, "%.3f");

            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COEE", ImGuiDataType_Double, &config.eccentricity, nullptr,
                               nullptr, "%.8f");

            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COEI", ImGuiDataType_Double, &config.inclination, nullptr,
                               nullptr, "%.5f");

            ImGui::Spacing();

            ImGui::TextUnformatted("RAAN (deg)");
            ImGui::SameLine(w3 + spacing);
            ImGui::TextUnformatted("Argument of periapsis (deg)");
            ImGui::SameLine(w3 * 2.0f + spacing * 2.0f);
            ImGui::TextUnformatted("True anomaly (deg)");

            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COERAAN", ImGuiDataType_Double, &config.raan, nullptr, nullptr,
                               "%.5f");

            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COEArgPeri", ImGuiDataType_Double, &config.argPeriapsis, nullptr,
                               nullptr, "%.5f");

            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COETrueAnomaly", ImGuiDataType_Double, &config.trueAnomaly,
                               nullptr, nullptr, "%.5f");

            // Convert COEs to inertial Cartesian state.
            const bool validOrbit = config.semiMajorAxis > 0.0 && config.eccentricity >= 0.0 &&
                                    config.eccentricity < 1.0;

            if (validOrbit) {
                const double inc = config.inclination * DEG_TO_RAD;
                const double O   = config.raan * DEG_TO_RAD;
                const double w   = config.argPeriapsis * DEG_TO_RAD;
                const double f   = config.trueAnomaly * DEG_TO_RAD;

                const double p =
                    config.semiMajorAxis * (1.0 - config.eccentricity * config.eccentricity);

                const double denom = 1.0 + config.eccentricity * std::cos(f);

                if (p > 0.0 && denom > 0.0) {
                    const double radius     = p / denom;
                    const double speedScale = std::sqrt(EARTH_MU / p);

                    const double rpX = radius * std::cos(f);
                    const double rpY = radius * std::sin(f);

                    const double vpX = -speedScale * std::sin(f);
                    const double vpY = speedScale * (config.eccentricity + std::cos(f));

                    const double cO = std::cos(O);
                    const double sO = std::sin(O);
                    const double cw = std::cos(w);
                    const double sw = std::sin(w);
                    const double ci = std::cos(inc);
                    const double si = std::sin(inc);

                    const double r11 = cO * cw - sO * sw * ci;
                    const double r12 = -cO * sw - sO * cw * ci;
                    const double r21 = sO * cw + cO * sw * ci;
                    const double r22 = -sO * sw + cO * cw * ci;
                    const double r31 = sw * si;
                    const double r32 = cw * si;

                    config.position[0] = r11 * rpX + r12 * rpY;
                    config.position[1] = r21 * rpX + r22 * rpY;
                    config.position[2] = r31 * rpX + r32 * rpY;

                    config.velocity[0] = r11 * vpX + r12 * vpY;
                    config.velocity[1] = r21 * vpX + r22 * vpY;
                    config.velocity[2] = r31 * vpX + r32 * vpY;
                }
            }
        } else {
            ImGui::Spacing();

            ImGui::Text("Position");
            ImGui::TextDisabled("Inertial frame N | meters (m)");
            ImGui::Spacing();

            const float w3 =
                std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f);

            const char* posLabels[] = {"X (m)", "Y (m)", "Z (m)"};

            for (int i = 0; i < 3; ++i) {
                ImGui::TextUnformatted(posLabels[i]);

                if (i > 0)
                    ImGui::SameLine(w3 * i + spacing * i);

                ImGui::SetNextItemWidth(w3);
                ImGui::InputScalar((std::string("##Pos") + std::to_string(i)).c_str(),
                                   ImGuiDataType_Double, &config.position[i], nullptr, nullptr,
                                   "%.6f");
            }

            ImGui::Spacing();

            ImGui::Text("Velocity");
            ImGui::TextDisabled("Inertial frame N | meters per second (m/s)");
            ImGui::Spacing();

            const char* velLabels[] = {"Vx (m/s)", "Vy (m/s)", "Vz (m/s)"};

            for (int i = 0; i < 3; ++i) {
                ImGui::TextUnformatted(velLabels[i]);

                if (i > 0)
                    ImGui::SameLine(w3 * i + spacing * i);

                ImGui::SetNextItemWidth(w3);
                ImGui::InputScalar((std::string("##Vel") + std::to_string(i)).c_str(),
                                   ImGuiDataType_Double, &config.velocity[i], nullptr, nullptr,
                                   "%.6f");
            }
        }

        ImGui::Spacing();

        ImGui::TextDisabled("Position (km): X %.3f | Y %.3f | Z %.3f", config.position[0] / 1000.0,
                            config.position[1] / 1000.0, config.position[2] / 1000.0);

        ImGui::TextDisabled("Velocity (km/s): X %.6f | Y %.6f | Z %.6f",
                            config.velocity[0] / 1000.0, config.velocity[1] / 1000.0,
                            config.velocity[2] / 1000.0);

        // Attitude
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Attitude");
        ImGui::TextDisabled("MRP components | Dimensionless");
        ImGui::Spacing();

        const float attitudeWidth =
            std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f);

        const char* sigmaLabels[] = {"Sigma 1", "Sigma 2", "Sigma 3"};

        for (int i = 0; i < 3; ++i) {
            ImGui::TextUnformatted(sigmaLabels[i]);

            if (i > 0)
                ImGui::SameLine(attitudeWidth * i + spacing * i);

            ImGui::SetNextItemWidth(attitudeWidth);
            ImGui::InputScalar((std::string("##Sigma") + std::to_string(i)).c_str(),
                               ImGuiDataType_Double, &config.sigmaBN[i], nullptr, nullptr, "%.8f");
        }

        // Angular velocity
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Angular Velocity");
        ImGui::TextDisabled("Body frame | Radians per second (rad/s)");
        ImGui::Spacing();

        const float omegaWidth =
            std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f);

        const char* omegaLabels[] = {"X (rad/s)", "Y (rad/s)", "Z (rad/s)"};

        for (int i = 0; i < 3; ++i) {
            ImGui::TextUnformatted(omegaLabels[i]);

            if (i > 0)
                ImGui::SameLine(omegaWidth * i + spacing * i);

            ImGui::SetNextItemWidth(omegaWidth);
            ImGui::InputScalar((std::string("##Omega") + std::to_string(i)).c_str(),
                               ImGuiDataType_Double, &config.omegaBNB[i], nullptr, nullptr, "%.8f");
        }
    }
};

REGISTER_TAB(InitialStateTab, "Settings", "Initial State");