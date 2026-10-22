
#pragma once

#include <string>
#include <utility>
#include <ctime>
#include <cmath>
#include <algorithm>

#include "../tab.h"
#include "gui/tab_registry.h"
#include "imgui.h"

class InitialStateTab : public ITab {
  public:
    explicit InitialStateTab(std::string instanceId = "0")
        : ITab("Initial State", std::move(instanceId)) {}

  protected:
    void OnRender() override {
        constexpr double PI         = 3.14159265358979323846;
        constexpr double DEG_TO_RAD = PI / 180.0;
        constexpr double EARTH_MU   = 3.986004418e14;

        ImGui::TextDisabled("Applied on reset");
        ImGui::Spacing();

        // Run Type
        static int runType     = 0;
        const char* runTypes[] = {"Regular", "Orbit Sim Test"};

        ImGui::Text("Run Type");
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::Combo("##RunType", &runType, runTypes, IM_ARRAYSIZE(runTypes));

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Epoch: UTC date/time stored as Unix seconds.
        static double epoch = 1700000000.0;
        static int year = 2023, month = 11, day = 14;
        static int hour = 22, minute = 13, second = 20;
        static int millisecond       = 0;
        static bool epochInitialized = false;
        static double lastEpoch      = 0.0;

        auto epochToFields = [&]() {
            const double whole          = std::floor(epoch);
            const std::time_t timestamp = static_cast<std::time_t>(whole);
            const std::tm* result       = std::gmtime(&timestamp);

            if (!result)
                return;

            year   = result->tm_year + 1900;
            month  = result->tm_mon + 1;
            day    = result->tm_mday;
            hour   = result->tm_hour;
            minute = result->tm_min;
            second = result->tm_sec;

            millisecond = static_cast<int>(std::round((epoch - whole) * 1000.0));

            if (millisecond >= 1000) {
                millisecond = 0;
                epoch += 1.0;
            }

            lastEpoch        = epoch;
            epochInitialized = true;
        };

        if (!epochInitialized || epoch != lastEpoch)
            epochToFields();

        ImGui::Text("Epoch");
        ImGui::TextDisabled("UTC date and time | Unix seconds");
        ImGui::Spacing();

        const float spacing   = ImGui::GetStyle().ItemSpacing.x;
        const float available = ImGui::GetContentRegionAvail().x;
        bool epochEdited      = false;

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

            epoch     = static_cast<double>(t) + static_cast<double>(millisecond) / 1000.0;
            lastEpoch = epoch;
        }

        // Translational State
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        static int stateMode     = 0;
        const char* stateModes[] = {"Orbital Elements", "Cartesian Position / Velocity"};

        ImGui::Text("Translational State");
        ImGui::TextDisabled("Orbital elements are converted to inertial state vectors");
        ImGui::Spacing();

        // Fit the dropdown to the currently available width.
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::Combo("##InputMode", &stateMode, stateModes, IM_ARRAYSIZE(stateModes));

        // Inertial Cartesian state: meters and meters/second.
        static double position[3] = {0.0, 0.0, 0.0};
        static double velocity[3] = {0.0, 0.0, 0.0};

        // Classical orbital elements.
        static double semiMajorAxis = 6978000.0;
        static double eccentricity  = 0.01;
        static double inclination   = 63.3;
        static double raan          = 88.2;
        static double argPeriapsis  = 347.8;
        static double trueAnomaly   = 135.3;

        if (stateMode == 0) {
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
            ImGui::InputScalar("##COEA", ImGuiDataType_Double, &semiMajorAxis, nullptr, nullptr,
                               "%.3f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COEE", ImGuiDataType_Double, &eccentricity, nullptr, nullptr,
                               "%.8f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COEI", ImGuiDataType_Double, &inclination, nullptr, nullptr,
                               "%.5f");

            ImGui::Spacing();

            ImGui::TextUnformatted("RAAN (deg)");
            ImGui::SameLine(w3 + spacing);
            ImGui::TextUnformatted("Argument of periapsis (deg)");
            ImGui::SameLine(w3 * 2.0f + spacing * 2.0f);
            ImGui::TextUnformatted("True anomaly (deg)");

            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COERAAN", ImGuiDataType_Double, &raan, nullptr, nullptr, "%.5f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COEArgPeri", ImGuiDataType_Double, &argPeriapsis, nullptr,
                               nullptr, "%.5f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##COETrueAnomaly", ImGuiDataType_Double, &trueAnomaly, nullptr,
                               nullptr, "%.5f");

            // Convert elliptical Earth-centered COEs to inertial state.
            const bool validOrbit =
                semiMajorAxis > 0.0 && eccentricity >= 0.0 && eccentricity < 1.0;

            if (validOrbit) {
                const double inc = inclination * DEG_TO_RAD;
                const double O   = raan * DEG_TO_RAD;
                const double w   = argPeriapsis * DEG_TO_RAD;
                const double f   = trueAnomaly * DEG_TO_RAD;

                const double p     = semiMajorAxis * (1.0 - eccentricity * eccentricity);
                const double denom = 1.0 + eccentricity * std::cos(f);

                if (p > 0.0 && denom > 0.0) {
                    const double radius     = p / denom;
                    const double speedScale = std::sqrt(EARTH_MU / p);

                    const double rpX = radius * std::cos(f);
                    const double rpY = radius * std::sin(f);
                    const double vpX = -speedScale * std::sin(f);
                    const double vpY = speedScale * (eccentricity + std::cos(f));

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

                    position[0] = r11 * rpX + r12 * rpY;
                    position[1] = r21 * rpX + r22 * rpY;
                    position[2] = r31 * rpX + r32 * rpY;

                    velocity[0] = r11 * vpX + r12 * vpY;
                    velocity[1] = r21 * vpX + r22 * vpY;
                    velocity[2] = r31 * vpX + r32 * vpY;
                }
            }
        } else {
            ImGui::Spacing();

            ImGui::Text("Position");
            ImGui::TextDisabled("Inertial frame N | meters (m)");
            ImGui::Spacing();
            ImGui::Spacing();

            const float w3 =
                std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f);

            ImGui::TextUnformatted("X (m)");
            ImGui::SameLine(w3 + spacing);
            ImGui::TextUnformatted("Y (m)");
            ImGui::SameLine(w3 * 2.0f + spacing * 2.0f);
            ImGui::TextUnformatted("Z (m)");

            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##PosX", ImGuiDataType_Double, &position[0], nullptr, nullptr,
                               "%.6f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##PosY", ImGuiDataType_Double, &position[1], nullptr, nullptr,
                               "%.6f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##PosZ", ImGuiDataType_Double, &position[2], nullptr, nullptr,
                               "%.6f");

            ImGui::Spacing();
            ImGui::Spacing();

            ImGui::Text("Velocity");
            ImGui::TextDisabled("Inertial frame N | meters per second (m/s)");
            ImGui::Spacing();
            ImGui::Spacing();

            ImGui::TextUnformatted("Vx (m/s)");
            ImGui::SameLine(w3 + spacing);
            ImGui::TextUnformatted("Vy (m/s)");
            ImGui::SameLine(w3 * 2.0f + spacing * 2.0f);
            ImGui::TextUnformatted("Vz (m/s)");

            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##VelX", ImGuiDataType_Double, &velocity[0], nullptr, nullptr,
                               "%.6f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##VelY", ImGuiDataType_Double, &velocity[1], nullptr, nullptr,
                               "%.6f");
            ImGui::SameLine(0.0f, spacing);
            ImGui::SetNextItemWidth(w3);
            ImGui::InputScalar("##VelZ", ImGuiDataType_Double, &velocity[2], nullptr, nullptr,
                               "%.6f");
        }

        ImGui::Spacing();

        ImGui::TextDisabled("Position (km): X %.3f | Y %.3f | Z %.3f", position[0] / 1000.0,
                            position[1] / 1000.0, position[2] / 1000.0);

        ImGui::TextDisabled("Velocity (km/s): X %.6f | Y %.6f | Z %.6f", velocity[0] / 1000.0,
                            velocity[1] / 1000.0, velocity[2] / 1000.0);

        // Attitude: modified Rodrigues parameters.
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Attitude");
        ImGui::TextDisabled("MRP components | Dimensionless");
        ImGui::Spacing();
        ImGui::Spacing();

        static double sigmaBN[3] = {0.0, 0.0, 0.0};

        const float attitudeWidth =
            std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f);

        ImGui::TextUnformatted("Sigma 1");
        ImGui::SameLine(attitudeWidth + spacing);
        ImGui::TextUnformatted("Sigma 2");
        ImGui::SameLine(attitudeWidth * 2.0f + spacing * 2.0f);

        ImGui::TextUnformatted("Sigma 3");

        ImGui::SetNextItemWidth(attitudeWidth);
        ImGui::InputScalar("##Sigma1", ImGuiDataType_Double, &sigmaBN[0], nullptr, nullptr, "%.8f");
        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(attitudeWidth);
        ImGui::InputScalar("##Sigma2", ImGuiDataType_Double, &sigmaBN[1], nullptr, nullptr, "%.8f");
        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(attitudeWidth);
        ImGui::InputScalar("##Sigma3", ImGuiDataType_Double, &sigmaBN[2], nullptr, nullptr, "%.8f");

        // Body angular velocity.
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Angular Velocity");
        ImGui::TextDisabled("Body frame | Radians per second (rad/s)");
        ImGui::Spacing();
        ImGui::Spacing();

        static double omegaBNB[3] = {0.0, 0.0, 0.0};

        const float omegaWidth =
            std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f);

        ImGui::TextUnformatted("X (rad/s)");
        ImGui::SameLine(omegaWidth + spacing);
        ImGui::TextUnformatted("Y (rad/s)");
        ImGui::SameLine(omegaWidth * 2.0f + spacing * 2.0f);
        ImGui::TextUnformatted("Z (rad/s)");

        ImGui::SetNextItemWidth(omegaWidth);
        ImGui::InputScalar("##OmegaX", ImGuiDataType_Double, &omegaBNB[0], nullptr, nullptr,
                           "%.8f");
        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(omegaWidth);
        ImGui::InputScalar("##OmegaY", ImGuiDataType_Double, &omegaBNB[1], nullptr, nullptr,
                           "%.8f");
        ImGui::SameLine(0.0f, spacing);
        ImGui::SetNextItemWidth(omegaWidth);
        ImGui::InputScalar("##OmegaZ", ImGuiDataType_Double, &omegaBNB[2], nullptr, nullptr,
                           "%.8f");
    }
};

REGISTER_TAB(InitialStateTab, "Settings", "Initial State");
