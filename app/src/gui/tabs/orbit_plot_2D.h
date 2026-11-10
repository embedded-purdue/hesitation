#pragma once

#include <string>
#include <utility>

#include "../tab.h"
#include "gui/tab_registry.h"
#include "imgui.h"
#include "implot.h"

class OrbitTab2D : public ITab {
  public:
    explicit OrbitTab2D(std::string instanceId = "0")
        : ITab("2D Orbit Plot", std::move(instanceId)) {}

  protected:
    void OnRender() override {
        // Enforce 1:1 aspect ratio and hide the legend (gridlines remain ENABLED)
        ImPlotFlags plotFlags = ImPlotFlags_Equal | ImPlotFlags_NoLegend;

        ImPlotAxisFlags axisFlags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;

        // Lock axis scale centered at origin (0,0) from -2.0 to +2.0
        float systemScale = 2.0f;
        ImPlot::SetNextAxisLimits(ImAxis_X1, -systemScale, systemScale, ImGuiCond_Always);
        ImPlot::SetNextAxisLimits(ImAxis_Y1, -systemScale, systemScale, ImGuiCond_Always);

        if (ImPlot::BeginPlot("##InterplanetaryTrajectory", ImVec2(-1, -1), plotFlags)) {
            // Passing "" for axis labels hides the titles cleanly across all ImPlot versions
            ImPlot::SetupAxes("", "", axisFlags, axisFlags);

            // Sun
            double sunX = 0.0, sunY = 0.0;
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 12.0f, ImVec4(1.0f, 0.85f, 0.0f, 1.0f));
            ImPlot::PlotScatter("Sun", &sunX, &sunY, 1);

            // 2. Earth Placeholder (1.0, 0.0)
            double earthX = 1.0, earthY = 0.0;
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6.0f, ImVec4(0.2f, 0.6f, 1.0f, 1.0f));
            ImPlot::PlotScatter("Earth", &earthX, &earthY, 1);

            // 3. Mars Placeholder (-1.2, 0.9)
            double marsX = -1.2, marsY = 0.9;
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 5.0f, ImVec4(0.9f, 0.3f, 0.2f, 1.0f));
            ImPlot::PlotScatter("Mars", &marsX, &marsY, 1);

            // 4. Spacecraft Placeholder (0.4, 0.3)
            double scX = 0.4, scY = 0.3;
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 5.0f, ImVec4(0.9f, 0.9f, 0.9f, 1.0f));
            ImPlot::PlotScatter("Spacecraft", &scX, &scY, 1);

            ImPlot::EndPlot();
        }
    }
};

REGISTER_TAB(OrbitTab2D, "Visualization", "2D Orbit Plot");