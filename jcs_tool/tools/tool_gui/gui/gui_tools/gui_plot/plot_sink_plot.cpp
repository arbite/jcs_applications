// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "plot_sink_plot.h"
#include "imgui.h"
#include <cmath>
#include <iostream>
#include "helpers.h"
#include "tool_gui_settings.h"

plot_sink_plot::plot_sink_plot(std::string const& node_name, std::string const& name, std::string const& units, float* val_ptr) :
    plot_sink(node_name, name),
    buffer_(tool_gui_settings::signal_plot_max_buffer_length)
{
    val_ptr_ = val_ptr;
    units_ = units;

    history_ = 10.0f;
    t_ = 0.0f;
    max_ = 0.0f;
    min_ = 0.0f;

    ave_time_ = 1.0f;
    average_ = 0.0f;
    average_count_ = 0;
}

void plot_sink_plot::update() {
    // Plot column
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    // Update data
    float value = (float)(*val_ptr_);

    // Imgui internally hashes the text in button to generate an id
    // But! It needs a unique id. Generating heaps of "internal" labelled buttons
    // will generate a lot of not unique ids. Push "this" and use that to generate a hash
    ImGui::PushID(this);

    // Controls above each plot
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);

    if (ImGui::Button("Min/max reset")) {
        min_ = value;
        max_ = value;
    }
    ImGui::SameLine();
    // ImGui::SliderFloat("History", &history_, 1, 30, "%.1f s");
    ImGui::PushItemWidth(100);
    ImGui::DragFloat("History", &history_, 1.0f, 1.0f, 30.0f, "%.1fs");
    ImGui::PopItemWidth();

    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::DragFloat("Average time", &ave_time_, 1.0f, 1.0f, 10.0f, "%.1fs");
    ImGui::PopItemWidth();

    ImGui::PopItemWidth();

    if (value < min_) { min_ = value; }
    if (value > max_) { max_ = value; }

    t_ += ImGui::GetIO().DeltaTime;
    buffer_.add_point(t_, value);

    // Find time range to compute average over, then compute average
    average_range_.Max = t_;
    average_range_.Min = t_ - ave_time_;
    average_count_ = 0;
    double ave = 0.0;
    for (int i=0; i<buffer_.data_.size(); ++i) {
        if (average_range_.Contains(buffer_.data_[i].x)) {
            ave += buffer_.data_[i].y;
            average_count_++;
        }
    }
    if (average_count_ > 0) {
        average_ = (float)(ave / (double)average_count_);
    }

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels | ImPlotFlags_NoFrame | 
                                   ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoGridLines;

    // remove padding between plot and frame
    ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0,0));
    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 80))) {
        ImPlot::SetupLegend(ImPlotLocation_East, ImPlotLegendFlags_Outside);
        ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ - history_, t_, ImGuiCond_Always);

        // Drive Y axis limits to max / min with a bit o padding
        float padding = (max_ - min_) * 0.1f;
        if (padding < 0.00f) { padding = 1.0f; }
        ImPlot::SetupAxisLimits(ImAxis_Y1, min_ - padding, max_ + padding, ImGuiCond_Always);

        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::PlotLine("##", &buffer_.data_[0].x, &buffer_.data_[0].y, buffer_.data_.size(), 0, buffer_.offset_, 2*sizeof(float));
        ImPlot::EndPlot();
    }

    // Information column
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%s", node_name_.c_str());
    ImGui::Text("%s", name_.c_str());
    ImGui::Text("Val: %.3f", value);
    ImGui::Text("Ave: %.3f", average_);
    ImGui::Text("Max: %.3f", max_);
    ImGui::Text("Min: %.3f", min_);

    ImGui::PopID();
}