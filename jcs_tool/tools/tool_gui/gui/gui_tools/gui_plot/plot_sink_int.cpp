// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "plot_sink_int.h"
#include "imgui.h"
#include "tool_gui_settings.h"
#include <cmath>

plot_sink_int::plot_sink_int(std::string const& node_name, std::string const& name, uint32_t* val_ptr) : 
    plot_sink(node_name, name),
    buffer_(tool_gui_settings::signal_plot_max_buffer_length)
{
    val_prt_u32_ = val_ptr;
    bit_width_ = width::uint_32;
    reset();
}
plot_sink_int::plot_sink_int(std::string const& node_name, std::string const& name, uint16_t* val_ptr) : 
    plot_sink(node_name, name),
    buffer_(tool_gui_settings::signal_plot_max_buffer_length)
{
    val_prt_u16_ = val_ptr;
    bit_width_ = width::uint_16;
    reset();
}
plot_sink_int::plot_sink_int(std::string const& node_name, std::string const& name, uint8_t* val_ptr) : 
    plot_sink(node_name, name),
    buffer_(tool_gui_settings::signal_plot_max_buffer_length)
{
    val_prt_u8_ = val_ptr;
    bit_width_ = width::uint_8;
    reset();
}

void plot_sink_int::reset() {
    history_ = 10.0f;
    t_ = 0.0f;
    max_ = 0.0f;
    min_ = 0.0f;
}

// Currently just outputting as a plot
void plot_sink_int::update() {
    // Update data
    float value;
    switch (bit_width_) {
        default:
            return;
        case width::uint_32:
            value= (float)(*val_prt_u32_);
            break;
        case width::uint_16:
            value = (float)(*val_prt_u16_);
            break;
        case width::uint_8:
            value = (float)(*val_prt_u8_);
            break;
    }

    // Plot column
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);

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
    ImGui::SliderFloat("History", &history_, 1, 30, "%.1f s");
    
    ImGui::PopItemWidth();

    if (value < min_) {
        min_ = value;
    }
    if (value > max_) {
        max_ = value;
    }

    t_ += ImGui::GetIO().DeltaTime;

    buffer_.add_point(t_, value);

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels | ImPlotFlags_NoFrame | ImPlotAxisFlags_AutoFit | 
                                   ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoGridLines;;

    // remove padding between plot and frame
    ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0,0));
    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 80))) {
        ImPlot::SetupLegend(ImPlotLocation_East, ImPlotLegendFlags_Outside);
        ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ - history_, t_, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 10);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::PlotLine("##", &buffer_.data_[0].x, &buffer_.data_[0].y, buffer_.data_.size(), 0, buffer_.offset_, 2*sizeof(float));
        ImPlot::EndPlot();
    }

    // Information column
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%s", node_name_.c_str());
    ImGui::Text("%s", name_.c_str());
    ImGui::Text("Val: %.3f", value);
    ImGui::Text("Max: %.3f", max_);
    ImGui::Text("Min: %.3f", min_);

    ImGui::PopID();

}
