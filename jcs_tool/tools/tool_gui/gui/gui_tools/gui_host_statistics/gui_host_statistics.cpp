
// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_statistics.h"
#include "imgui_stdlib.h"
#include "implot.h"
#include <cmath>
#include <iostream>
#include "helpers.h"
#include "jcs_user_external.h"

#include "jcs_dev_motor_controller.h"

gui_host_statistics::gui_host_statistics(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Statistics", host, gui_if, target_device),
    to_mean_buffer_(2000),
    thread_timestamp_buffer_(2000),
    start_cycle_time_old_ns_(0),
    cycle_buffer_(2000),
    data_exchange_buffer_(2000),
    thread_offset_controller_error_buffer_(2000),
    thread_offset_controller_correction_buffer_(2000),
    max_history_s_(30),
    fit_x_(true), fit_y_(true)
{}

int gui_host_statistics::startup() {
    int new_buffer_size = max_history_s_ * (int)host_->base_frequency_get();

    cycle_buffer_.update_size(new_buffer_size);
    data_exchange_buffer_.update_size(new_buffer_size);
    to_mean_buffer_.update_size(new_buffer_size);
    thread_timestamp_buffer_.update_size(new_buffer_size);

    thread_offset_controller_error_buffer_.update_size(new_buffer_size);
    thread_offset_controller_correction_buffer_.update_size(new_buffer_size);

    t_start_ns_ = (double)jcs::external::time_now_ns();
    return jcs::RET_OK;
}

int gui_host_statistics::step_rt() {
    return jcs::RET_OK;
}

int gui_host_statistics::step_rt_always() {
    t_s_ = (float)(((double)jcs::external::time_now_ns() - t_start_ns_)*1e-9);
    // Get stats
    timing_ = host_->statistics_timing_get();
    health_ = host_->statistics_health_get();

    // Populate plots
    // Note: Not caring about locking or anything here. Just go for gold
    cycle_buffer_.add_point(t_s_, (float)timing_.total_cycle_time_ns/1000.0f);
    data_exchange_buffer_.add_point(t_s_, (float)timing_.data_exchange_time_ns/1000.0f);
    // Thread wakup delta
    // Skip the first sample: start_cycle_time_old_ns_ has nothing in it yet, so the
    // delta would be the raw clock value.
    if (start_cycle_time_old_ns_ != 0) {
        thread_timestamp_buffer_.add_point(t_s_, (float)(timing_.start_cycle_time_ns - start_cycle_time_old_ns_)/1000.0f);
    }
    start_cycle_time_old_ns_ = timing_.start_cycle_time_ns;

    to_mean_buffer_.add_point(t_s_, (float)health_.thread.mean);

    transport_ = host_->statistics_transport_get();
    // Thread offset PI controller
    thread_offset_controller_error_buffer_.add_point(t_s_, (float)transport_.thread_offset_error_ns/1000.0f);
    thread_offset_controller_correction_buffer_.add_point(t_s_, (float)transport_.thread_offset_correction_ns/1000.0f);

    return jcs::RET_OK;
}

int gui_host_statistics::render() {

    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

    ImGui::Text("Times");
    if (ImGui::BeginTable("Times", 2, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Cycle Time (us)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)timing_.total_cycle_time_ns/1000.0f);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Data Exchange Time (us)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)timing_.data_exchange_time_ns/1000.0f);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Cycle - Data Exchange Time difference (us)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", fabsf((float)(timing_.data_exchange_time_ns-timing_.total_cycle_time_ns)/1000.0f));

        ImGui::EndTable();
    }

    ImGui::Checkbox("Fit T axis", &fit_x_);
    ImGui::SameLine();
    ImGui::Checkbox("Fit Y axis", &fit_y_);

    ImPlotAxisFlags x_flags = fit_x_ ? ImPlotAxisFlags_AutoFit : 0;
    ImPlotAxisFlags y_flags = fit_y_ ? ImPlotAxisFlags_AutoFit : 0;

    if (ImPlot::BeginPlot("Times plot")) {
        ImPlot::SetupAxes("t", nullptr, x_flags, y_flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_s_ - max_history_s_, t_s_, ImGuiCond_Once);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

        cycle_buffer_.plot_line("Cycle Time");
        data_exchange_buffer_.plot_line("Data Exchange Time");

        ImPlot::EndPlot();
    }

    ImGui::Text("Pending Sequences Overrun");
    if (ImGui::BeginTable("Pending", 4, table_flags)) {
        ImGui::TableSetupColumn("Rate",                ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Overrun count",       ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Last timestamp (us)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Counts percentage",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        // Full rate
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Full Rate");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", (int)health_.pending_sequences_fullrate.overrun_count);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3fus", (float)health_.pending_sequences_fullrate.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%7.3f", (float)health_.pending_sequences_fullrate.counts_percent);
        // Subrate 0
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Sub rate - 0");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", (int)health_.pending_sequences_subrate_0.overrun_count);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3fus", (float)health_.pending_sequences_subrate_0.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%7.3f", (float)health_.pending_sequences_subrate_0.counts_percent);
        // Subrate 1
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Sub rate - 1");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", (int)health_.pending_sequences_subrate_1.overrun_count);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3fus", (float)health_.pending_sequences_subrate_1.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%7.3f", (float)health_.pending_sequences_subrate_1.counts_percent);
        ImGui::EndTable();
    }

    ImGui::Text("Transport Overrun");
    if (ImGui::BeginTable("Transport", 3, table_flags)) {
        ImGui::TableSetupColumn("Overrun count",       ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Last timestamp (us)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Counts percentage",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%u", (int)health_.transport.overrun_count);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.transport.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3f", (float)health_.transport.counts_percent);
        ImGui::EndTable();
    }

    ImGui::Text("Base Rate Jitter Overrun");
    if (ImGui::BeginTable("Thread overrun", 3, table_flags)) {
        ImGui::TableSetupColumn("Overrun count",       ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Last timestamp (us)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Counts percentage",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%u", (int)health_.thread.overrun_count);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.thread.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3f", (float)health_.thread.counts_percent);
        ImGui::EndTable();
    }
    ImGui::Text("Cycle Period Statistics");
    if (ImGui::BeginTable("Thread stats", 2, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Mean (ns)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.thread.mean);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Variance (ns^2)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.thread.variance);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);        
        ImGui::Text("Std Deviation (ns)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.thread.std_dev);
        ImGui::EndTable();
    }

    // More pretty plots
    if (ImPlot::BeginPlot("Thread cycle time plot")) {
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);

        ImPlot::SetupAxes("t (s)", "cycle time (ns)", x_flags, y_flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_s_ - max_history_s_, t_s_, ImGuiCond_Once);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

        to_mean_buffer_.plot_line("Mean");

        ImPlot::PopStyleVar();
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Thread wakeup delta plot")) {
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);

        ImPlot::SetupAxes("t (s)", "delta (us)", x_flags, y_flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_s_ - max_history_s_, t_s_, ImGuiCond_Once);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

        thread_timestamp_buffer_.plot_line("delta");

        ImPlot::PopStyleVar();
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Thread offset controller error plot")) {
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);

        ImPlot::SetupAxes("t (s)", "error (us)", x_flags, y_flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_s_ - max_history_s_, t_s_, ImGuiCond_Once);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

        thread_offset_controller_error_buffer_.plot_line("Error");

        ImPlot::PopStyleVar();
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Thread offset controller correction plot")) {
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);

        ImPlot::SetupAxes("t (s)", "correction (us)", x_flags, y_flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_s_ - max_history_s_, t_s_, ImGuiCond_Once);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

        thread_offset_controller_correction_buffer_.plot_line("Correction");

        ImPlot::PopStyleVar();
        ImPlot::EndPlot();
    }
    return jcs::RET_OK;
}
