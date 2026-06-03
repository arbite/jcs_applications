// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_icmu_calibrate.h"
#include <iostream>
#include "jcs_user_external.h"
#include "imgui_helpers.h"

//////////////////////////////////////////////////////////////////////
gui_icmu_calibrate::gui_icmu_calibrate(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("iC-MU Calibration", host, gui_if, target_device),
    signal_out_master_raw_("None", 0), signal_out_nonius_raw_("None", 0),
    is_ready_(false), can_start_(false),
    samples_max_(20000),
    tab_hand_(gui_if),
    tab_driven_(host, gui_if),
    active_tab_(0)
{
}

int gui_icmu_calibrate::startup() {
    // Configure line storage
    u16_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::uint16_s, 0));

    can_start_ = true;

    required_output_signal_names_ = { target_device_+"::icmu_master_raw",
                                      target_device_+"::icmu_nonius_raw" };

    // Size core buffers
    core_.resize_buffers(samples_max_);

    // Driven tab startup
    tab_driven_.startup();

    return jcs::RET_OK;
}

int gui_icmu_calibrate::ready_test() {
    can_start_ = true;
    if (helpers::output_signals_check(gui_if_->get_u16_output_signal_names(), &required_output_signal_names_) == false) {
        can_start_ = false;
    }

    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_, "icmu_mode", "icmu_mode_calibrate"), "Parameter failed: icmu_mode" )
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "icmu_start"), "Parameter failed: icmu_start" )

    if (can_start_) {
        is_ready_ = true;
    }
    helpers::sleep_ms(500);
    return jcs::RET_OK;
}

int gui_icmu_calibrate::step_rt() {
    host_->sig_output_get_rt(0, &u16_output_signal_store_);

    uint16_t master_raw = u16_output_signal_store_[signal_out_master_raw_.index_];
    uint16_t nonius_raw = u16_output_signal_store_[signal_out_nonius_raw_.index_];

    // Route to active tab
    if (active_tab_ == 0) {
        tab_hand_.step_rt(core_, master_raw, nonius_raw, host_->base_frequency_get());
    } else {
        tab_driven_.step_rt(core_, master_raw, nonius_raw);
    }

    return jcs::RET_OK;
}

int gui_icmu_calibrate::render() {
    ImGui::Text("iC-MU Calibration");
    ImGui::Separator();
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure correct configuration is used.");
    ImGui::Text("- Ensure motor configuration has parameter estimator_0_theta_passthrough set to yes.");
    ImGui::Text("- Ensure device is not temperature clamped.");
    ImGui::Text("- For hand calibration: ensure encoder target can be rotated freely.");
    ImGui::Text("- For driven calibration: ensure motor controllers are tuned and encoder zeroed.");
    ImGui::Separator();

    // Signal checks
    ImGui::Text("Required output signals:");
    ImGui::Text("- iC-MU master raw: `icmu_master_raw`");
    ImGui::Text("- iC-MU nonius raw: `icmu_nonius_raw`");

    ImGui::Separator();
    helpers::output_signals_check(gui_if_->get_u16_output_signal_names(), &required_output_signal_names_);

    ImGui::Separator();
    if (ImGui::Button("Click to make device ready for tests!")) {
        if (ready_test() != jcs::RET_OK) {
            return jcs::RET_OK;
        }
    }

    if (!can_start_) {
        ImGui::Text("Can't start! Most likely missing signal.");
    }
    {
        ImGuiDisabled ui_disabled(!is_ready_);
        // Shared signal selection
        ImGui::Separator();
        ImGui::Text("Configure encoder signals");
        helpers::combo_select("icmu_master_raw source", gui_if_->get_u16_output_signal_names(), &signal_out_master_raw_);
        helpers::combo_select("icmu_nonius_raw source", gui_if_->get_u16_output_signal_names(), &signal_out_nonius_raw_);
        // Shared sample count
        ImGui::Separator();
        {
            int value = samples_max_;
            if (ImGui::InputInt("Number of samples", &value, 1000, 5000, ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0 && value <= 100000) {
                    samples_max_ = value;
                    core_.resize_buffers(samples_max_);
                }
            }
        }
        // Live signal display
        ImGui::Separator();
        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                                             ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;
        if (ImGui::BeginTable("LiveSignals", 2, table_flags)) {
            ImGui::TableSetupColumn("Signal", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Value",  ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("icmu_master_raw");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%u", u16_output_signal_store_[signal_out_master_raw_.index_]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("icmu_nonius_raw");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%u", u16_output_signal_store_[signal_out_nonius_raw_.index_]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("Sample count");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%d / %d", core_.samples_idx(), core_.samples_max());

            ImGui::EndTable();
        }
        {
            float progress = (float)core_.samples_idx() / (float)core_.samples_max();
            char buf[32];
            sprintf(buf, "%d/%d", core_.samples_idx(), core_.samples_max());
            ImGui::ProgressBar(progress, ImVec2(0.0f, 0.0f), buf);
            ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::Text("Capture progress");
        }
        // Tab bar for calibrators
        ImGui::Separator();
        if (ImGui::BeginTabBar("CalibrationTabs")) {
            if (ImGui::BeginTabItem("Hand")) {
                active_tab_ = 0;
                ImGui::Separator();
                tab_hand_.render_parameters();
                ImGui::Separator();
                tab_hand_.render_controls(core_);
                ImGui::Separator();
                tab_hand_.render_state();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Driven")) {
                active_tab_ = 1;
                ImGui::Separator();
                tab_driven_.render_parameters();
                ImGui::Separator();
                tab_driven_.render_controls(core_);
                ImGui::Separator();
                tab_driven_.render_state();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        // Calibration
        ImGui::Separator();
        {
            bool has_samples = core_.samples_idx() > 0;
            ImGuiDisabled can_calib(!has_samples);
            if (ImGui::Button("Do calibration")) {
                core_.do_calibration(host_, target_device_);
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset calibration")) {
                is_ready_ = false;
                PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "icmu_soft_reset"), "Parameter failed: icmu_soft_reset" )
                core_.clear_buffers();
            }
        }
        // Results
        core_.render_results();
        core_.render_write_to_file(target_device_);
        // core_.render_write_to_device(host_, target_device_);

        // Plots (raw data + nonius phase error before/after)
        core_.render_plots();
    }

    return jcs::RET_OK;
}