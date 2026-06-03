// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "icmu_calibrate_hand.h"
#include "imgui_helpers.h"
#include "imgui.h"

icmu_calibrate_hand::icmu_calibrate_hand(gui_interface* gui_if)
    : state_(state::standby_s),
      wait_time_s_(5.0f),
      wait_movement_tick_(0),
      wait_movement_tick_max_(0),
      gui_if_(gui_if)
{
}

void icmu_calibrate_hand::step_rt(icmu_calibration_core& core,
                                   uint16_t master_raw, uint16_t nonius_raw,
                                   int base_frequency)
{
    switch (state_) {
        default:
        case state::standby_s:
            break;

        case state::initialise_s:
            core.clear_buffers();
            wait_movement_tick_ = 0;
            wait_movement_tick_max_ = static_cast<int>(wait_time_s_ * static_cast<float>(base_frequency));
            state_ = state::wait_movement_s;
            break;

        case state::wait_movement_s:
            wait_movement_tick_++;
            if (wait_movement_tick_ >= wait_movement_tick_max_) {
                state_ = state::moving_s;
            }
            break;

        case state::moving_s:
            if (!core.record_sample(master_raw, nonius_raw)) {
                state_ = state::finish_s;
            }
            break;

        case state::finish_s:
            // state_ = state::standby_s;
            break;
    }
}

// UI
void icmu_calibrate_hand::render_parameters() {
    ImGui::Text("Hand rotation parameters");
    {
        float value = wait_time_s_;
        if (ImGui::InputFloat("Wait time before sampling (s)", &value, 0.5f, 1.0f, "%.1f", ImGuiInputTextFlags_EscapeClearsAll)) {
            if (value >= 0.0f) { wait_time_s_ = value; }
        }
    }
}

void icmu_calibrate_hand::render_controls(icmu_calibration_core& core) {
    ImGui::Text("Rotate the encoder target smoothly through at least one full revolution.");

    // Transition from finish here so we can stop host
    if (state_ != state::standby_s) {
        if (state_ == state::finish_s) {
            gui_if_->stop();
            state_ = state::standby_s;
        }
    }
    {
        ImGuiDisabled running_dis(state_ != state::standby_s);
        if (ImGui::Button("Start hand calibration")) {
            if (gui_if_->start() != jcs::RET_OK) { return; }
            helpers::sleep_ms(500);
            state_ = state::initialise_s;
        }
    }
    ImGui::SameLine();
    {
        ImGuiDisabled cancel_dis(state_ == state::standby_s);
        if (ImGui::Button("Cancel##hand")) {
            gui_if_->stop();
            state_ = state::standby_s;
            core.clear_buffers();
        }
    }
}

void icmu_calibrate_hand::render_state() {
    ImGui::Text("State: ");
    ImGui::SameLine();
    switch (state_) {
        default:
        case state::standby_s:       ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Standby");       break;
        case state::initialise_s:    ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Initialising");  break;
        case state::wait_movement_s: ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.8f, 1.0f), "Waiting (%d/%d)",
                                        wait_movement_tick_, wait_movement_tick_max_);                      break;
        case state::moving_s:        ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Sampling");      break;
        case state::finish_s:        ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Finishing");     break;
    }
}
