// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "icmu_calibrate_driven.h"
#include <iostream>
#include <cmath>
#include "imgui.h"
#include "imgui_helpers.h"
#include "jcs_user_external.h"

icmu_calibrate_driven::icmu_calibrate_driven(jcs::jcs_host* host, gui_interface* gui_if)
    : host_(host),
      gui_if_(gui_if),
      state_(state::standby_s),
      drive_mode_(drive_mode::free_s),
      signal_in_source_w_m_("None", 0),
      signal_out_source_i_mot_("None", 0),
      velocity_ramp_(1.0 / static_cast<double>(host->base_frequency_get())),
      // Free mode defaults
      free_speed_rads_(6.28f),       // ~1 RPS
      free_duration_s_(10.0f),
      free_ramp_time_s_(2.0f),
      free_tick_(0),
      free_tick_max_(0),
      // Limited mode defaults
      limited_speed_rads_(3.14f),    // ~0.5 RPS
      limited_ramp_time_s_(1.0f),
      limited_current_threshold_(2.0f),
      limited_debounce_ticks_(50),
      limited_debounce_count_(0),
      limited_passes_target_(4),
      limited_passes_done_(0),
      limited_timeout_s_(10.0f),
      limited_timeout_tick_(0),
      limited_timeout_tick_max_(0),
      limited_current_direction_(1.0f)
{
}

int icmu_calibrate_driven::startup() {
    f32_input_signal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    f32_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    required_input_signal_names_ = { "HOST::w_m" };

    return jcs::RET_OK;
}

void icmu_calibrate_driven::step_rt(icmu_calibration_core& core,
                                     uint16_t master_raw, uint16_t nonius_raw)
{
    // Always read output signals for live display / endstop detection
    host_->sig_output_get_rt(0, &f32_output_signal_store_);

    switch (state_) {
        default:
        case state::standby_s:
            break;

        // Common init
        case state::initialise_s:
        {
            core.clear_buffers();

            // Zero velocity command
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = 0.0f;
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            if (drive_mode_ == drive_mode::free_s) {
                // Ramp up to target speed
                velocity_ramp_.start(0.0f, free_speed_rads_, free_ramp_time_s_, 0.5f, 0.0f);
                state_ = state::free_ramp_up_s;
            } else {
                // Limited: ramp to first direction
                limited_passes_done_ = 0;
                limited_current_direction_ = 1.0f;
                limited_debounce_count_ = 0;
                velocity_ramp_.start(0.0f, limited_speed_rads_ * limited_current_direction_,
                                     limited_ramp_time_s_, 0.5f, 0.0f);
                state_ = state::limited_ramp_s;
            }
            break;
        }

        // Free mode
        case state::free_ramp_up_s:
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = velocity_ramp_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            // Record samples during ramp too
            core.record_sample(master_raw, nonius_raw);

            if (velocity_ramp_.is_done()) {
                free_tick_ = 0;
                free_tick_max_ = static_cast<int>(
                    free_duration_s_ * static_cast<float>(host_->base_frequency_get()));
                state_ = state::free_rotating_s;
            }
            break;

        case state::free_rotating_s:
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = free_speed_rads_;
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            core.record_sample(master_raw, nonius_raw);

            free_tick_++;
            if (free_tick_ >= free_tick_max_ || core.samples_idx() >= core.samples_max()) {
                // Ramp down
                velocity_ramp_.start(free_speed_rads_, 0.0f, free_ramp_time_s_, 0.5f, 0.0f);
                state_ = state::free_ramp_down_s;
            }
            break;

        case state::free_ramp_down_s:
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = velocity_ramp_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            if (velocity_ramp_.is_done()) {
                f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = 0.0f;
                host_->sig_input_set_rt(0, f32_input_signal_store_);
                state_ = state::finish_s;
            }
            break;

        // Limited mode
        case state::limited_ramp_s:
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = velocity_ramp_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            core.record_sample(master_raw, nonius_raw);

            if (velocity_ramp_.is_done()) {
                limited_timeout_tick_ = 0;
                limited_timeout_tick_max_ = static_cast<int>(
                    limited_timeout_s_ * static_cast<float>(host_->base_frequency_get()));
                limited_debounce_count_ = 0;
                state_ = state::limited_moving_s;
            }
            break;

        case state::limited_moving_s:
        {
            float target_speed = limited_speed_rads_ * limited_current_direction_;
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = target_speed;
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            core.record_sample(master_raw, nonius_raw);

            // Endstop detection: check |i_q| against threshold
            float i_q = f32_output_signal_store_[signal_out_source_i_mot_.index_];
            if (fabs(i_q) >= limited_current_threshold_) {
                limited_debounce_count_++;
            } else {
                limited_debounce_count_ = 0;
            }

            // Timeout check
            limited_timeout_tick_++;

            if (limited_debounce_count_ >= limited_debounce_ticks_) {
                state_ = state::limited_endstop_detected_s;
            } else if (limited_timeout_tick_ >= limited_timeout_tick_max_) {
                std::cout << "WARNING: Limited mode timeout reached without endstop detection.\n";
                state_ = state::limited_endstop_detected_s;
            }
            break;
        }

        case state::limited_endstop_detected_s:
        {
            limited_passes_done_++;
            std::cout << "Endstop detected. Pass " << limited_passes_done_
                      << "/" << limited_passes_target_ << "\n";

            if (limited_passes_done_ >= limited_passes_target_ ||
                core.samples_idx() >= core.samples_max()) {
                // Done — ramp to zero
                float current_speed = limited_speed_rads_ * limited_current_direction_;
                velocity_ramp_.start(current_speed, 0.0f, limited_ramp_time_s_, 0.5f, 0.0f);
                state_ = state::limited_ramp_reverse_s;
            } else {
                // Reverse direction
                float current_speed = limited_speed_rads_ * limited_current_direction_;
                limited_current_direction_ *= -1.0f;
                float next_speed = limited_speed_rads_ * limited_current_direction_;
                velocity_ramp_.start(current_speed, next_speed, limited_ramp_time_s_, 0.5f, 0.0f);
                state_ = state::limited_ramp_s;
            }
            break;
        }

        case state::limited_ramp_reverse_s:
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = velocity_ramp_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            if (velocity_ramp_.is_done()) {
                f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = 0.0f;
                host_->sig_input_set_rt(0, f32_input_signal_store_);
                state_ = state::finish_s;
            }
            break;

        // Finish
        case state::finish_s:
            // state_ = state::standby_s;
            break;
    }
}

void icmu_calibrate_driven::render_parameters() {
    ImGui::Text("Motor-driven calibration parameters");

    // Mode selection
    ImGui::Separator();
    if (ImGui::RadioButton("Free rotation (no endstops)", drive_mode_ == drive_mode::free_s)) {
        drive_mode_ = drive_mode::free_s;
    }
    if (ImGui::RadioButton("Limited rotation (with endstops)", drive_mode_ == drive_mode::limited_s)) {
        drive_mode_ = drive_mode::limited_s;
    }

    // Motor signal selection
    ImGui::Separator();
    ImGui::Text("Motor signals");
    helpers::combo_select("Velocity command input (w_m)", gui_if_->get_f32_input_signal_names(), &signal_in_source_w_m_);
    helpers::combo_select("Motor current output (i_mot)", gui_if_->get_f32_output_signal_names(), &signal_out_source_i_mot_);

    ImGui::Separator();

    if (drive_mode_ == drive_mode::free_s) {
        ImGui::Text("Free rotation");
        {
            float value = free_speed_rads_;
            if (ImGui::InputFloat("Speed (rad/s)", &value, 0.1f, 1.0f, "%.3f", ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0.0f) { free_speed_rads_ = value; }
            }
        }
        {
            float value = free_duration_s_;
            if (ImGui::InputFloat("Duration (s)", &value, 1.0f, 5.0f, "%.1f", ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0.0f) { free_duration_s_ = value; }
            }
        }
        {
            float value = free_ramp_time_s_;
            if (ImGui::InputFloat("Ramp time (s)", &value, 0.1f, 1.0f, "%.2f", ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0.0f) { free_ramp_time_s_ = value; }
            }
        }
    } else {
        ImGui::Text("Limited rotation (endstop bounce)");
        {
            float value = limited_speed_rads_;
            if (ImGui::InputFloat("Speed (rad/s)", &value, 0.1f, 1.0f, "%.3f", ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0.0f) { limited_speed_rads_ = value; }
            }
        }
        {
            float value = limited_ramp_time_s_;
            if (ImGui::InputFloat("Ramp time (s)", &value, 0.1f, 1.0f, "%.2f", ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0.0f) { limited_ramp_time_s_ = value; }
            }
        }
        {
            float value = limited_current_threshold_;
            if (ImGui::InputFloat("Current threshold |i_mot| (A)", &value, 0.1f, 1.0f, "%.3f", ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0.0f) { limited_current_threshold_ = value; }
            }
        }
        {
            int value = limited_debounce_ticks_;
            if (ImGui::InputInt("Debounce ticks", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0) { limited_debounce_ticks_ = value; }
            }
        }
        {
            int value = limited_passes_target_;
            if (ImGui::InputInt("Number of passes", &value, 1, 2, ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0) { limited_passes_target_ = value; }
            }
        }
        {
            float value = limited_timeout_s_;
            if (ImGui::InputFloat("Timeout per sweep (s)", &value, 1.0f, 5.0f, "%.1f", ImGuiInputTextFlags_EscapeClearsAll)) {
                if (value > 0.0f) { limited_timeout_s_ = value; }
            }
        }
    }
}

void icmu_calibrate_driven::render_controls(icmu_calibration_core& core) {
    ImGui::Text("Starting this test will command velocity. Ensure it is safe to do so.");

    // Transition from finish here so we can stop host
    if (state_ != state::standby_s) {
        if (state_ == state::finish_s) {
            gui_if_->stop();
            state_ = state::standby_s;
        }
    }
    {
        ImGuiDisabled running_dis(state_ != state::standby_s);
        if (ImGui::Button("Start driven calibration")) {
            if (gui_if_->start() != jcs::RET_OK) { return; }
            helpers::sleep_ms(500);
            state_ = state::initialise_s;
        }
    }
    ImGui::SameLine();
    {
        ImGuiDisabled cancel_dis(state_ == state::standby_s);
        if (ImGui::Button("Cancel##driven")) {
            // Zero velocity and stop
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = 0.0f;
            host_->sig_input_set_rt(0, f32_input_signal_store_);
            helpers::sleep_ms(200);
            gui_if_->stop();
            state_ = state::standby_s;
            core.clear_buffers();
        }
    }


    // {
    //     bool test_running = (state_ != state::standby_s);
    //     if (test_running) { ImGui::BeginDisabled(); }
    //     if (ImGui::Button("Start driven calibration")) {
    //         state_ = state::initialise_s;
    //     }
    //     if (test_running) { ImGui::EndDisabled(); }
    // }

    // ImGui::SameLine();
    // {
    //     bool test_idle = (state_ == state::standby_s);
    //     if (test_idle) { ImGui::BeginDisabled(); }
    //     if (ImGui::Button("Cancel##driven")) {
    //         // Zero velocity and stop
    //         f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = 0.0f;
    //         host_->sig_input_set_rt(0, f32_input_signal_store_);
    //         state_ = state::standby_s;
    //         core.clear_buffers();
    //     }
    //     if (test_idle) { ImGui::EndDisabled(); }
    // }
}

void icmu_calibrate_driven::render_state() {
    ImGui::Text("State: ");
    ImGui::SameLine();
    switch (state_) {
        default:
        case state::standby_s:               ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Standby");          break;
        case state::initialise_s:            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Initialising");     break;
        case state::free_ramp_up_s:          ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.8f, 1.0f), "Ramping up");       break;
        case state::free_rotating_s:         ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Rotating (%d/%d)",
                                                 free_tick_, free_tick_max_);                                          break;
        case state::free_ramp_down_s:        ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.8f, 1.0f), "Ramping down");     break;
        case state::limited_ramp_s:          ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.8f, 1.0f), "Ramping");          break;
        case state::limited_moving_s:        ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Moving (pass %d/%d)",
                                                 limited_passes_done_ + 1, limited_passes_target_);                    break;
        case state::limited_endstop_detected_s: ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "Endstop!");      break;
        case state::limited_ramp_reverse_s:  ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.8f, 1.0f), "Stopping");        break;
        case state::finish_s:                ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Finishing");        break;
    }

    // Show passes in limited mode
    if (drive_mode_ == drive_mode::limited_s && state_ != state::standby_s) {
        ImGui::Text("Passes: %d / %d", limited_passes_done_, limited_passes_target_);
    }
}
