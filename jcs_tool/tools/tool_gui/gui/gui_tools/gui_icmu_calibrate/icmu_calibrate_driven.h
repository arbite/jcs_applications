// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef ICMU_CALIBRATE_DRIVEN_H_
#define ICMU_CALIBRATE_DRIVEN_H_

#include "jcs_host.h"
#include "gui_interface.h"
#include <string>
#include <vector>
#include "helpers.h"
#include "ramp.h"
#include "icmu_calibration_core.h"

//////////////////////////////////////////////////////////////////////
/// Motor-driven iC-MU calibration tab.
/// The motor rotates the encoder target under velocity control.
///
/// Two modes:
///   Free     - unlimited rotation; spin at a set speed for a duration.
///   Limited  - the joint has endstops; bounce back and forth between
///              them using a current threshold to detect contact.
class icmu_calibrate_driven {
public:
    icmu_calibrate_driven(jcs::jcs_host* host, gui_interface* gui_if);

    int startup();

    void step_rt(icmu_calibration_core& core,
                 uint16_t master_raw, uint16_t nonius_raw);

    void render_parameters();
    void render_controls(icmu_calibration_core& core);
    void render_state();

private:
    jcs::jcs_host*  host_;
    gui_interface*  gui_if_;

    enum class state {
        standby_s,
        initialise_s,

        // Free mode states
        free_ramp_up_s,
        free_rotating_s,
        free_ramp_down_s,

        // Limited mode states
        limited_ramp_s,
        limited_moving_s,
        limited_endstop_detected_s,
        limited_ramp_reverse_s,

        finish_s,
    } state_;

    enum class drive_mode {
        free_s,
        limited_s,
    };
    drive_mode drive_mode_;

    std::vector<float> f32_input_signal_store_;
    std::vector<float> f32_output_signal_store_;

    helpers::combo_source signal_in_source_w_m_;     // Velocity command input
    helpers::combo_source signal_out_source_i_mot_;  // Motor current output
    std::vector<std::string> required_input_signal_names_;

    ramp velocity_ramp_;

    // Free mode parameters
    float free_speed_rads_;
    float free_duration_s_;
    float free_ramp_time_s_;
    int   free_tick_;
    int   free_tick_max_;

    // Limited mode parameters
    float limited_speed_rads_;
    float limited_ramp_time_s_;
    float limited_current_threshold_;   // |i_mot| above this = endstop hit
    int   limited_debounce_ticks_;      // Ticks above threshold before declaring endstop
    int   limited_debounce_count_;      // Running counter
    int   limited_passes_target_;       // How many full sweeps (endstop-to-endstop)
    int   limited_passes_done_;
    float limited_timeout_s_;           // Max time per sweep before forced stop
    int   limited_timeout_tick_;
    int   limited_timeout_tick_max_;
    float limited_current_direction_;   // +1.0 or -1.0
};

#endif
