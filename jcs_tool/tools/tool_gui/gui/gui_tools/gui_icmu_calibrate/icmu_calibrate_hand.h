// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef ICMU_CALIBRATE_HAND_H_
#define ICMU_CALIBRATE_HAND_H_

#include "jcs_host.h"
#include <string>
#include <vector>
#include "icmu_calibration_core.h"
#include "gui_interface.h"

//////////////////////////////////////////////////////////////////////
/// Hand-rotation iC-MU calibration tab.
/// The user manually rotates the encoder target while samples are recorded.
class icmu_calibrate_hand {
public:
    icmu_calibrate_hand(gui_interface* gui_if);

    void step_rt(icmu_calibration_core& core,
                 uint16_t master_raw, uint16_t nonius_raw,
                 int base_frequency);

    void render_parameters();
    void render_controls(icmu_calibration_core& core);
    void render_state();

private:
    enum class state {
        standby_s,
        initialise_s,
        wait_movement_s,
        moving_s,
        finish_s,
    } state_;

    // Configurable test parameters
    float wait_time_s_;
    int   wait_movement_tick_;
    int   wait_movement_tick_max_;
    
    gui_interface* gui_if_;
};

#endif
