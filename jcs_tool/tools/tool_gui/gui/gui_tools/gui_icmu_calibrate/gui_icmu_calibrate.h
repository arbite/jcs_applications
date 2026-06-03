// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_ICMU_CALIBRATE_H_
#define GUI_ICMU_CALIBRATE_H_

#include "jcs_host.h"
#include "imgui.h"
#include <string>
#include <vector>
#include "helpers.h"
#include "gui_type_base.h"
#include "gui_interface.h"

#include "icmu_calibration_core.h"
#include "icmu_calibrate_hand.h"
#include "icmu_calibrate_driven.h"

//////////////////////////////////////////////////////////////////////
class gui_icmu_calibrate : public gui_type_base {
public:
    gui_icmu_calibrate(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_icmu_calibrate() {}

    int startup();
    int step_rt();
    int render();

private:
    bool is_ready_;
    bool can_start_;
    int ready_test();

    std::vector<std::string> required_output_signal_names_;

    std::vector<uint16_t> u16_output_signal_store_;
    helpers::combo_source signal_out_master_raw_;
    helpers::combo_source signal_out_nonius_raw_;
    int samples_max_;

    icmu_calibration_core core_;

    icmu_calibrate_hand   tab_hand_;
    icmu_calibrate_driven tab_driven_;
    // Track which tab is active for step_rt routing
    int active_tab_;
};

#endif
