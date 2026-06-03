// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_ICMU_CALIBRATE_H_
#define GUI_ICMU_CALIBRATE_H_

#include "jcs_host.h"
#include "imgui.h"
#include <array>
#include <string>
#include "helpers.h"
#include "gui_type_base.h"
#include "gui_interface.h"

#include "MU_3SL_defs.h"
#include "MU_3SL_interface.h"

//////////////////////////////////////////////////////////////////////
class gui_icmu_calibrate : public gui_type_base {
public:
    gui_icmu_calibrate(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_icmu_calibrate() {}

    int startup();
    int step_rt();
    int render();

private:

    enum class state {
        standby_s,
        wait_movement_s,
        moving_s,
    };
    state state_;

    std::vector<std::string> required_output_signal_names_;

    bool is_ready_;
    bool can_start_;
    int ready_test();

    // Storage
    std::vector<uint16_t> u16_output_signal_store_;
    // Signal sources (selectable via combo)
    helpers::combo_source signal_out_master_raw_;
    helpers::combo_source signal_out_nonius_raw_;
    //
    std::vector<uint16_t> samples_master_raw_;
    std::vector<uint16_t> samples_nonius_raw_;
    const static int samples_max_ = 20000;
    int samples_idx_;

    // helpers
    int wait_movement_tick_;

    int write_coeffs_to_file();
    int emit_config(std::string const& file_path);

    // icmu calibration
    struct hw_calibration {
        uint8_t mpc;
        uint8_t GX_M;
        uint8_t VOSS_M;
        uint8_t VOSC_M;
        uint8_t PH_M;
        uint8_t GX_N;
        uint8_t VOSS_N;
        uint8_t VOSC_N;
        uint8_t PH_N;

        uint8_t SPO_BASE;
        std::vector<uint8_t> SPO;
    };
    hw_calibration hw_calibration_;

    uint32_t mu_configure_for_calibration();
    uint32_t mu_do_calibration();
    static int const result_size_ = 1024 * 256;
    // char calibration_analysis_result_[result_size_];

    std::string calibration_analysis_result_;

    void mu_print_calibration_analysis(void);
    uint32_t mu_write_analog_calib_to_device(void);
    uint32_t mu_write_nonius_calib_to_device(void);

    // Misc helpers
    void mu_print_analyse_result_log(const MU_CalibrationAnalyzeResult* result);
    bool mu_print_relative_info(const MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result);
    void mu_generate_print_analog_adjustments(const MU_Calibration* calibration);
    void mu_adjust_nonius(MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result);
};

#endif