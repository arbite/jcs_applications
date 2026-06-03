// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef ICMU_CALIBRATION_CORE_H_
#define ICMU_CALIBRATION_CORE_H_

#include "jcs_host.h"
#include <string>
#include <vector>
#include <cstdint>
#include "helpers.h"

#include "MU_3SL_defs.h"
#include "MU_3SL_interface.h"
#include "plot_measurement_multi.h"

//////////////////////////////////////////////////////////////////////
/// Shared iC-MU calibration logic.
/// Owns sample buffers, runs the MU library analysis, renders results,
/// and writes calibration values to the device.
/// Both hand and driven calibration tabs create an instance of this.
//////////////////////////////////////////////////////////////////////
class icmu_calibration_core {
public:
    icmu_calibration_core();

    // ---------------------------------------------------------------
    // Sample buffer management
    // ---------------------------------------------------------------
    void resize_buffers(int max_samples);
    void clear_buffers();
    int  samples_max()  const { return samples_max_; }
    int  samples_idx()  const { return samples_idx_; }

    /// Record one sample pair.  Returns false if buffer is full.
    bool record_sample(uint16_t master_raw, uint16_t nonius_raw);

    // ---------------------------------------------------------------
    // Calibration
    // ---------------------------------------------------------------
    /// Run the full iC-MU calibration analysis on the recorded samples.
    uint32_t do_calibration(jcs::jcs_host* host, std::string const& target_device);

    // ---------------------------------------------------------------
    // UI
    // ---------------------------------------------------------------
    /// Render the copyable YAML result text.
    void render_results();

    /// Render raw data and nonius phase error plots.
    void render_plots();

    /// Render the "Write to device" and "Write to file" buttons.
    // void render_write_to_device(jcs::jcs_host* host, std::string const& target_device);
    int  render_write_to_file(std::string const& target_device);

private:
    // ---------------------------------------------------------------
    // Sample buffers
    // ---------------------------------------------------------------
    std::vector<uint16_t> samples_master_raw_;
    std::vector<uint16_t> samples_nonius_raw_;
    int samples_max_;
    int samples_idx_;

    // ---------------------------------------------------------------
    // Hardware calibration values
    // ---------------------------------------------------------------
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

    std::string calibration_analysis_result_;

    // ---------------------------------------------------------------
    // Plots
    // ---------------------------------------------------------------
    // Raw ADC data
    plot_measurement_multi plot_raw_;
    plot_measurement_multi::channel* plot_raw_master_;
    plot_measurement_multi::channel* plot_raw_nonius_;

    // Nonius phase margin (from MU library analysis)
    plot_measurement_multi plot_nonius_;
    plot_measurement_multi::channel* plot_nonius_phase_error_;
    plot_measurement_multi::channel* plot_nonius_margin_before_;
    plot_measurement_multi::channel* plot_nonius_margin_after_;
 
    void update_raw_plot();
    void update_nonius_plots(const MU_CalibrationAnalyzeResult* before_result,
                             const MU_CalibrationAnalyzeResult* after_result);


    // ---------------------------------------------------------------
    // MU library helpers
    // ---------------------------------------------------------------
    void mu_print_analyse_result_log(const MU_CalibrationAnalyzeResult* result);
    void mu_print_relative_info(const MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result);
    void mu_generate_print_analog_adjustments(const MU_Calibration* calibration);
    bool mu_adjust_nonius(MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result);
    bool validate_result(const MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result, 
                         bool check_adjustable, bool check_residuals);

    // ---------------------------------------------------------------
    // Write helpers
    // ---------------------------------------------------------------
    uint32_t mu_write_analog_calib_to_device(jcs::jcs_host* host, std::string const& target_device);
    uint32_t mu_read_analog_calib_from_device(jcs::jcs_host* host, std::string const& target_device);
    uint32_t mu_write_nonius_calib_to_device(jcs::jcs_host* host, std::string const& target_device);
    uint32_t mu_read_nonius_calib_from_device(jcs::jcs_host* host, std::string const& target_device);
    int emit_config(std::string const& file_path, std::string const& target_device);
};

#endif