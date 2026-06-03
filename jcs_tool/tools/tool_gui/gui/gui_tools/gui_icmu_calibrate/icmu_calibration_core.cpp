// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "icmu_calibration_core.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include "imgui.h"
#include "ImGuiFileDialog.h"

#include "mu_3sl_calibration_adjustments.h"

//////////////////////////////////////////////////////////////////////
icmu_calibration_core::icmu_calibration_core()
    : samples_max_(20000),
      samples_idx_(0),
      plot_raw_("Raw encoder data", "Sample", "ADC count", samples_max_),
      plot_nonius_("Nonius calibration", "Sample", "Value (absolute resolution)", 1)
{
    samples_master_raw_.resize(samples_max_, 0);
    samples_nonius_raw_.resize(samples_max_, 0);

    // Raw data plot — x-axis is sample index
    for (int i = 0; i < samples_max_; i++) {
        plot_raw_.x_[i] = static_cast<double>(i);
    }
    plot_raw_.add_channel("Master", ImVec4(0.0f, 1.0f, 0.4f, 1.0f));
    plot_raw_.add_channel("Nonius", ImVec4(1.0f, 0.4f, 0.0f, 1.0f));
    plot_raw_master_ = plot_raw_.get_channel("Master");
    plot_raw_nonius_ = plot_raw_.get_channel("Nonius");

    // Nonius plot — phase error + margin before/after optimisation
    plot_nonius_.add_channel("Phase error",    ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
    plot_nonius_.add_channel("Margin (before)", ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
    plot_nonius_.add_channel("Margin (after)",  ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
    plot_nonius_phase_error_    = plot_nonius_.get_channel("Phase error");
    plot_nonius_margin_before_  = plot_nonius_.get_channel("Margin (before)");
    plot_nonius_margin_after_   = plot_nonius_.get_channel("Margin (after)");
}

// ---------------------------------------------------------------
// Sample buffer management
// ---------------------------------------------------------------
void icmu_calibration_core::resize_buffers(int max_samples) {
    samples_max_ = max_samples;
    samples_master_raw_.resize(samples_max_, 0);
    samples_nonius_raw_.resize(samples_max_, 0);

    // Resize raw plot
    plot_raw_.x_.resize(samples_max_);
    for (int i = 0; i < samples_max_; i++) {
        plot_raw_.x_[i] = static_cast<double>(i);
    }
    plot_raw_master_->y_.resize(samples_max_, 0.0);
    plot_raw_nonius_->y_.resize(samples_max_, 0.0);
}

void icmu_calibration_core::clear_buffers() {
    std::fill(samples_master_raw_.begin(), samples_master_raw_.end(), 0);
    std::fill(samples_nonius_raw_.begin(), samples_nonius_raw_.end(), 0);
    samples_idx_ = 0;
}

bool icmu_calibration_core::record_sample(uint16_t master_raw, uint16_t nonius_raw) {
    if (samples_idx_ >= samples_max_) {
        return false;
    }
    samples_master_raw_[samples_idx_] = master_raw;
    samples_nonius_raw_[samples_idx_] = nonius_raw;
    samples_idx_++;
    return true;
}

// ---------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------
uint32_t icmu_calibration_core::do_calibration(jcs::jcs_host* host, std::string const& target_device) {

    // Update raw data plot
    update_raw_plot();

    // Stop, then force a calibration read from the actual device.
    PARAM_NOTIFY_ERROR( host->write_command(target_device, "icmu_stop"), "Parameter failed: icmu_stop" )
    PARAM_NOTIFY_ERROR( host->write_command(target_device, "icmu_config_read"), "Parameter failed: icmu_config_read" )

    // Read MPC from device
    hw_calibration_.mpc = 0;
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_MPC", &hw_calibration_.mpc), "Parameter failed: icmu_config_MPC" )

    // Read current calib from device
    if (mu_read_analog_calib_from_device(host, target_device) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    if (mu_read_nonius_calib_from_device(host, target_device) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }

    // Start ic_mu
    PARAM_NOTIFY_ERROR( host->write_command(target_device, "icmu_start"), "Parameter failed: icmu_start" )

    uint8_t revision_code = MU_REV_MU_Y2;

    // MU_Calibration_AnalogTrackAdjustments initial_master_adjustments = {0, 0, 0, 0, 0};
    // MU_Calibration_AnalogTrackAdjustments initial_nonius_adjustments = {0, 0, 0, 0, 0};
    MU_Calibration_AnalogTrackAdjustments initial_master_adjustments = {
        hw_calibration_.GX_M,
        hw_calibration_.VOSS_M,
        hw_calibration_.VOSC_M,
        hw_calibration_.PH_M,
        0  // PHR_M - not used on iC-MU
    };
    MU_Calibration_AnalogTrackAdjustments initial_nonius_adjustments = {
        hw_calibration_.GX_N,
        hw_calibration_.VOSS_N,
        hw_calibration_.VOSC_N,
        hw_calibration_.PH_N,
        0  // PHR_N - not used on iC-MU
    };

    MU_Calibration* calibration = MU_createCalibration(revision_code);

    // Preconfigure number of master periods - we get this from the device config
    unsigned int n_master_periods = 1 << hw_calibration_.mpc;
    MU_Calibration_preconfigureNumberOfMasterPeriods(calibration, n_master_periods);

    MU_Calibration_setCurrentAnalogTrackAdjustments(
        calibration,
        &initial_master_adjustments,
        &initial_nonius_adjustments);

    // Tell the library the current nonius offset table
    MU_NoniusTrackOffsetTableParameters current_nonius_params;
    current_nonius_params.spoBase = hw_calibration_.SPO_BASE;
    for (int i = 0; i < 15; i++) {
        current_nonius_params.spoN[i] = hw_calibration_.SPO[i];
    }
    MU_Calibration_NoniusTrackOffsetTable current_nonius_table;
    MU_getNoniusTrackOffsetTableByParameters(&current_nonius_params, &current_nonius_table);
    MU_Calibration_setCurrentNoniusTrackOffsetTable(calibration, &current_nonius_table);

    // Analyze the data
    MU_CalibrationAnalyzeResult* analyzeResult = MU_Calibration_analyzeRawData(
        calibration,
        samples_master_raw_.data(),
        samples_nonius_raw_.data(),
        samples_idx_);

    if (analyzeResult == NULL) {
        MU_Calibration_delete(calibration);
        return jcs::RET_ERROR;
    }

    printf("\n\n---- iC-MU Analysis Results ----\n");
    mu_print_analyse_result_log(analyzeResult);
    mu_print_relative_info(calibration, analyzeResult);
    bool is_adjustable = validate_result(calibration, analyzeResult, true, false);
    if (!is_adjustable) {
        MU_CalibrationAnalyzeResult_delete(analyzeResult);
        MU_Calibration_delete(calibration);
        return jcs::RET_ERROR;
    }

    // Apply analog correction
    MU_Calibration_adjustAnalogByAnalyzeResult(calibration, analyzeResult);
    mu_generate_print_analog_adjustments(calibration);

    // Apply nonius correction
    if (!mu_adjust_nonius(calibration, analyzeResult)) {
        std::cout << "Warning: post-correction residuals exceed threshold\n";
    }

    // Write the calibration and restart the device to load them onto ic-mu
    std::cout << "Warning: Writing calibrations to device.\n";
    mu_write_analog_calib_to_device(host, target_device);
    mu_write_nonius_calib_to_device(host, target_device);
    PARAM_NOTIFY_ERROR( host->write_command(target_device, "icmu_stop"), "Parameter failed: icmu_stop" )
    PARAM_NOTIFY_ERROR( host->write_command(target_device, "icmu_start"), "Parameter failed: icmu_start" )

    // Clean up
    MU_CalibrationAnalyzeResult_delete(analyzeResult);
    MU_Calibration_delete(calibration);

    return jcs::RET_OK;
}

// ---------------------------------------------------------------
// UI
// ---------------------------------------------------------------
void icmu_calibration_core::render_results() {
    ImGui::Separator();
    ImGui::PushID("icmu_calibration_render_results");
    ImGui::Text("Calibration analysis YAML config output");
    helpers::result_text_copyable(calibration_analysis_result_);
    ImGui::PopID();
}

void icmu_calibration_core::update_raw_plot() {
    for (int i = 0; i < samples_idx_; i++) {
        plot_raw_master_->y_[i] = static_cast<double>(samples_master_raw_[i]);
        plot_raw_nonius_->y_[i] = static_cast<double>(samples_nonius_raw_[i]);
    }
}

void icmu_calibration_core::update_nonius_plots(
    const MU_CalibrationAnalyzeResult* before_result,
    const MU_CalibrationAnalyzeResult* after_result)
{
    // Get nonius curve sample count
    size_t n_samples = MU_Calibration_numberOfNoniusCurveSamples(before_result);
    if (n_samples == 0) { return; }

    int n = static_cast<int>(n_samples);

    // Resize plot x-axis and all channels
    plot_nonius_.x_.resize(n);
    for (int i = 0; i < n; i++) {
        plot_nonius_.x_[i] = static_cast<double>(i);
    }
    plot_nonius_phase_error_->y_.resize(n, 0.0);
    plot_nonius_margin_before_->y_.resize(n, 0.0);
    plot_nonius_margin_after_->y_.resize(n, 0.0);

    // Phase error (same for before and after — it's the raw error from the data)
    const long* phase_error = MU_Calibration_noniusPhaseError(before_result);
    if (phase_error != NULL) {
        for (int i = 0; i < n; i++) {
            plot_nonius_phase_error_->y_[i] = static_cast<double>(phase_error[i]);
        }
    }

    // Phase margin before optimisation (from original result without optimised SPO table)
    const long* margin_before = MU_Calibration_noniusPhaseMargin(before_result);
    if (margin_before != NULL) {
        for (int i = 0; i < n; i++) {
            plot_nonius_margin_before_->y_[i] = static_cast<double>(margin_before[i]);
        }
    }

    // Phase margin after optimisation (from result with optimised SPO table)
    if (after_result != NULL) {
        size_t n_after = MU_Calibration_numberOfNoniusCurveSamples(after_result);
        int n_a = static_cast<int>(n_after);
        plot_nonius_margin_after_->y_.resize(n_a, 0.0);

        const long* margin_after = MU_Calibration_noniusPhaseMargin(after_result);
        if (margin_after != NULL) {
            for (int i = 0; i < n_a; i++) {
                plot_nonius_margin_after_->y_[i] = static_cast<double>(margin_after[i]);
            }
        }
    }
}

void icmu_calibration_core::render_plots() {
    ImGui::Separator();
    plot_raw_.plot();

    ImGui::Separator();
    plot_nonius_.plot();
}

// void icmu_calibration_core::render_write_to_device(jcs::jcs_host* host, std::string const& target_device) {
//     if (ImGui::Button("Write analog calibration to device")) {
//         mu_write_analog_calib_to_device(host, target_device);
//     }
//     ImGui::SameLine();
//     if (ImGui::Button("Write nonius calibration to device")) {
//         mu_write_nonius_calib_to_device(host, target_device);
//     }
// }

int icmu_calibration_core::render_write_to_file(std::string const& target_device) {
    if (ImGui::Button("Write coefficients to file")) {
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("icmu_choose_dir_key", "Choose Directory", nullptr, config);
    }
    if (ImGuiFileDialog::Instance()->Display("icmu_choose_dir_key")) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            if (emit_config(ImGuiFileDialog::Instance()->GetCurrentPath(), target_device) != jcs::RET_OK) {
                ImGuiFileDialog::Instance()->Close();
                return jcs::RET_ERROR;
            }
        }
        ImGuiFileDialog::Instance()->Close();
    }
    return jcs::RET_OK;
}

int icmu_calibration_core::emit_config(std::string const& file_path, std::string const& target_device) {
    std::string file_name = "dev_" + target_device + "_icmu_raw.csv";
    std::string path_and_file = file_path + "/" + file_name;

    std::cout << "Writing to: " << path_and_file << "\n";

    std::ofstream config_file(path_and_file);
    config_file << "MASTER_RAW,NONIUS_RAW\n";
    for (int i = 0; i < samples_idx_; i++) {
        config_file << samples_master_raw_[i] << "," << samples_nonius_raw_[i] << "\n";
    }

    return jcs::RET_OK;
}

// ---------------------------------------------------------------
// Write to device
// ---------------------------------------------------------------
uint32_t icmu_calibration_core::mu_write_analog_calib_to_device(jcs::jcs_host* host, std::string const& target_device) {
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_GX_M",   hw_calibration_.GX_M),   "Parameter failed: icmu_config_GX_M" )
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_VOSS_M", hw_calibration_.VOSS_M), "Parameter failed: icmu_config_VOSS_M" )
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_VOSC_M", hw_calibration_.VOSC_M), "Parameter failed: icmu_config_VOSC_M" )
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_PH_M",   hw_calibration_.PH_M),   "Parameter failed: icmu_config_PH_M" )

    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_GX_N",   hw_calibration_.GX_N),   "Parameter failed: icmu_config_GX_N" )
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_VOSS_N", hw_calibration_.VOSS_N), "Parameter failed: icmu_config_VOSS_N" )
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_VOSC_N", hw_calibration_.VOSC_N), "Parameter failed: icmu_config_VOSC_N" )
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_PH_N",   hw_calibration_.PH_N),   "Parameter failed: icmu_config_PH_N" )

    return jcs::RET_OK;
}

uint32_t icmu_calibration_core::mu_read_analog_calib_from_device(jcs::jcs_host* host, std::string const& target_device) {
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_GX_M",   &hw_calibration_.GX_M),   "Parameter failed: icmu_config_GX_M" )
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_VOSS_M", &hw_calibration_.VOSS_M), "Parameter failed: icmu_config_VOSS_M" )
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_VOSC_M", &hw_calibration_.VOSC_M), "Parameter failed: icmu_config_VOSC_M" )
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_PH_M",   &hw_calibration_.PH_M),   "Parameter failed: icmu_config_PH_M" )

    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_GX_N",   &hw_calibration_.GX_N),   "Parameter failed: icmu_config_GX_N" )
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_VOSS_N", &hw_calibration_.VOSS_N), "Parameter failed: icmu_config_VOSS_N" )
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_VOSC_N", &hw_calibration_.VOSC_N), "Parameter failed: icmu_config_VOSC_N" )
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_PH_N",   &hw_calibration_.PH_N),   "Parameter failed: icmu_config_PH_N" )

    return jcs::RET_OK;
}

uint32_t icmu_calibration_core::mu_write_nonius_calib_to_device(jcs::jcs_host* host, std::string const& target_device) {
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_SPO_BASE", hw_calibration_.SPO_BASE), "Parameter failed: icmu_config_SPO_BASE" )
    PARAM_NOTIFY_ERROR( host->write_uint8(target_device, "icmu_config_SPO",      hw_calibration_.SPO),      "Parameter failed: icmu_config_SPO" )
    return jcs::RET_OK;
}

uint32_t icmu_calibration_core::mu_read_nonius_calib_from_device(jcs::jcs_host* host, std::string const& target_device) {
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_SPO_BASE", &hw_calibration_.SPO_BASE), "Parameter failed: icmu_config_SPO_BASE" )
    if (hw_calibration_.SPO.size() != 15) {
        hw_calibration_.SPO.resize(15);
    }
    PARAM_NOTIFY_ERROR( host->read_uint8(target_device, "icmu_config_SPO",      &hw_calibration_.SPO),      "Parameter failed: icmu_config_SPO" )
    return jcs::RET_OK;
}

// ---------------------------------------------------------------
// MU library helpers
// ---------------------------------------------------------------
void icmu_calibration_core::mu_print_analyse_result_log(const MU_CalibrationAnalyzeResult* result) {
    int size = MU_Calibration_getAnalyzeResultLog(result, NULL, 0, MU_CALIBRATION_ANALYZE_RESULT_LOG_ALL) + 1;
    char* result_log_message = new char[size];

    MU_Calibration_getAnalyzeResultLog(result, result_log_message, size, MU_CALIBRATION_ANALYZE_RESULT_LOG_ALL);

    std::cout << "---- iC-MU Analysis Log ----\n";
    std::cout << result_log_message << "\n";

    delete[] result_log_message;
}

void icmu_calibration_core::mu_print_relative_info(const MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result) {

    std::cout << "-- Relative adjustments --\n";
    std::cout << "Residual errors (relative changes in LSB)\n";

    MU_Calibration_RelativeAnalogTrackAdjustments relative_master_track_adjustments;
    MU_Calibration_getRelativeMasterTrackAdjustments(result, &relative_master_track_adjustments);

    MU_Calibration_RelativeAnalogTrackAdjustments relative_nonius_track_adjustments;
    MU_Calibration_getRelativeNoniusTrackAdjustments(result, &relative_nonius_track_adjustments);

    char buf[1024];
    snprintf(buf, sizeof(buf),
        "Track:             Master |   Nonius\n"
        "  Cosine gain:   %8.4f | %8.4f\n"
        "  Sine offset:   %8.4f | %8.4f\n"
        "  Cosine offset: %8.4f | %8.4f\n"
        "  Phase adjust:  %8.4f | %8.4f\n\n",
        relative_master_track_adjustments.cosineGain_lsb,
        relative_nonius_track_adjustments.cosineGain_lsb,
        relative_master_track_adjustments.sineOffset_lsb,
        relative_nonius_track_adjustments.sineOffset_lsb,
        relative_master_track_adjustments.cosineOffset_lsb,
        relative_nonius_track_adjustments.cosineOffset_lsb,
        relative_master_track_adjustments.phase_lsb,
        relative_nonius_track_adjustments.phase_lsb);

    std::cout << buf;
}

void icmu_calibration_core::mu_generate_print_analog_adjustments(const MU_Calibration* calibration) {
    printf("-- Analog parameters after adjustment --\n");

    MU_Calibration_AnalogTrackAdjustments master_track_adjustments;
    MU_Calibration_getAnalogMasterTrackAdjustments(calibration, &master_track_adjustments);
    MU_Calibration_AnalogTrackAdjustments nonius_track_adjustments;
    MU_Calibration_getAnalogNoniusTrackAdjustments(calibration, &nonius_track_adjustments);

    // Build YAML result
    calibration_analysis_result_  = "########################################################################\n";
    calibration_analysis_result_ += "# iC-MU configuration\n";
    calibration_analysis_result_ += "#\n";
    calibration_analysis_result_ += "# Analog gains\n";

    calibration_analysis_result_ += "icmu_config_GX_M   : " + std::to_string(master_track_adjustments.cosineGain) + "\n";
    calibration_analysis_result_ += "icmu_config_VOSS_M : " + std::to_string(master_track_adjustments.sineOffset) + "\n";
    calibration_analysis_result_ += "icmu_config_VOSC_M : " + std::to_string(master_track_adjustments.cosineOffset) + "\n";
    calibration_analysis_result_ += "icmu_config_PH_M   : " + std::to_string(master_track_adjustments.phase) + "\n";

    calibration_analysis_result_ += "icmu_config_GX_N   : " + std::to_string(nonius_track_adjustments.cosineGain) + "\n";
    calibration_analysis_result_ += "icmu_config_VOSS_N : " + std::to_string(nonius_track_adjustments.sineOffset) + "\n";
    calibration_analysis_result_ += "icmu_config_VOSC_N : " + std::to_string(nonius_track_adjustments.cosineOffset) + "\n";
    calibration_analysis_result_ += "icmu_config_PH_N   : " + std::to_string(nonius_track_adjustments.phase) + "\n";
    calibration_analysis_result_ += "\n";

    hw_calibration_.GX_M   = master_track_adjustments.cosineGain;
    hw_calibration_.VOSS_M = master_track_adjustments.sineOffset;
    hw_calibration_.VOSC_M = master_track_adjustments.cosineOffset;
    hw_calibration_.PH_M   = master_track_adjustments.phase;

    hw_calibration_.GX_N   = nonius_track_adjustments.cosineGain;
    hw_calibration_.VOSS_N = nonius_track_adjustments.sineOffset;
    hw_calibration_.VOSC_N = nonius_track_adjustments.cosineOffset;
    hw_calibration_.PH_N   = nonius_track_adjustments.phase;

    std::cout << "Cosine gain (GX_M):     " << std::to_string(hw_calibration_.GX_M) << "\n";
    std::cout << "Sine offset (VOSS_M):   " << std::to_string(hw_calibration_.VOSS_M) << "\n";
    std::cout << "Cosine offset (VOSC_M): " << std::to_string(hw_calibration_.VOSC_M) << "\n";
    std::cout << "Phase (PH_M):           " << std::to_string(hw_calibration_.PH_M) << "\n";
    std::cout << "Cosine gain (GX_N):     " << std::to_string(hw_calibration_.GX_N) << "\n";
    std::cout << "Sine offset (VOSS_N):   " << std::to_string(hw_calibration_.VOSS_N) << "\n";
    std::cout << "Cosine offset (VOSC_N): " << std::to_string(hw_calibration_.VOSC_N) << "\n";
    std::cout << "Phase (PH_N):           " << std::to_string(hw_calibration_.PH_N) << "\n";
    std::cout << "\n";
}

bool icmu_calibration_core::mu_adjust_nonius(MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result) {

    MU_Calibration_NoniusTrackOffsetTable optimised_nonius_offset_table;
    MU_Calibration_getOptimizedNoniusTrackOffsetTable(result, &optimised_nonius_offset_table);

    MU_Calibration_setCurrentNoniusTrackOffsetTable(calibration, &optimised_nonius_offset_table);

    // Generate a new analysis with the optimised nonius data
    MU_CalibrationAnalyzeResult* optimized_result = MU_Calibration_analyzeRawData(
        calibration,
        samples_master_raw_.data(),
        samples_nonius_raw_.data(),
        samples_idx_);

    MU_Calibration_getOptimizedNoniusTrackOffsetTable(optimized_result, &optimised_nonius_offset_table);

    MU_NoniusTrackOffsetTableParameters optimized_nonius_offset_parameters;
    MU_getNoniusTrackOffsetTableParameters(&optimised_nonius_offset_table, &optimized_nonius_offset_parameters);

    calibration_analysis_result_ += "# Optimized nonius track offset parameters\n";
    calibration_analysis_result_ += "icmu_config_SPO_BASE : " + std::to_string(optimized_nonius_offset_parameters.spoBase) + "\n";
    calibration_analysis_result_ += "icmu_config_SPO      : [";
    for (int i = 0; i < 14; i++) {
        calibration_analysis_result_ += std::to_string(optimized_nonius_offset_parameters.spoN[i]) + ", ";
    }
    calibration_analysis_result_ += std::to_string(optimized_nonius_offset_parameters.spoN[14]) + "]\n";

    std::cout << "\n";
    std::cout << "Optimized nonius track offset parameters\n";
    std::cout << "SPO_BASE: " << std::to_string(optimized_nonius_offset_parameters.spoBase) + "\n";
    for (int i = 0; i < 15; i++) {
        std::cout << "SPO_" << std::to_string(i) << ": " << std::to_string(optimized_nonius_offset_parameters.spoN[i]) + "\n";
    }
    std::cout << "\n";

    // Store for writing
    hw_calibration_.SPO_BASE = optimized_nonius_offset_parameters.spoBase;
    hw_calibration_.SPO.clear();
    for (int i = 0; i < 15; i++) {
        hw_calibration_.SPO.push_back(optimized_nonius_offset_parameters.spoN[i]);
    }

    bool pass = validate_result(calibration, optimized_result, false, true);

    // Update nonius plots (phase error + margin before/after optimisation)
    update_nonius_plots(result, optimized_result);

    MU_CalibrationAnalyzeResult_delete(optimized_result);

    return pass;
}

bool icmu_calibration_core::validate_result(const MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result, 
    bool check_adjustable, bool check_residuals)
{
    const int message_size = 1024;
    char message[message_size];
    bool pass = true;

    if (check_adjustable || check_residuals) {
        std::cout << "-- Validation --\n";
    }

    if (check_adjustable) {
        bool is_adjustable = MU_Calibration_isAnalogAnalyzeResultAdjustable(
            calibration,
            result,
            message,
            message_size,
            MU_ADJUSTMENT_MESSAGE_LOG_ALL);
        pass &= is_adjustable;
        std::cout << "Adjustable: " << (is_adjustable ? "yes" : "no") << "\n";
        if (!is_adjustable) {
            std::cout << message << "\n";
        }
    }

    if (check_residuals) {
        MU_Calibration_RelativeAnalogTrackAdjustments rel_m;
        MU_Calibration_RelativeAnalogTrackAdjustments rel_n;

        MU_Calibration_getRelativeMasterTrackAdjustments(result, &rel_m);
        MU_Calibration_getRelativeNoniusTrackAdjustments(result, &rel_n);

        const double permissable_residual_error = 1.0;

        bool residuals_ok =
            fabs(rel_m.cosineGain_lsb)   <= permissable_residual_error &&
            fabs(rel_n.cosineGain_lsb)   <= permissable_residual_error &&
            fabs(rel_m.sineOffset_lsb)   <= permissable_residual_error &&
            fabs(rel_n.sineOffset_lsb)   <= permissable_residual_error &&
            fabs(rel_m.cosineOffset_lsb) <= permissable_residual_error &&
            fabs(rel_n.cosineOffset_lsb) <= permissable_residual_error &&
            fabs(rel_m.phase_lsb)        <= permissable_residual_error &&
            fabs(rel_n.phase_lsb)        <= permissable_residual_error;
        pass &= residuals_ok;

        std::cout << "Residuals OK: " << (residuals_ok ? "yes" : "no") << "\n";
        if (residuals_ok) {
            std::cout << "\n";
            std::cout << "All residual errors are smaller than" << permissable_residual_error << "\n"; 
            std::cout << "Calibration is within acceptable limits\n\n";
        }
    }
    return pass;
}