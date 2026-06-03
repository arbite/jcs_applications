// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_icmu_calibrate.h"
#include <iostream>
#include "jcs_user_external.h"
#include "helpers.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include "ImGuiFileDialog.h"
#include "imgui_stdlib.h"

// iC MU library
// #include "MU_3SL_defs.h"
// #include "MU_3SL_interface.h"
#include "mu_3sl_calibration_adjustments.h"

//////////////////////////////////////////////////////////////////////
gui_icmu_calibrate::gui_icmu_calibrate(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("iC-MU Calibration", host, gui_if, target_device),
    signal_out_master_raw_("None", 0), signal_out_nonius_raw_("None", 0),
    is_ready_(false), can_start_(false)
{

}

int gui_icmu_calibrate::startup() {
    // Configure line storage
    u16_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::uint16_s, 0));

    can_start_ = true;

    required_output_signal_names_ = { target_device_+"::icmu_master_raw",
                                      target_device_+"::icmu_nonius_raw" };

    samples_master_raw_.resize(samples_max_);
    samples_nonius_raw_.resize(samples_max_);
    samples_idx_ = 0;

    wait_movement_tick_ = 0;
    state_ = state::standby_s;
    return jcs::RET_OK;
}

int gui_icmu_calibrate::step_rt() {
    host_->sig_output_get_rt(0, &u16_output_signal_store_);

    switch (state_) {
        default:
        case state::standby_s:
            break;

        case state::wait_movement_s:
            samples_idx_ = 0;
            wait_movement_tick_++;
            if (wait_movement_tick_ < 5000) {
                return jcs::RET_OK;
            }
            state_ = state::moving_s;
            break;

        case state::moving_s:
            samples_master_raw_[samples_idx_] = u16_output_signal_store_[signal_out_master_raw_.index_];
            samples_nonius_raw_[samples_idx_] = u16_output_signal_store_[signal_out_nonius_raw_.index_];
            samples_idx_++;
            if (samples_idx_ > samples_max_) {
                state_ = state::standby_s;
            }
            break;
    }
    return jcs::RET_OK;
}

int gui_icmu_calibrate::render() {

    ImGui::Text("iC-MU calibration tool");
    ImGui::Separator();
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure correct configuration is used.");
    ImGui::Text("- Ensure motor configuration has parameter estimator_0_theta_passthrough set to yes.");
    ImGui::Text("- Ensure device is not temperature clamped.");
    ImGui::Text("- Ensure motor can spin freely.");
    ImGui::Separator();

    if (ImGui::Button("Click to configure ic-mu encoder for calibration")) {
        mu_configure_for_calibration();
    }
    if (ImGui::Button("Click to configure ic-mu encoder for normal operation")) {
        // mu_configure_for_calibration();
    }    

    ImGui::Separator();

    ImGui::Text("Tool expects the following:");
    ImGui::Text("- JCS output signal iC-MU master raw: `icmu_master_raw`");
    ImGui::Text("- JCS output signal iC-MU nonius raw: `icmu_nonius_raw`");

    ImGui::Separator();
    helpers::output_signals_check(gui_if_->get_u16_output_signal_names(), &required_output_signal_names_);

    ImGui::Separator();
    ImGui::Text("Configure signals");
    helpers::combo_select("icmu_master_raw source", gui_if_->get_u16_output_signal_names(), &signal_out_master_raw_);
    helpers::combo_select("icmu_nonius_raw source", gui_if_->get_u16_output_signal_names(), &signal_out_nonius_raw_);

    if (!can_start_) {
        ImGui::Text("Can't start! Most likely missing signal.");
    }

    ImGui::Separator();
    ImGui::Text("Starting this test will start JCS system. Ensure it is safe to do so.");
    if (ImGui::Button("Start test")) {
        std::fill(samples_master_raw_.begin(), samples_master_raw_.end(), 0);
        std::fill(samples_nonius_raw_.begin(), samples_nonius_raw_.end(), 0);
        wait_movement_tick_ = 0;
        state_ = state::wait_movement_s;
    }

    ImGui::SameLine();
    if (ImGui::Button("Cancel test")) {
        switch (state_) {
        default:
        case state::standby_s: 
            break;
        case state::wait_movement_s:
        case state::moving_s:
            state_ = state::standby_s;
            break;
        }
    }    

    ImGui::Separator();

    // Some nice stats
    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                         ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;
    if (ImGui::BeginTable("Measurements", 2, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("icmu_master_raw");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%u", u16_output_signal_store_[signal_out_master_raw_.index_]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("icmu_nonius_raw");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%u", u16_output_signal_store_[signal_out_nonius_raw_.index_]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("Wait movement tick");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%u", wait_movement_tick_);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("Sample index");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%u", samples_idx_);

        ImGui::EndTable();
    }
    {
        float progress = (float)samples_idx_ / (float)samples_max_; 
        char buf[32];
        sprintf(buf, "%d/%d", samples_idx_, samples_max_);
        ImGui::ProgressBar(progress, ImVec2(0.0f, 0.0f), buf);
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::Text("Test progress");
    }
    // {
    //     float error = (1.0f - (theta_error_ / (2.0f*(float)M_PI)));
    //     ImGui::ProgressBar(error, ImVec2(0.0f, 0.0f));
    //     ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    //     ImGui::Text("Position error");
    // }

    ImGui::Separator();

    bool can_calib = state_ == state::standby_s;
    if (!can_calib) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("Do calibration")) {
        mu_do_calibration();
    }
    if (!can_calib) {
        ImGui::EndDisabled();
    }
    mu_print_calibration_analysis();

    write_coeffs_to_file();

    if (ImGui::Button("Write analog calibration to device")) {
        mu_write_analog_calib_to_device();
    }
    ImGui::SameLine();
    if (ImGui::Button("Write nonius calibration to device")) {
        mu_write_nonius_calib_to_device();
    }


    // // Cleanup, but don't clear is_ready here
    // if (!is_ready_) {
    //     ImGui::EndDisabled();
    // }

    return jcs::RET_OK;
}

int gui_icmu_calibrate::write_coeffs_to_file() {
    // Choose file to write to
    if (ImGui::Button("Write coefficients to file")) {
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("choose_dir_key", "Choose Directory", nullptr, config);
    }
    // display
    if (ImGuiFileDialog::Instance()->Display("choose_dir_key"))  {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            if (emit_config(ImGuiFileDialog::Instance()->GetCurrentPath()) != jcs::RET_OK) {
                return jcs::RET_ERROR;
            }
        }
        ImGuiFileDialog::Instance()->Close();
    }
    return jcs::RET_OK;
}

int gui_icmu_calibrate::emit_config(std::string const& file_path) {
    // Write to file
    std::string file_name = "dev_" + target_device_ + "_icmu_raw.csv";
    std::string path_and_file = file_path + "/" + file_name;

    std::cout << "Writing to: " << path_and_file << "\n";

    std::ofstream config_file(path_and_file); 

    // MASTER_RAW,NONIUS_RAW
    // 312,14491
    config_file << "MASTER_RAW,NONIUS_RAW\n";
    for (int i=0; i<samples_master_raw_.size(); i++) {
        config_file << samples_master_raw_[i] << "," << samples_nonius_raw_[i] << "\n";
    }


    return jcs::RET_OK;
}


uint32_t gui_icmu_calibrate::mu_configure_for_calibration() {
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_, "icmu_mode", "icmu_mode_calibrate"), "Parameter failed: icmu_mode" )

    return jcs::RET_OK;
}

uint32_t gui_icmu_calibrate::mu_do_calibration() {
    
    // Stop jcs if not already

    // Read MPC from device
    hw_calibration_.mpc = 0;
    PARAM_NOTIFY_ERROR( host_->read_uint8(target_device_, "icmu_config_MPC", &hw_calibration_.mpc), "Parameter failed: icmu_config_MPC" )

    // Start ic_mu
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "icmu_start"), "Parameter failed: icmu_start" )


    uint8_t revision_code = MU_REV_MU_Y2;

    MU_Calibration_AnalogTrackAdjustments initial_master_adjustments = {0, 0, 0, 0, 0};
    MU_Calibration_AnalogTrackAdjustments initial_nonius_adjustments = {0, 0, 0, 0, 0};

    MU_Calibration* calibration = NULL;
    
    calibration = MU_createCalibration(revision_code);

    // Preconfigure number of master periods - we get this from the devices config
    unsigned int n_master_periods = 1 << hw_calibration_.mpc;
    MU_Calibration_preconfigureNumberOfMasterPeriods(
        calibration,
        n_master_periods);

    MU_Calibration_setCurrentAnalogTrackAdjustments(
        calibration,
        &initial_master_adjustments,
        &initial_nonius_adjustments);

    // Analyze the data
    MU_CalibrationAnalyzeResult* analyzeResult = NULL;

    analyzeResult = MU_Calibration_analyzeRawData(
        calibration,
        samples_master_raw_.data(),
        samples_nonius_raw_.data(),
        samples_master_raw_.size());

    if (analyzeResult == NULL) {
        MU_Calibration_delete(calibration);
        return jcs::RET_ERROR;
    }

    // // Generate results text
    // int analysis_result_size = MU_Calibration_getAnalyzeResultLog(analyze_result, NULL, 0, MU_CALIBRATION_ANALYZE_RESULT_LOG_ALL) + 1;
    // if (analysis_result_size > result_size_) {
    //     std::cout << "iC mu analysis results bigger than static buffer size.\n";
    //     return jcs::RET_ERROR;
    // }

    // int result_offset = 0;
    // int remaining_result_size = result_size_ - strlen(calibration_analysis_result_);

    // // MU_Calibration_getAnalyzeResultLog(
    // //         analyze_result,
    // //         calibration_analysis_result_ + result_offset,
    // //         remaining_result_size,
    // //         MU_CALIBRATION_ANALYZE_RESULT_LOG_ALL);   

    // mu_print_analyse_result_log(analyze_result, calibration_analysis_result_ + result_offset, remaining_result_size);

    printf("\n\n---- iC-MU Analysis Results ----\n");
    // printAnalyzeResultLog(analyzeResult);
    mu_print_analyse_result_log(analyzeResult);
    bool is_adjustable = mu_print_relative_info(calibration, analyzeResult);

    if (is_adjustable) {
        MU_Calibration_adjustAnalogByAnalyzeResult(calibration, analyzeResult);
        mu_generate_print_analog_adjustments(calibration);
        mu_adjust_nonius(calibration, analyzeResult);
    }

    return jcs::RET_OK;
}

void gui_icmu_calibrate::mu_print_calibration_analysis(void) {

    ImGui::Text("Calibration analysis YAML config output");

    // ImGui::InputTextMultiline("##source", calibration_analysis_result_, IM_ARRAYSIZE(calibration_analysis_result_), ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 16), ImGuiInputTextFlags_ReadOnly);
    ImGui::InputTextMultiline("##source", &calibration_analysis_result_, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 22), ImGuiInputTextFlags_ReadOnly);

    // IMGUI_API bool  InputTextMultiline(const char* label, std::string* str, const ImVec2& size = ImVec2(0, 0), ImGuiInputTextFlags flags = 0, ImGuiInputTextCallback callback = nullptr, void* user_data = nullptr);

}
uint32_t gui_icmu_calibrate::mu_write_analog_calib_to_device(void) {
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_GX_M",   hw_calibration_.GX_M),   "Parameter failed: icmu_config_GX_M" )
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_VOSS_M", hw_calibration_.VOSS_M), "Parameter failed: icmu_config_VOSS_M" )
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_VOSC_M", hw_calibration_.VOSC_M), "Parameter failed: icmu_config_VOSC_M" )
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_PH_M",   hw_calibration_.PH_M),   "Parameter failed: icmu_config_PH_M" )

    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_GX_N",   hw_calibration_.GX_N),   "Parameter failed: icmu_config_GX_N" )
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_VOSS_N", hw_calibration_.VOSS_N), "Parameter failed: icmu_config_VOSS_N" )
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_VOSC_N", hw_calibration_.VOSC_N), "Parameter failed: icmu_config_VOSC_N" )
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_PH_N",   hw_calibration_.PH_N),   "Parameter failed: icmu_config_PH_N" )

    return jcs::RET_OK;
}
uint32_t gui_icmu_calibrate::mu_write_nonius_calib_to_device(void) {
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_SPO_BASE", hw_calibration_.SPO_BASE), "Parameter failed: icmu_config_SPO_BASE" )
    PARAM_NOTIFY_ERROR( host_->write_uint8(target_device_, "icmu_config_SPO",      hw_calibration_.SPO),      "Parameter failed: icmu_config_SPO" )
    return jcs::RET_OK;
}


void gui_icmu_calibrate::mu_print_analyse_result_log(const MU_CalibrationAnalyzeResult* result) {
    int size = MU_Calibration_getAnalyzeResultLog(result, NULL, 0, MU_CALIBRATION_ANALYZE_RESULT_LOG_ALL) + 1;
    char* result_log_message = new char[size];
    
    MU_Calibration_getAnalyzeResultLog(result, result_log_message, size, MU_CALIBRATION_ANALYZE_RESULT_LOG_ALL);

    std::cout << "---- iC-MU Analysis Log ----\n";
    std::cout << result_log_message << "\n";

    delete[] result_log_message;
}
bool gui_icmu_calibrate::mu_print_relative_info(const MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result) {

    std::cout << "-- Relative adjustments --\n";
    std::cout << "Residual errors (relative changes in LSB)\n";

    MU_Calibration_RelativeAnalogTrackAdjustments relative_master_track_adjustments;
    MU_Calibration_getRelativeMasterTrackAdjustments(result, &relative_master_track_adjustments);

    MU_Calibration_RelativeAnalogTrackAdjustments relative_nonius_track_adjustments;
    MU_Calibration_getRelativeNoniusTrackAdjustments(result, &relative_nonius_track_adjustments);

    char cout_formatting_sux_buffer[1024];
    snprintf(cout_formatting_sux_buffer, sizeof(cout_formatting_sux_buffer), 
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

    std::cout << cout_formatting_sux_buffer;

    // Print adjustables log
    const int message_size = 1024;
    char* message = new char[message_size];

    bool is_adjustable = MU_Calibration_isAnalogAnalyzeResultAdjustable(
        calibration,
        result,
        message,
        message_size,
        MU_ADJUSTMENT_MESSAGE_LOG_ALL);

    if (is_adjustable) {
        std::cout << "Result is adjustable!\n";
    } else {
        std::cout << "Result is NOT adjustable!\n";
    }

    // Check if we should any more adjustments
    const double permissable_residual_error = 1.0;

    if ( (fabs(relative_master_track_adjustments.cosineGain_lsb) <= permissable_residual_error) &&
         (fabs(relative_nonius_track_adjustments.cosineGain_lsb) <= permissable_residual_error) &&
         (fabs(relative_master_track_adjustments.sineOffset_lsb) <= permissable_residual_error) &&
         (fabs(relative_nonius_track_adjustments.sineOffset_lsb) <= permissable_residual_error) &&
         (fabs(relative_master_track_adjustments.cosineOffset_lsb) <= permissable_residual_error) &&
         (fabs(relative_nonius_track_adjustments.cosineOffset_lsb) <= permissable_residual_error) &&
         (fabs(relative_master_track_adjustments.phase_lsb) <= permissable_residual_error) &&
         (fabs(relative_nonius_track_adjustments.phase_lsb) <= permissable_residual_error) ) {
        printf("\n"
            "All residual errors are smaller than %.3f.\n"
            "No further analog calibration is required\n\n",
            permissable_residual_error);
    }

    return is_adjustable;
}

void gui_icmu_calibrate::mu_generate_print_analog_adjustments(const MU_Calibration* calibration) {
    printf("-- Analog parameters after adjustment --\n");
    // printAnalogAdjustments(calibration);

    // Get the master track calibration values
    MU_Calibration_AnalogTrackAdjustments master_track_adjustments;
    MU_Calibration_getAnalogMasterTrackAdjustments(calibration, &master_track_adjustments);
    // Get the nonius track calibration values
    MU_Calibration_AnalogTrackAdjustments nonius_track_adjustments;
    MU_Calibration_getAnalogNoniusTrackAdjustments(calibration, &nonius_track_adjustments);

    // Set calibration results
    calibration_analysis_result_  = "  ########################################################################\n";
    calibration_analysis_result_ += "  # iC-MU configuration\n";
    calibration_analysis_result_ += "  #\n";
    calibration_analysis_result_ += "  # Analog gains\n";

    calibration_analysis_result_ += "  icmu_config_GX_M   : " + std::to_string(master_track_adjustments.cosineGain) + "\n";
    calibration_analysis_result_ += "  icmu_config_VOSS_M : " + std::to_string(master_track_adjustments.sineOffset) + "\n";
    calibration_analysis_result_ += "  icmu_config_VOSC_M : " + std::to_string(master_track_adjustments.cosineOffset) + "\n";
    calibration_analysis_result_ += "  icmu_config_PH_M   : " + std::to_string(master_track_adjustments.phase) + "\n";

    calibration_analysis_result_ += "  icmu_config_GX_N   : " + std::to_string(nonius_track_adjustments.cosineGain) + "\n";
    calibration_analysis_result_ += "  icmu_config_VOSS_N : " + std::to_string(nonius_track_adjustments.sineOffset) + "\n";
    calibration_analysis_result_ += "  icmu_config_VOSC_N : " + std::to_string(nonius_track_adjustments.cosineOffset) + "\n";
    calibration_analysis_result_ += "  icmu_config_PH_N   : " + std::to_string(nonius_track_adjustments.phase) + "\n";
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

void gui_icmu_calibrate::mu_adjust_nonius(MU_Calibration* calibration, const MU_CalibrationAnalyzeResult* result) {

    MU_Calibration_NoniusTrackOffsetTable optimised_nonius_offset_table;
    MU_Calibration_getOptimizedNoniusTrackOffsetTable(result, &optimised_nonius_offset_table);

    MU_Calibration_setCurrentNoniusTrackOffsetTable(calibration, &optimised_nonius_offset_table);

    // Generate a new analysis with the optimised nonius data
    MU_CalibrationAnalyzeResult* optimized_result = NULL;
    optimized_result = MU_Calibration_analyzeRawData(
        calibration,
        samples_master_raw_.data(),
        samples_nonius_raw_.data(),
        samples_master_raw_.size());

    // Pritn optimized nonius track offset table

    // Print nonius offset parameters
    MU_Calibration_getOptimizedNoniusTrackOffsetTable(optimized_result, &optimised_nonius_offset_table);

    MU_NoniusTrackOffsetTableParameters optimized_nonius_offset_parameters;
    MU_getNoniusTrackOffsetTableParameters(&optimised_nonius_offset_table, &optimized_nonius_offset_parameters);

    calibration_analysis_result_ += "  # Optimized nonius track offset parameters\n";
    calibration_analysis_result_ += "  icmu_config_SPO_BASE : " + std::to_string(optimized_nonius_offset_parameters.spoBase) + "\n";
    calibration_analysis_result_ += "  icmu_config_SPO      : [";
    for (int i=0; i<14; i++) {
        calibration_analysis_result_ += std::to_string(optimized_nonius_offset_parameters.spoN[i]) + ", ";
    }
    calibration_analysis_result_ +=  std::to_string(optimized_nonius_offset_parameters.spoN[14]) + "]\n";

    std::cout << "\n";
    std::cout << "Optimized nonius track offset parameters\n";
    std::cout << "SPO_BASE: " << std::to_string(optimized_nonius_offset_parameters.spoBase) + "\n";
    for (int i=0; i<15; i++) {
        std::cout << "SPO_" << std::to_string(i) << ": " << std::to_string(optimized_nonius_offset_parameters.spoN[i]) + "\n";
    }
    std::cout << "\n";
    // Store for writing
    hw_calibration_.SPO_BASE = optimized_nonius_offset_parameters.spoBase;
    hw_calibration_.SPO.clear();
    for (int i=0; i<15; i++) {
        hw_calibration_.SPO.push_back(optimized_nonius_offset_parameters.spoN[i]);
    }
}



  // # icmu_config_SPO_BASE: 11
  // # icmu_config_SPO: [1, 0, 1, 0, 0, 2, 1, 1, 15, 15, 15, 14, 15, 14, 1]