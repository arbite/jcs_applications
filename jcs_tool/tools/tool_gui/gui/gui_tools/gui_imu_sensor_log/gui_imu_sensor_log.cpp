// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_imu_sensor_log.h"
#include <iostream>
#include "jcs_user_external.h"
#include "helpers.h"
#include <cmath>
#include <numeric>
#include "ImGuiFileDialog.h"

//////////////////////////////////////////////////////////////////////
gui_imu_sensor_log::gui_imu_sensor_log(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("IMU sensor log", host, gui_if, target_device),
    state_(state::standby_s)
{

}

int gui_imu_sensor_log::startup() {

    // Signal index helpers
    can_start_ = true;

    if (can_start_) {
        is_ready_ = true;
    }

    return jcs::RET_OK;
}

int gui_imu_sensor_log::step_rt() {
    return jcs::RET_OK;
}

int gui_imu_sensor_log::render() {

    ImGui::Text("IMU sensor log tool");
    ImGui::Separator();
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure correct configuration is used.");

    ImGui::Separator();


    if (ImGui::Button("Sampler start")) {
        imu_sample_start();
    }
    ImGui::SameLine();
    if (ImGui::Button("Sampler stop")) {
        state_ = state::standby_s;
    }
    ImGui::Separator();

    imu_sample_step();
    imu_sample_step();
    imu_sample_step();

    // // Cleanup, but don't clear is_ready here
    // if (!is_ready_) {
    //     ImGui::EndDisabled();
    // }

    return jcs::RET_OK;
}

void gui_imu_sensor_log::imu_sample_start() {
    imu_param_idx_ = 0;
    accel_line_.resize(32 * 3, 0);
    gyro_line_.resize(32 * 3, 0);
    state_ = state::sampling_s;
}

int gui_imu_sensor_log::imu_sample_step() {
    std::vector<uint16_t> accel_raw(3);
    std::vector<uint16_t> gyro_raw(3);

    switch (state_) {
        default:
        case state::standby_s:
            break;

        case state::sampling_s:
            {
                std::string accel_cmd = "device_" + std::to_string(imu_param_idx_) + "_acc_raw";
                std::string gyro_cmd = "device_" + std::to_string(imu_param_idx_) + "_gyro_raw";

                host_->read_uint16(target_device_, accel_cmd, &accel_raw);
                host_->read_uint16(target_device_, gyro_cmd, &gyro_raw);

                int offset = imu_param_idx_ * 3;
                accel_line_[offset + 0] = accel_raw[0];
                accel_line_[offset + 1] = accel_raw[1];
                accel_line_[offset + 2] = accel_raw[2];
                gyro_line_[offset + 0] = gyro_raw[0];
                gyro_line_[offset + 1] = gyro_raw[1];
                gyro_line_[offset + 2] = gyro_raw[2];

                imu_param_idx_++;
                if (imu_param_idx_ >= 32) {
                    // Line complete - print it
                    std::cout << "acc: ";
                    for (int i = 0; i < 32 * 3; i++) {
                        std::cout << accel_line_[i];
                        if (i < 32 * 3 - 1) std::cout << ",";
                    }
                    std::cout << "\n";

                    std::cout << "gyr: ";
                    for (int i = 0; i < 32 * 3; i++) {
                        std::cout << gyro_line_[i];
                        if (i < 32 * 3 - 1) std::cout << ",";
                    }
                    std::cout << "\n";

                    imu_param_idx_ = 0;
                }
            }
            break;
    }
    return jcs::RET_OK;
}








// // Copyright (c) 2024 Arbite Robotics Pty Ltd
// // https://arbite.io
// //
// #include "gui_imu_sensor_log.h"
// #include <iostream>
// #include "jcs_user_external.h"
// #include "helpers.h"
// #include <cmath>
// #include <numeric>
// #include "ImGuiFileDialog.h"

// //////////////////////////////////////////////////////////////////////
// gui_imu_sensor_log::gui_imu_sensor_log(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
//     gui_type_base("IMU sensor log", host, gui_if, target_device),
//     sampler_(host->base_frequency_get(), &sensor_names_, (32+32+4)*3, host->base_frequency_get(), 10)
// {
//     // sampler_(host->base_frequency_get()/16, gui_if->get_f32_output_signal_names(), (32+4)*3, 10, 10)

//     sout_a_a_x_idx_ = 0;
//     sout_a_a_y_idx_ = 0;
//     sout_a_a_z_idx_ = 0;
//     sout_a_b_x_idx_ = 0;
//     sout_a_b_y_idx_ = 0;
//     sout_a_b_z_idx_ = 0;

//     sout_w_a_x_idx_ = 0;
//     sout_w_a_y_idx_ = 0;
//     sout_w_a_z_idx_ = 0;
//     sout_w_b_x_idx_ = 0;
//     sout_w_b_y_idx_ = 0;
//     sout_w_b_z_idx_ = 0;

//     sout_m_x_idx_ = 0;
//     sout_m_y_idx_ = 0;
//     sout_m_z_idx_ = 0;

//     storage_acc_.resize(32*3);
//     storage_gyro_.resize(32*3);
//     storage_mag_.resize(4*3);
//     storage_line_.resize((32 + 32 + 4) * 3);
//     storage_idx_.resize(2);
//     sout_active_imu_idx_ = 0;
//     sout_active_mag_idx_ = 1;

//     index_imu_ = 0;
//     index_mag_ = 0;

//     // required_input_signal_names_ = { "HOST::th_m" };
//     // required_output_signal_names_ = { target_device_+"::th_m_0",
//     //                                   target_device_+"::w_m_0",
//     //                                   target_device_+"::i_q" };
//     is_ready_ = false;
//     can_start_ = true;

//     // Generate a huge number of sensor names
//     for (int i=0; i<32; i++) {
//         sensor_names_.push_back("a_" + std::to_string(i) + "_x");
//         sensor_names_.push_back("a_" + std::to_string(i) + "_y");
//         sensor_names_.push_back("a_" + std::to_string(i) + "_z");
//     }
//     for (int i=0; i<32; i++) {
//         sensor_names_.push_back("g_" + std::to_string(i) + "_x");
//         sensor_names_.push_back("g_" + std::to_string(i) + "_y");
//         sensor_names_.push_back("g_" + std::to_string(i) + "_z");
//     }
//     for (int i=0; i<4; i++) {
//         sensor_names_.push_back("m_" + std::to_string(i) + "_x");
//         sensor_names_.push_back("m_" + std::to_string(i) + "_y");
//         sensor_names_.push_back("m_" + std::to_string(i) + "_z");
//     }
// }

// int gui_imu_sensor_log::startup() {
//     // Confiugure line storage
//     f32_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
//     // f32_input_signal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

//     // Signal index helpers
//     can_start_ = true;
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::a_x",  &sout_a_a_x_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::a_y",  &sout_a_a_y_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::a_z",  &sout_a_a_z_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::al_x", &sout_a_b_x_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::al_y", &sout_a_b_y_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::al_z", &sout_a_b_z_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::w_x",  &sout_w_a_x_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::w_y",  &sout_w_a_y_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::w_z",  &sout_w_a_z_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::q_0",  &sout_w_b_x_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::q_1",  &sout_w_b_y_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::q_2",  &sout_w_b_z_idx_) != jcs::RET_OK) { can_start_ = false; }

//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::m_x", &sout_m_x_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::m_y", &sout_m_y_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::m_z", &sout_m_z_idx_) != jcs::RET_OK) { can_start_ = false; }

//     // if (helpers::signals_names_contains(gui_if_->get_u16_output_signal_names(), target_device_+"::imu_debug_index", &sout_active_imu_idx_) != jcs::RET_OK) { can_start_ = false; }
//     // if (helpers::signals_names_contains(gui_if_->get_u16_output_signal_names(), target_device_+"::mag_debug_index", &sout_active_mag_idx_) != jcs::RET_OK) { can_start_ = false; }

//     storage_idx_[0] = 0;
//     storage_idx_[1] = 0;
//     mag_do_sample_ = true;

//     for (int i=0; i<storage_line_.size(); i++) {
//         storage_line_[0] = 0.0f;
//     }

//     if (sampler_.startup((double)jcs::external::time_now_ns()) != jcs::RET_OK) {
//         return jcs::RET_ERROR;
//     }

//     if (can_start_) {
//         is_ready_ = true;
//     }

//     return jcs::RET_OK;
// }

// int gui_imu_sensor_log::step_rt() {

//     host_->sig_output_get_rt(0, &f32_output_signal_store_);
//     host_->sig_output_get_rt(0, &storage_idx_);

//     index_imu_ = storage_idx_[sout_active_imu_idx_];
//     index_imu_ /= 2;

//     // Accelerometers and gyros
//     storage_acc_[index_imu_*6 + 0]  = f32_output_signal_store_[sout_a_a_x_idx_];
//     storage_acc_[index_imu_*6 + 1]  = f32_output_signal_store_[sout_a_a_y_idx_];
//     storage_acc_[index_imu_*6 + 2]  = f32_output_signal_store_[sout_a_a_z_idx_];
//     storage_acc_[index_imu_*6 + 3]  = f32_output_signal_store_[sout_a_b_x_idx_];
//     storage_acc_[index_imu_*6 + 4]  = f32_output_signal_store_[sout_a_b_y_idx_];
//     storage_acc_[index_imu_*6 + 5]  = f32_output_signal_store_[sout_a_b_z_idx_];

//     storage_gyro_[index_imu_*6 + 0] = f32_output_signal_store_[sout_w_a_x_idx_];
//     storage_gyro_[index_imu_*6 + 1] = f32_output_signal_store_[sout_w_a_y_idx_];
//     storage_gyro_[index_imu_*6 + 2] = f32_output_signal_store_[sout_w_a_z_idx_];
//     storage_gyro_[index_imu_*6 + 3] = f32_output_signal_store_[sout_w_b_x_idx_];
//     storage_gyro_[index_imu_*6 + 4] = f32_output_signal_store_[sout_w_b_y_idx_];
//     storage_gyro_[index_imu_*6 + 5] = f32_output_signal_store_[sout_w_b_z_idx_];

//     if (mag_do_sample_) {
//         index_mag_ = storage_idx_[sout_active_mag_idx_];
//         storage_mag_[index_mag_*3 + 0]  = f32_output_signal_store_[sout_m_x_idx_];
//         storage_mag_[index_mag_*3 + 1]  = f32_output_signal_store_[sout_m_y_idx_];
//         storage_mag_[index_mag_*3 + 2]  = f32_output_signal_store_[sout_m_z_idx_];
//         if (index_mag_ >= 3) {
//             mag_do_sample_ = false;
//         }
//     }

//     // Sample if last one
//     if (index_imu_ >= 14) {
//         mag_do_sample_ = true;
//         // This is some bull shit
//         for (int i=0; i<storage_acc_.size(); i++ ) {
//             storage_line_[i] = storage_acc_[i];
//         }
//         int start = storage_acc_.size();
//         for (int i=0; i<storage_gyro_.size(); i++ ) {
//             storage_line_[start + i] = storage_gyro_[i];
//         }

//         start += storage_gyro_.size();
//         for (int i=0; i<storage_mag_.size(); i++ ) {
//             storage_line_[start + i] = storage_mag_[i];
//         }
//         sampler_.step_rt((double)jcs::external::time_now_ns(), &storage_line_);

//         // reset storage so can catch data loss
//         for (int i=0; i<storage_line_.size(); i++) {
//             storage_line_[i] = -1000000.0f;
//         }

//     }
//     return jcs::RET_OK;
// }

// int gui_imu_sensor_log::render() {

//     ImGui::Text("IMU sensor log tool");
//     ImGui::Separator();
//     ImGui::Text("Notes:");
//     ImGui::Text("- Ensure correct configuration is used.");

//     ImGui::Separator();

//     if (!can_start_) {
//         ImGui::Text("Can't start! Most likely missing signal.");
//     }
//     if (!is_ready_) {
//         ImGui::BeginDisabled();
//     }

//     ImGui::Separator();

//     if (ImGui::Button("Sampler start")) {
//         sampler_.start();
//     }
//     ImGui::SameLine();
//     if (ImGui::Button("Sampler stop")) {
//         sampler_.stop();
//     }
//     ImGui::Separator();

//     // Some nice stats
//     static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
//                                          ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;
//     if (ImGui::BeginTable("Measurements", 2, table_flags)) {
//         ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
//         ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

//         ImGui::TableNextRow();
//         ImGui::TableSetColumnIndex(0); ImGui::Text("imu_idx");
//         ImGui::TableSetColumnIndex(1); ImGui::Text("%u", index_imu_);

//         ImGui::TableNextRow();
//         ImGui::TableSetColumnIndex(0); ImGui::Text("mag_idx");
//         ImGui::TableSetColumnIndex(1); ImGui::Text("%u", index_mag_);


//         ImGui::EndTable();
//     }
//     ImGui::Separator();

//     sampler_.render_interface();
//     ImGui::Separator();

//     sampler_.render_status();
//     sampler_.render_plots();
//     ImGui::Separator();

//     sampler_.channels_write_to_file();



//     // Cleanup, but don't clear is_ready here
//     if (!is_ready_) {
//         ImGui::EndDisabled();
//     }

//     return jcs::RET_OK;
// }
