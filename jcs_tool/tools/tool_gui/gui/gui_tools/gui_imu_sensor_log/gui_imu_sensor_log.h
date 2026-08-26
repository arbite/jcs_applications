// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_IMU_SENSOR_LOGGER_H_
#define GUI_IMU_SENSOR_LOGGER_H_

#include "jcs_host.h"
#include "imgui.h"
#include <array>
#include <string>
#include "helpers.h"
#include "gui_type_base.h"
#include "gui_interface.h"

//////////////////////////////////////////////////////////////////////
class gui_imu_sensor_log : public gui_type_base {
public:
    gui_imu_sensor_log(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_imu_sensor_log() {}

    int startup();
    int step_rt();
    int render();

private:
    bool is_ready_;
    bool can_start_;

    std::vector<uint16_t> accel_line_;
    std::vector<uint16_t> gyro_line_;

    enum class state {
        standby_s,
        sampling_s
    };
    state state_;

    uint32_t imu_param_idx_;
    void imu_sample_start();
    int imu_sample_step();

    // std::vector<float>  storage_acc_;
    // std::vector<float>  storage_gyro_;
    // std::vector<float>  storage_mag_;
    // std::vector<float>  storage_line_;
    // std::vector<uint8_t> storage_idx_;
    // int index_imu_;
    // int index_mag_;
    // bool mag_do_sample_;

    // // sampler sampler_;

    // // std::vector<std::string> required_input_signal_names_;
    // std::vector<std::string> required_output_signal_names_;

    // // std::vector<float> f32_input_signal_store_;
    // std::vector<float> f32_output_signal_store_;


    // int sout_a_a_x_idx_;
    // int sout_a_a_y_idx_;
    // int sout_a_a_z_idx_;
    // int sout_a_b_x_idx_;
    // int sout_a_b_y_idx_;
    // int sout_a_b_z_idx_;
    // int sout_w_a_x_idx_;
    // int sout_w_a_y_idx_;
    // int sout_w_a_z_idx_;
    // int sout_w_b_x_idx_;
    // int sout_w_b_y_idx_;
    // int sout_w_b_z_idx_;

    // int sout_m_x_idx_;
    // int sout_m_y_idx_;
    // int sout_m_z_idx_;

    // int sout_active_imu_idx_;
    // int sout_active_mag_idx_;

    // std::vector<std::string> sensor_names_;

};

#endif