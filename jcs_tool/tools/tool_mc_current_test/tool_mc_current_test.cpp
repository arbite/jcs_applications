// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "tool_mc_current_test.h"

#include "jcs_host.h"
#include "task_rt.h"
#include "recorder.h"
#include <string>
#include <iostream>
#include "config.h"
#include "jcs_user_external.h"

// This tool applies a test current to the D axis
// while rotating the motor.
// The motor is slowly rotated to distribute the generated heat
// evenly around the motor.
//
// System will shut down when an over temperature causes an e-stop

tool_mc_current_test::tool_mc_current_test(std::string name, jcs::jcs_host* host) :
    jcs_tool_if(name, host, true)
{
    ramp_i_energise_   = 5.0;
    ramp_i_ramp_time_  = 2.0;
    ramp_i_dwell_time_ = 2.0;

    rotate_start_       = 0.0;
    rotate_n_rotations_ = 1.0;
    rotate_time_        = 10.0;

    host_sigs_out_sz_ = 0;
    host_sigs_in_sz_  = 0;
    sig_index_i_d_  = 0;
    sig_index_th_m_ = 0;
    sig_index_w_m_  = 0;
}

// tool_mc_current_test::~tool_mc_current_test::() {}


int tool_mc_current_test::load_config(std::string tool_config) {
    // Tool requires a config
    if (tool_config.empty()) {
        std::cout << "Error: tool_mc_current_test requires config file with -tc.\n";
        return jcs::RET_ERROR; 
    }
    // Open configuration file
    YAML::Node conf;
    if (!config::get_yaml_doc(tool_config, conf)) {
        std::cout << "tool_mc_current_test: No config file found - using defaults\n";
        return jcs::RET_ERROR;
    }

    ramp_i_energise_   = conf["ramp_i_energise"].as<double>();
    ramp_i_ramp_time_  = conf["ramp_i_ramp_time"].as<double>();
    ramp_i_dwell_time_ = conf["ramp_i_dwell_time"].as<double>();

    rotate_start_       = conf["rotate_theta_start"].as<double>();
    rotate_n_rotations_ = conf["rotate_n_rotations"].as<double>();
    rotate_time_        = conf["rotate_time"].as<double>();

    std::cout << "tool_mc_current_test: Got n_rotations: " << rotate_n_rotations_ << std::endl;
    std::cout << "tool_mc_current_test: Got rotate_time: " << rotate_time_ << std::endl;

    return jcs::RET_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
int tool_mc_current_test::step_startup_rt() {

    // System is initialised by the time we get here - we can get info from dev_host
    const static double base_frequency_hz = static_cast<double>(host_->base_frequency_get());
    const static double dt = 1.0 / base_frequency_hz;

    // Initialise tools
    i_ramp_ = new ramp(dt);
    i_rotate_ = new rotate(dt);

    // Get signal sizes
    host_sigs_out_sz_ = host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0);
    host_sigs_in_sz_  = host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0);

    // Get names of float signals
    std::vector<std::string> host_sigs_out_names(host_sigs_out_sz_);
    if (host_->sig_output_name_get(jcs::signal_type::float32_s, 0, &host_sigs_out_names) != jcs::RET_OK) {
        std::cout << "tool_mc_current_test: Could not get output signal names\n";
        return jcs::RET_ERROR;
    }
    std::vector<std::string> host_sigs_in_names(host_sigs_in_sz_);
    if (host_->sig_input_name_get(jcs::signal_type::float32_s, 0, &host_sigs_in_names) != jcs::RET_OK) {
        std::cout << "tool_mc_current_test: Could not get input signal names\n";
        return jcs::RET_ERROR;
    }

    for (int i=0; i<host_sigs_in_names.size(); i++) {
        std::cout << "tool_mc_current_test: Input signal " << i << " name: " << host_sigs_in_names[i] << std::endl;
    }
    for (int i=0; i<host_sigs_out_names.size(); i++) {
        std::cout << "tool_mc_current_test: Output signal " << i << " name: " << host_sigs_out_names[i] << std::endl;
    }

    // Get some helpful signal indices
    if (host_->sig_input_index_get(jcs::signal_type::float32_s, 0, "host_i_d", &sig_index_i_d_) != jcs::RET_OK) {
        std::cout << "tool_mc_current_test: Host signal host_i_d not found\n";
        return jcs::RET_ERROR;
    }
    if (host_->sig_input_index_get(jcs::signal_type::float32_s, 0, "host_th_m", &sig_index_th_m_) != jcs::RET_OK) {
        std::cout << "tool_mc_current_test: Host signal host_th_m not found\n";
        return jcs::RET_ERROR;
    }
    if (host_->sig_input_index_get(jcs::signal_type::float32_s, 0, "host_w_m", &sig_index_w_m_) != jcs::RET_OK) {
        std::cout << "tool_mc_current_test: Host signal host_w_m not found\n";
        return jcs::RET_ERROR;
    }

    // Resize the per tick signal storage
    signals_out_f_.resize(host_sigs_out_sz_);
    signals_in_f_.resize(host_sigs_in_sz_);

    // We will record: Timestamp, input signals, output signals
    signals_record_f_.resize(1 + host_sigs_in_sz_ + host_sigs_out_sz_);
    // Build the header vector
    signals_record_header_.push_back("timestamp_ns");
    signals_record_header_.insert(signals_record_header_.end(), host_sigs_in_names.begin(), host_sigs_in_names.end());
    signals_record_header_.insert(signals_record_header_.end(), host_sigs_out_names.begin(), host_sigs_out_names.end());

    // Configure data recorders
    int n_points_fixed =  static_cast<int>(ramp_i_ramp_time_ * base_frequency_hz);
    int n_points_rotate = static_cast<int>(rotate_time_ * base_frequency_hz);

    // Initialise the signal recorders
    rec_fixed_ = new recorder(signals_record_f_.size(), n_points_fixed, "data_fixed.csv");
    rec_rotate_ = new recorder(signals_record_f_.size(), n_points_rotate, "data_rotate.csv");

    // Ready to go
    state_ = behaviour_state::ramp_to_current_s;
    i_ramp_->start(0.0, ramp_i_energise_, ramp_i_ramp_time_, 0.5, ramp_i_dwell_time_);

    // Sample start time
    t_0_data_ = jcs::external::time_now_ns();

    return jcs::RET_OK;
}
int tool_mc_current_test::step_rt() {
    host_->sig_input_set_rt(0, signals_in_f_);
    host_->sig_output_get_rt(0, &signals_out_f_);

    // Assemble recording vector
    signals_record_f_[0] = static_cast<float>(jcs::external::time_now_ns() - t_0_data_);
    std::copy(signals_in_f_.begin(),  signals_in_f_.end(),  signals_record_f_.begin()+1);
    std::copy(signals_out_f_.begin(), signals_out_f_.end(), signals_record_f_.begin()+1+host_sigs_in_sz_);

    switch (state_) {
        default:
        case behaviour_state::ramp_to_current_s:
            signals_in_f_[sig_index_i_d_] = i_ramp_->step();
            // Record dwell data
            if (i_ramp_->in_dwell()) {
                rec_fixed_->add(signals_record_f_);
            }
            if (i_ramp_->is_done()) {
                i_rotate_->start(rotate_start_, rotate_n_rotations_, true, rotate_time_);
                state_ = behaviour_state::rotate_axis_s;
                // Reset start time for rotating data storage
                t_0_data_ = jcs::external::time_now_ns();
            }
            break;

        case behaviour_state::rotate_axis_s:
            signals_in_f_[sig_index_th_m_] = i_rotate_->step();
            signals_in_f_[sig_index_w_m_]  = i_rotate_->omega();

            // Record rotating data
            rec_rotate_->add(signals_record_f_);

            if (i_rotate_->is_done()) {
                // All done. Shut down the system
                return jcs::RET_NRDY;
            }
            break;


    }
    return jcs::RET_OK;
}
int tool_mc_current_test::step_shutdown_rt() {
    // Write out our recorded data
    std::cout << "Writing out data" << std::endl;
    rec_fixed_->write_to_file(signals_record_header_);
    rec_rotate_->write_to_file(signals_record_header_);
    return jcs::RET_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
int tool_mc_current_test::step_parameter_startup() {
    // Start network configuration
    if (host_->start_network() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }

    if (host_->start() != jcs::RET_OK) {
        host_->shutdown();
        return jcs::RET_ERROR;
    }
    return jcs::RET_OK;
}

int tool_mc_current_test::step_parameter() {
    host_->estop_decode_maybe();
    return jcs::RET_OK;
}

int tool_mc_current_test::step_parameter_shutdown() {
    host_->stop();
    host_->shutdown();
    return jcs::RET_OK;
}