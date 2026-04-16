// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "helpers.h"
#include "jcs_host.h"

#include <vector>
#include <string>
#include "imgui.h"

#ifndef MULTI_CHAN_SAMPLER_H_
#define MULTI_CHAN_SAMPLER_H_

// multi channel sampler
class sampler {
public:
    sampler(int const base_frequency_hz,
        std::vector<std::string>* output_signal_names, int const n_channels,
        int const initial_sample_rate_hz, int const initial_sample_time_s,
        std::vector<std::string>* channel_labels = nullptr);
    ~sampler();

    int startup(double time_now_ns);
    void step_rt(double time_now_ns, std::vector<float>* f32_output_signal_store);

    void render_status();
    void render_interface();
    void render_plots();

    void start();
    void stop();

    int get_sample_time_s()  { return sample_time_s_; }
    int get_sample_rate_hz() { return sample_rate_hz_; }

    int set_sample_time_s(int sample_time_s);
    int set_sample_rate_hz(int sample_rate_hz);

    int channels_write_to_file();

    int get_channel_count() { return static_cast<int>(channels_.size()); }
    int get_channel_data(int channel_idx, std::vector<float>* time_out, std::vector<float>* data_out);

private:
    enum class sampler_state {
        off_s,
        filter_seed_s,
        sampling_s
    };
    sampler_state state_;

    std::vector<std::string>* f32_output_signal_names_;
    std::vector<std::string>* channel_labels_;

    int base_frequency_hz_;
    int sample_rate_hz_;
    int sample_tick_max_;
    int sample_tick_;
    int sample_time_s_;
    int storage_length_;

    double t_start_ns_;

    int sampler_reconfigure();

    struct channel {
        std::string name_;
        std::string source_;
        int source_combo_index_;

        helpers::scrolling_buffer buffer_;
        float signal_filtered_;
        helpers::ma_filter filter_;

        bool fit_y_;
        bool fit_x_;
        bool plot_cursors_;
        double cursor_tag_[4];
        float t_centre_;
        float y_centre_;
        double span_s_;

        channel(std::string const& name, std::string const& source, int storage_length, double base_freq_hz, int sample_rate_hz);
        void plot();
    };
    std::vector<channel*> channels_;
    bool using_filter_;

    void channels_startup(bool use_first_source);
    void channels_set_signals_size(int size);
    void channels_compute();
    void channels_clear();
    void channels_seed_filter(std::vector<float>* input);
    void channels_step_filter(std::vector<float>* input);
    void channels_sample(float time_s);
    void channels_render_select_source();
    int emit_data(std::string const& path_and_file);
};

#endif