// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "sampler.h"
#include "implot.h"
#include <iostream>
#include <iomanip>
#include "ImGuiFileDialog.h"

sampler::sampler(int const base_frequency_hz, std::vector<std::string>* output_signal_names, int const n_channels, int const initial_sample_rate_hz, int const initial_sample_time_s, std::vector<std::string>* channel_labels) :
    base_frequency_hz_(base_frequency_hz),
    f32_output_signal_names_(output_signal_names),
    channel_labels_(channel_labels),
    state_(sampler_state::off_s),
    sample_rate_hz_(initial_sample_rate_hz),
    sample_time_s_(initial_sample_time_s),
    storage_length_( initial_sample_rate_hz * initial_sample_time_s)
{
    // Channels NOT initialised here. We need true dt
    // however - size the vector
    channels_.resize(n_channels);
    if (initial_sample_rate_hz == base_frequency_hz) {
        using_filter_ = false;
    } else {
        using_filter_ = true;
    }
}

sampler::~sampler() {
    for (int i=0; i<channels_.size(); i++) {
        if (channels_[i] != nullptr) {
            delete channels_[i];
        }
    }
}

int sampler::startup(double time_now_ns) {
    // Build the channels first
    if (channel_labels_ != nullptr && channel_labels_->size() != channels_.size()) {
        std::cout << "Warning: Sampler channel labels not same size as channels. Using default channel names\n";
    }
    bool use_labels = (channel_labels_ != nullptr && channel_labels_->size() == channels_.size());
    for (int i = 0; i < channels_.size(); i++) {
        std::string label = use_labels ? channel_labels_->at(i) : ("Channel " + std::to_string(i));
        channels_[i] = new channel(label,
                                   "None",
                                   1000,
                                   static_cast<double>(base_frequency_hz_),
                                   sample_rate_hz_);
    }
    // Configure the sampler
    if (sampler_reconfigure() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    // Start the channels
    channels_startup(false);

    t_start_ns_ = time_now_ns;

    return jcs::RET_OK;
}

// Notes:
// Using dearimgui scrolling buffer which stores x,y points.
// This means extra memory will be used for storing time (stored in x) multiple times.
// But this allows us to use striding, which is the most performant way to plot
void sampler::step_rt(double time_now_ns, std::vector<float>* f32_output_signal_store) {

    switch (state_) {
        default:
        case sampler_state::off_s:
            break;

        case sampler_state::filter_seed_s:
            sample_tick_ = 0;
            channels_seed_filter(f32_output_signal_store);
            state_ = sampler_state::sampling_s;

            // fall through
        case sampler_state::sampling_s:
            channels_step_filter(f32_output_signal_store);
            sample_tick_++;
            if (sample_tick_ < sample_tick_max_) {
                break;
            }
            sample_tick_ = 0;
            // Sample
            float t_s = (float)((time_now_ns - t_start_ns_) * 1e-9);
            channels_sample(t_s);
            break;
    }
}

void sampler::render_status() {
    ImGui::Text("Sampler State: ");
    ImGui::SameLine();
    switch (state_) {
        default:
        case sampler_state::off_s:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");
            break;
        case sampler_state::filter_seed_s:
        case sampler_state::sampling_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Sampling");
            break;
    }
}
void sampler::render_interface() {
    ImGui::Text("Sampler Settings");
    ImGui::Separator();
    ImGui::Text("Base frequency: %uHz", base_frequency_hz_);
    // Get parameters
    {
        int value = sample_rate_hz_;
        ImGui::InputInt("Sample rate (Hz)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            set_sample_rate_hz(value);
        }
    }
    ImGui::Checkbox("Using downsample filter", &using_filter_);
    {
        int value = sample_time_s_;
        ImGui::InputInt("Sample time (s)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            // sample_time_s_ = value;
            set_sample_time_s(value);
        }
    }
    // Get sources
    channels_render_select_source();
}
void sampler::render_plots() {
    for (int i=0; i<channels_.size(); i++) {
        channels_[i]->plot();
    }
}

void sampler::start() {
    channels_clear();
    state_ = sampler_state::filter_seed_s;
}
void sampler::stop() {
    state_ = sampler_state::off_s;
}

int sampler::set_sample_time_s(int sample_time_s) {
    stop();
    sample_time_s_ = sample_time_s;
    return sampler_reconfigure();
}
int sampler::set_sample_rate_hz(int sample_rate_hz) {
    stop();
    sample_rate_hz_ = sample_rate_hz;
    return sampler_reconfigure();
}

int sampler::sampler_reconfigure() {
    // Compute sample rate
    if (sample_rate_hz_ > (int)base_frequency_hz_ ) {
        std::cout << "sampler: Sample rate must be lower than host base_frequency. Clamping to " << (int)base_frequency_hz_ << "\n";
        sample_rate_hz_ = (int)base_frequency_hz_;
    }
    sample_tick_max_ = (int)base_frequency_hz_ / sample_rate_hz_;
    if ((sample_tick_max_*sample_rate_hz_) != (int)base_frequency_hz_) {
        std::cout << "sampler: Sample rate must divide into host base_frequency. Setting to 10Hz\n";
        sample_rate_hz_ = 10;
        sample_tick_max_ = (int)base_frequency_hz_ / sample_rate_hz_;
        return jcs::RET_ERROR;
    }
    sample_tick_ = 0;
    // Compute storage length
    storage_length_ = sample_rate_hz_ * sample_time_s_;
    channels_compute();
    return jcs::RET_OK;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
sampler::channel::channel(std::string const& name, std::string const& source, int storage_length, double base_freq_hz, int sample_rate_hz) :
    name_(name),
    source_(source),
    source_combo_index_(0),
    buffer_(storage_length),
    filter_(sample_rate_hz/2.0, 1.0/base_freq_hz),
    plot_cursors_(false),
    fit_y_(true), fit_x_(true),
    span_s_(0.0)
{
    for (int c=0; c<4; c++) {
        cursor_tag_[c] = 0.0;
    }
}

void sampler::channel::plot() {
    // Imgui internally hashes the text in button to generate an id
    // But! It needs a unique id. Generating heaps of "internal" labelled buttons
    // will generate a lot of not unique ids. Push "this" and use that to generate a hash
    ImGui::PushID(name_.c_str());

    bool buffer_is_empty = buffer_.data_.empty();

    if (buffer_is_empty) {
        ImGui::Text("Waiting on buffer data");
        ImGui::BeginDisabled(true);
    }

    if (ImGui::Checkbox("Show measurement cursors", &plot_cursors_)) {
        cursor_tag_[0] = cursor_tag_[1] = t_centre_;
        cursor_tag_[2] = cursor_tag_[3] = y_centre_;
    }
    ImGui::SameLine();
    if (ImGui::Button("Centre measurement cursors")) {
        cursor_tag_[0] = cursor_tag_[1] = t_centre_;
        cursor_tag_[2] = cursor_tag_[3] = y_centre_;
    }
    ImGui::SameLine();

    ImGui::Checkbox("Fit T axis", &fit_x_);
    ImGui::SameLine();
    ImGui::Checkbox("Fit Y axis", &fit_y_);

    if (ImPlot::BeginPlot(name_.c_str())) {

        ImPlotAxisFlags x_flags = fit_x_ ? ImPlotAxisFlags_AutoFit : 0;
        ImPlotAxisFlags y_flags = fit_y_ ? ImPlotAxisFlags_AutoFit : 0;
        
        ImPlot::SetupAxes("t", "y", x_flags, y_flags);

        double t_min = buffer_.last_point().x - span_s_;
        double t_max = buffer_.last_point().x;
        // With ImGuiCond_Always the plot is nice, but I can't move the T axis around
        // With ImGuiCond_Once I see some rendering artifacts, but I can move the T axis around....
        ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImGuiCond_Once);

        t_centre_ = 0.5f*(ImPlot::GetPlotLimits().X.Max - ImPlot::GetPlotLimits().X.Min) + ImPlot::GetPlotLimits().X.Min;
        y_centre_ = 0.5f*(ImPlot::GetPlotLimits().Y.Max - ImPlot::GetPlotLimits().Y.Min) + ImPlot::GetPlotLimits().Y.Min;

        if (plot_cursors_) {
            // X0
            ImPlot::DragLineX(0, &cursor_tag_[0], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag_[0], ImVec4(1,0,0,1), "%.3f", cursor_tag_[0]);
            // X1
            ImPlot::DragLineX(1, &cursor_tag_[1], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag_[1], ImVec4(1,0,0,1), "%.3f", cursor_tag_[1]);
            // Y0
            ImPlot::DragLineY(2, &cursor_tag_[2], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag_[2], ImVec4(1,0,0,1), "%.3f", cursor_tag_[2]);
            // Y1
            ImPlot::DragLineY(3, &cursor_tag_[3], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag_[3], ImVec4(1,0,0,1), "%.3f", cursor_tag_[3]);
        }

        buffer_.plot_line(source_.c_str());
        ImPlot::EndPlot();
    }
    if (plot_cursors_) {
        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                             ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

        if (ImGui::BeginTable("Measurements", 6, table_flags)) {
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("X_0");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[0]);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("X_1");
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.6f",cursor_tag_[1]);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("delta X");
            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%.6f", cursor_tag_[0] - cursor_tag_[1]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Y_0");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[2]);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("Y_1");
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.6f",cursor_tag_[3]);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("delta Y");
            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%.6f", cursor_tag_[2] - cursor_tag_[3]);

            ImGui::EndTable();
        }
    }

    if (buffer_is_empty) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}

void sampler::channels_startup(bool use_first_source) {
    for (int i=0; i<channels_.size(); i++) {
        if (use_first_source) {
            channels_[i]->source_ = f32_output_signal_names_->at(0);
            channels_[i]->source_combo_index_ = 0;
        } else {
            // use incremental sources
            int source_idx = i >= f32_output_signal_names_->size() ? (f32_output_signal_names_->size()-1) : i;
            channels_[i]->source_ = f32_output_signal_names_->at(source_idx);
            channels_[i]->source_combo_index_ = source_idx;
        }
    }
    channels_compute();
}
void sampler::channels_compute() {
    double cutoff_hz = (double)sample_rate_hz_ / 2.0;
    for (int i=0; i<channels_.size(); i++) {
        channels_[i]->span_s_ = sample_time_s_;
        channels_[i]->filter_.cutoff_set(cutoff_hz);
        channels_[i]->buffer_.update_size(storage_length_);
    }
}
void sampler::channels_clear() {
    for (int i=0; i<channels_.size(); i++) {
        channels_[i]->buffer_.erase();
    }
}
void sampler::channels_render_select_source() {
    for (int i=0; i<channels_.size(); i++) {
        helpers::combo_select(channels_[i]->name_ + " Source", f32_output_signal_names_, &channels_[i]->source_combo_index_, &channels_[i]->source_);
    }
}
void sampler::channels_seed_filter(std::vector<float>* input) {
    for (int i=0; i<channels_.size(); i++) {
        channels_[i]->filter_.seed(input->at( channels_[i]->source_combo_index_ ));
    }
}
void sampler::channels_step_filter(std::vector<float>* input) {
    for (int i=0; i<channels_.size(); i++) {
        if (using_filter_ == true) {
            channels_[i]->signal_filtered_ = channels_[i]->filter_.step(input->at( channels_[i]->source_combo_index_ ));
        } else {
            channels_[i]->signal_filtered_ = input->at( channels_[i]->source_combo_index_ );
        }
    }
}
void sampler::channels_sample(float time_s) {
    for (int i=0; i<channels_.size(); i++) {
        channels_[i]->buffer_.add_point(time_s, channels_[i]->signal_filtered_);
    }
}

int sampler::channels_write_to_file() {
    // Choose file to write to
    if (ImGui::Button("Write sampler channel data to file")) {
        stop();

        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("choose_dir_key", "Choose File", ".csv", config);
    }
    // Display
    if (ImGuiFileDialog::Instance()->Display("choose_dir_key"))  {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            if (emit_data(ImGuiFileDialog::Instance()->GetFilePathName()) != jcs::RET_OK) {
                return jcs::RET_ERROR;
            }
        }
        ImGuiFileDialog::Instance()->Close();
    }
    return jcs::RET_OK;
}

int sampler::emit_data(std::string const& path_and_file) {
    std::cout << "Writing to: " << path_and_file << "\n";

    std::ofstream config_file(path_and_file);

    // Force time to be 3 digits precision, but leave the rest as default
    std::streamsize default_precision = config_file.precision();

    config_file << "t,";
    for (int i=0; i<channels_.size()-1; i++) {
        config_file << channels_[i]->source_ << ",";
    }
    config_file << channels_[channels_.size()-1]->source_ << "\n";

    for (int i=0; i<channels_[0]->buffer_.data_.size(); i++) {
        ImVec2 point = channels_[0]->buffer_[i];

        // Time is 6 sig fig
        config_file << std::fixed << std::setprecision(6) << point.x << ",";

        // Channel data is default
        config_file.precision(default_precision);
        config_file << point.y << ",";
        // Middle channels
        for (int ch=1; ch<channels_.size()-1; ch++) {
            config_file << channels_[ch]->buffer_[i].y << ",";
        }
        // Last channel
        config_file << channels_[channels_.size()-1]->buffer_[i].y << "\n";
    }
    std::cout << "Done\n";
    return jcs::RET_OK;
}

int sampler::get_channel_data(int channel_idx, std::vector<float>* time_out, std::vector<float>* data_out) {
    if (channel_idx < 0 || channel_idx >= static_cast<int>(channels_.size())) {
        return jcs::RET_ERROR;
    }
    helpers::scrolling_buffer& buf = channels_[channel_idx]->buffer_;
    int n = static_cast<int>(buf.data_.size());
    if (n == 0) {
        return jcs::RET_ERROR;
    }
    if (data_out != nullptr) {
        data_out->resize(n);
    }
    if (time_out != nullptr) {
        time_out->resize(n);
    }
    for (int i = 0; i < n; ++i) {
        // operator[] handles the circular offset
        ImVec2 pt = buf[i];
        if (time_out != nullptr) {
            (*time_out)[i] = pt.x;
        }
        if (data_out != nullptr) {
            (*data_out)[i] = pt.y;
        }
    }
    return jcs::RET_OK;
}