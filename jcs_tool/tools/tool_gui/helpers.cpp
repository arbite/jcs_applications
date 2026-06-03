// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "helpers.h"
#include "jcs_user_external.h"
#include <cmath>
#include <algorithm>

// Helper to display a little (?) mark which shows a tooltip when hovered.
// In your own code you may want to display an actual icon if you are using a merged icon fonts (see docs/FONTS.md)
void helpers::HelpMarker(const char* desc) {
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}


void helpers::combo_select(std::string const& name, std::vector<std::string> const* sources, combo_source* source) {
    const char* combo_preview_value = sources->at(source->index_).c_str();

    ImGui::PushID(name.c_str());
    if (ImGui::BeginCombo(name.c_str(), combo_preview_value, 0)) {
        for (int i = 0; i < sources->size(); ++i) {
            const bool is_selected = (source->index_ == i);
            if (ImGui::Selectable(sources->at(i).c_str(), is_selected)) {
                source->index_ = i;
            }
            // Set the initial focus when opening the combo
            // (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        source->source_ = sources->at(source->index_);
        // std::cout << "Got " << name << " " << *dest << "\n";
        ImGui::EndCombo();
    }
    ImGui::PopID();
}
void helpers::combo_select(std::string const& name, std::vector<std::string> const* sources, int* current_idx, std::string* dest) {
    const char* combo_preview_value = sources->at(*current_idx).c_str();

    ImGui::PushID(name.c_str());
    if (ImGui::BeginCombo(name.c_str(), combo_preview_value, 0)) {
        for (int i = 0; i < sources->size(); ++i) {
            const bool is_selected = (*current_idx == i);
            if (ImGui::Selectable(sources->at(i).c_str(), is_selected)) {
                *current_idx = i;
            }
            // Set the initial focus when opening the combo
            // (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        if (dest != nullptr) {
            *dest = sources->at(*current_idx);
        }
        // std::cout << "Got " << name << " " << *dest << "\n";
        ImGui::EndCombo();
    }
    ImGui::PopID();
}

helpers::combo_source::combo_source(std::string const& source, int const index) {
    source_ = source;
    index_ = index;
}

void helpers::listbox_select(std::string const& name, std::vector<std::string>* sources, int display_max_items, int* current_idx, std::string* dest) {
    ImGui::PushID(name.c_str());
    if (ImGui::BeginListBox(name.c_str(), ImVec2(-FLT_MIN, display_max_items * ImGui::GetTextLineHeightWithSpacing()))) {
        for (int i = 0; i < sources->size(); ++i) {
            const bool is_selected = (*current_idx == i);
            if (ImGui::Selectable(sources->at(i).c_str(), is_selected)) {
                *current_idx = i;
            }
            // Set the initial focus when opening the combo
            // (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        if (dest != nullptr) {
            *dest = sources->at(*current_idx);
        }
        ImGui::EndListBox();
    }
    ImGui::PopID();
}

void helpers::result_text_copyable(std::string const& text, int const extra_lines) {
    std::string res_text = text;
    int line_count = 1 + (int)std::count(text.begin(), text.end(), '\n');
    int height = line_count + extra_lines; // extra_lines for padding, default 0
    ImGui::PushID(res_text.c_str());
    ImGui::InputTextMultiline("", &res_text, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * height), ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_AutoSelectAll);
    if (ImGui::SmallButton("Copy")) {
        ImGui::SetClipboardText(text.c_str());
    }
    ImGui::PopID();
}
void helpers::result_text_copyable(std::string const& text, float const& result) {
    std::string res_text = text + std::to_string(result);
    ImGui::PushID(res_text.c_str());
    ImGui::InputText("", &res_text, ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_AutoSelectAll);
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy")) {
        ImGui::SetClipboardText(text.c_str());
    }
    ImGui::PopID();
}
void helpers::result_text_copyable(std::string const& text, int const height, std::vector<float> const& result, int const wrap_after_elements) {
   std::string res_text = text + "[ ";
   // Build an indent string
   std::string indent(res_text.length(), ' ');

   for (int i = 0; i < result.size(); ++i) {
       if (i > 0) {
           res_text += ", ";
           // Add newline and indent after every wrap_after_elements
           if (i % wrap_after_elements == 0) {
               res_text += "\n" + indent;
           }
       }
       // res_text += std::to_string(result[i]);
        char buffer[32];
        // 10 chars wide, 8 decimal places
        snprintf(buffer, sizeof(buffer), "%10.8f", result[i]);
        // Add a space to line up if not negative
        if (result[i] >= 0.0f) {
            res_text += " ";
        }
        res_text += buffer;
   }
   res_text += " ]";

   ImGui::PushID(res_text.c_str());
   ImGui::InputTextMultiline("", &res_text, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * height), ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_AutoSelectAll);
    if (ImGui::SmallButton("Copy")) {
        ImGui::SetClipboardText(text.c_str());
    }
   ImGui::PopID();
}

std::string helpers::to_string_with_dp(double val, int dp) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(dp) << val;
    return out.str();
}

int helpers::build_signal_names_list(jcs::jcs_host* host, jcs::signal_type sig_type,
                                      std::vector<std::string>* input_signal_names,
                                      std::vector<std::string>* output_signal_names) {
    // Inputs
    for (int i=0; i<host->sig_input_sz_unsafe_rt(sig_type, 0); i++) {
        std::string node_name;
        if (host->sig_input_node_name_get(sig_type, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "build_signal_names_list: Error getting node name for input signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host->sig_input_name_get(sig_type, 0, i, &name) != jcs::RET_OK) {
            std::cout << "build_signal_names_list: Error getting input signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        input_signal_names->push_back(node_name + "::" + name);
    }
    // Outputs
    for (int i=0; i<host->sig_output_sz_unsafe_rt(sig_type, 0); i++) {
        std::string node_name;
        if (host->sig_output_node_name_get(sig_type, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "build_signal_names_list: Error getting node name for output signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host->sig_output_name_get(sig_type, 0, i, &name) != jcs::RET_OK) {
            std::cout << "build_signal_names_list: Error getting output signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        output_signal_names->push_back(node_name + "::" + name);
    }
    return jcs::RET_OK;
}

int helpers::signals_names_contains(std::vector<std::string>* signal_names, std::string const& name, int* found_index) {
    for (int i=0; i<signal_names->size(); i++) {
        if (signal_names->at(i).find(name) != std::string::npos) {
            *found_index = i;
            return jcs::RET_OK;
        }
    }
    return jcs::RET_ERROR;
}

bool helpers::signals_check(std::vector<std::string>* signal_names_store, std::vector<std::string>* required_signal_names) {
    bool got_all = true;
    int dummy = 0;
    for (int i=0; i<required_signal_names->size(); i++) {
        if (signals_names_contains(signal_names_store, required_signal_names->at(i), &dummy) == jcs::RET_OK) {
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "%s ", required_signal_names->at(i).c_str());
        } else {
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "%s ", required_signal_names->at(i).c_str());
            got_all = false;
        }
        if (i != required_signal_names->size()-1) {
            ImGui::SameLine();
        }
    }
    return got_all;
}

bool helpers::input_signals_check(std::vector<std::string>* input_signal_names_store, std::vector<std::string>* required_input_signal_names) {
    ImGui::Text("Required input signals check:  ");
    ImGui::SameLine();
    return signals_check(input_signal_names_store, required_input_signal_names);
}
bool helpers::output_signals_check(std::vector<std::string>* output_signal_names_store, std::vector<std::string>* required_output_signal_names) {
    ImGui::Text("Required output signals check: ");
    ImGui::SameLine();
    return signals_check(output_signal_names_store, required_output_signal_names);
}


// Normalise [-pi, pi]
double helpers::angle_norm_pipi(double angle) {
    double angle_norm = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle_norm < 0) {
        angle_norm += 2.0 * M_PI;
    }
    return angle_norm - M_PI;
}

// Normalise [0, 2pi]
double helpers::angle_norm_2pi(double angle) {
    double angle_norm = std::fmod(angle, 2.0 * M_PI);
    if (angle_norm < 0) {
        angle_norm += 2.0 * M_PI;
    }
    return angle_norm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void helpers::sleep_ms(long ms) {
    jcs::external::sleep_us(ms * 1e3);
}

long int helpers::time_now_ms() {
    return jcs::external::time_now_ns() / 1e6;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nice plotting helper
helpers::plot_measurement::plot_measurement(std::string const& name, std::string const& x_name, std::string const& y_name, int const size) :
    name_(name), x_name_(x_name), y_name_(y_name), size_(size), plot_cursors_(false)
{
    x_.resize(size_);
    y_.resize(size_);
    for (int i=0; i<4; i++) {
        cursor_tag_[i] = 0.0;
    }
}
helpers::plot_measurement::plot_measurement(std::string const& name, std::string const& x_name, std::string const& y_name, int const sample_time_s, int const sample_rate_hz) :
    plot_measurement(name, x_name, y_name, sample_time_s * sample_rate_hz)
{
    update_sample_rate(sample_rate_hz);
}

void helpers::plot_measurement::plot() {
    ImGui::PushID(name_.c_str());

    ImGui::Checkbox("Show measurement cursors", &plot_cursors_);
    ImGui::SameLine();
    ImGui::Text("(Double click plot to zoom to extents. Right click plot for menu.)");

    if (ImPlot::BeginPlot(name_.c_str())) {
        ImPlot::SetupAxes(x_name_.c_str(), y_name_.c_str());

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
        ImPlot::PlotLine(name_.c_str(), &x_[0], &y_[0], x_.size());
        ImPlot::EndPlot();
    }

    if (plot_cursors_) {
        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                             ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

        if (ImGui::BeginTable("Measurements", 2, table_flags)) {
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta %s", x_name_.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[0] - cursor_tag_[1]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta %s", y_name_.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[2] - cursor_tag_[3]);
            ImGui::EndTable();
        }
    }
    ImGui::PopID();
}

void helpers::plot_measurement::update_sample_rate(int sample_rate_hz) {
    for (int i=0; i<x_.size(); i++) {
        x_[i] = (float)i / (float)sample_rate_hz;
    }
}

void helpers::plot_measurement::update_storage_length(int sample_time_s, int sample_rate_hz) {
    int new_sample_length = sample_time_s * sample_rate_hz;
    x_.resize(new_sample_length);
    y_.resize(new_sample_length);
    update_sample_rate(sample_rate_hz);
}


helpers::ma_filter::ma_filter(double cutoff_hz, double dt) {
    u_prev_ = 0.0;
    dt_ = dt;
    cutoff_set(cutoff_hz);
}
helpers::ma_filter::~ma_filter() {}

double helpers::ma_filter::step(double value) {
    double delta = value - u_prev_;
    double u_temp = u_prev_ + alpha_ * delta;
    u_prev_ = u_temp;
    return u_temp;
}
void helpers::ma_filter::seed(double seed_value) {
    u_prev_ = seed_value;
}
void helpers::ma_filter::cutoff_set(double cutoff_hz) {
    alpha_ = (dt_ / (dt_ + 1.0 / (2.0 * M_PI * cutoff_hz)));
}