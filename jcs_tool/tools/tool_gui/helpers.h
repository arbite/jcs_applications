// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef HELPERS_H_
#define HELPERS_H_

#include <vector>
#include <string>
#include "imgui.h"
#include "implot.h"
#include "jcs_host.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include "imgui_stdlib.h"
#include <math.h>

#define M_TWO_PI (2.0 * M_PI)

#define PARAM_NOTIFY(cmd, str) if (cmd != jcs::RET_OK) {     \
                                   std::cout << str << "\n"; \
                               }

#define PARAM_NOTIFY_ACTION(cmd, str, action)   if (cmd != jcs::RET_OK) {     \
                                                    std::cout << str << "\n"; \
                                                    action                    \
                                                }

#define PARAM_NOTIFY_OK(cmd, str) if (cmd != jcs::RET_OK) {     \
                                    std::cout << str << "\n"; \
                                    return jcs::RET_OK;       \
                                  }

#define PARAM_NOTIFY_ERROR(cmd, str) if (cmd != jcs::RET_OK) {     \
                                        std::cout << str << "\n"; \
                                        return jcs::RET_ERROR;    \
                                     }

#define PARAM_NOTIFY_CLEANUP_ERROR(cmd, str, cleanup) if ( (cmd) != jcs::RET_OK) {   \
                       std::cout << (str) << "\n"; \
                       cleanup                   \
                       return jcs::RET_ERROR;    \
                    }

#define PARAM_NOTIFY_CLEANUP_OK(cmd, str, cleanup) if ( (cmd) != jcs::RET_OK) {   \
                       std::cout << (str) << "\n"; \
                       cleanup                   \
                       return jcs::RET_OK;    \
                    }

#define CHECK_CLEANUP_OK(cmd, cleanup) if ( (cmd) != jcs::RET_OK) { \
                                           cleanup                  \
                                           return jcs::RET_OK;      \
                                       }

namespace helpers {
    void HelpMarker(const char* desc);

    // Combo signal selection helpers
    void combo_select(std::string const& name, std::vector<std::string> const* sources, int* current_idx, std::string* dest);
    struct combo_source {
        std::string source_;
        int index_;
        combo_source(std::string const& source, int const index);
    };
    void combo_select(std::string const& name, std::vector<std::string> const* sources, combo_source* source);
    void listbox_select(std::string const& name, std::vector<std::string>* sources, int display_max_items, int* current_idx, std::string* dest);
    void result_text_copyable(std::string const& text, int const extra_lines=1);
    void result_text_copyable(std::string const& text, float const& result);
    void result_text_copyable(std::string const& text, int const height, std::vector<float> const& result, int const wrap_after_elements);
    std::string to_string_with_dp(double val, int dp);

    int build_signal_names_list(jcs::jcs_host* host, jcs::signal_type sig_type,
                                std::vector<std::string>* input_signal_names,
                                std::vector<std::string>* output_signal_names);

    int signals_names_contains(std::vector<std::string>* signal_names, std::string const& name, int* found_index) ;
    bool signals_check(std::vector<std::string>* signal_names_store, std::vector<std::string>* required_signal_names);
    bool input_signals_check(std::vector<std::string>* input_signal_names_store, std::vector<std::string>* required_input_signal_names);
    bool output_signals_check(std::vector<std::string>* output_signal_names_store, std::vector<std::string>* required_output_signal_names);

    // utility structure for realtime plot
    struct scrolling_buffer {
        int max_size_;
        int offset_;
        ImVector<ImVec2> data_;

        scrolling_buffer(int max_size = 2000) {
            max_size_ = max_size;
            offset_  = 0;
            data_.reserve(max_size_);
            // update_size(max_size);
        }
        
        void update_size(int new_size) {
            max_size_ = new_size;
            offset_ = 0;
            // data_.resize(max_size_);
            erase();
            data_.reserve(max_size_);
        }

        void add_point(float x, float y) {
            if (data_.size() < max_size_) {
                data_.push_back(ImVec2(x, y));
            } else {
                data_[offset_] = ImVec2(x, y);
                offset_ = (offset_ + 1) % max_size_;
            }
        }

        ImVec2 first_point() {
            if (data_.empty()) {
                return ImVec2(0.0f, 0.0f);
            } else if (data_.size() < max_size_ || offset_ == 0) {
                return data_.front();
            } else {
                return data_[offset_];
            }
        }

        ImVec2 last_point() {
            if (data_.empty()) {
                return ImVec2(0.0f, 0.0f);
            } else if (data_.size() < max_size_ || offset_ == 0) {
                return data_.back();
            } else {
                return data_[offset_ - 1];
            }
        }
        
        ImVec2 operator[](size_t index) {
            if (data_.empty()) {
                return ImVec2(0.0f, 0.0f);
            } else if (data_.size() < max_size_ || offset_ == 0) {
                if (index > data_.size()-1) {
                    return data_.back();
                }
                return data_[index];
            } else {
                return data_[ (offset_ + index) % max_size_ ];
            }
        }

        void erase() {
            if (data_.size() > 0) {
                data_.shrink(0);
                offset_  = 0;
            }
        }

        void plot_line(std::string const& name) {
            ImGui::PushID(name.c_str());
            if (data_.size() == 0) {
                ImPlotPoint zero = ImPlotPoint(0.0f, 0.0f);
                ImPlot::PlotLine(name.c_str(), &zero.x, &zero.y, 1, 0, 0, 2*sizeof(float));
            } else {
                ImPlot::PlotLine(name.c_str(), &data_[0].x, &data_[0].y, data_.size(), 0, offset_, 2*sizeof(float));
            }
            ImGui::PopID();
        }
    };

    // Normalise [-pi, pi]
    double angle_norm_pipi(double angle);
    // Normalise [0, 2pi]
    double angle_norm_2pi(double angle);

    // Snooze for ms
    void sleep_ms(long ms);

    // Time now in ms
    long int time_now_ms();

    // Step response test
    struct plot_measurement {
        std::string name_;
        std::string x_name_;
        std::string y_name_;
        int size_;
        std::vector<float> x_;
        std::vector<float> y_;
        bool plot_cursors_;
        double cursor_tag_[4];
        plot_measurement(std::string const& name, std::string const& x_name, std::string const& y_name, int const size);
        void plot();

        // Sample rate based
        plot_measurement(std::string const& name, std::string const& x_name, std::string const& y_name, int const sample_time_s, int const sample_rate_hz);
        void update_sample_rate(int sample_rate_hz);
        void update_storage_length(int sample_time_s, int sample_rate_hz);
    };

    // Maths helpers
    inline double sq(double v) {
        return v*v;
    }

    struct vec2 {
        double x, y;
        vec2(double _x, double _y) : x(_x), y(_y) {}
        vec2() : x(0.0), y(0.0) {}

        // No checks for overflow - beware
        double& operator[] (size_t idx)       { return ((double*)(void*)(char*)this)[idx]; }
        double  operator[] (size_t idx) const { return ((const double*)(const void*)(const char*)this)[idx]; }

        vec2& operator=(const vec2& v) {
            x = v.x;
            y = v.y;
            return *this;
        }

        vec2 operator-(const vec2& v) const { return vec2(x - v.x, y - v.y); }
        vec2 operator+(const vec2& v) const { return vec2(x + v.x, y + v.y); }
    };

    struct mat22 {
        double a11, a12, a21, a22;

        mat22(double _a11, double _a12, double _a21, double _a22) : a11(_a11), a12(_a12), a21(_a21), a22(_a22) {}
        mat22() : a11(0.0), a12(0.0), a21(0.0), a22(0.0) {}

        // No checks for overflow - beware
        double& operator[] (size_t idx)       { return ((double*)(void*)(char*)this)[idx]; }
        double  operator[] (size_t idx) const { return ((const double*)(const void*)(const char*)this)[idx]; }

        vec2 operator*(const vec2& v) const {
            return vec2(a11*v.x + a12*v.y, a21*v.x + a22*v.y);
        }

        // In place element wise multiply
        mat22& multiply(const double v) {
            a11 *= v;
            a12 *= v;
            a21 *= v;
            a22 *= v;
            return *this;
        }
        // In place element wise divide
        mat22& divide(const double v) {
            return multiply(1.0 / v);
        }

        // Return a transposed matrix
        mat22 T() {
            return mat22(a11, a21, a12, a22);
        }
    };

    struct controller_pd {
        double kp, kd;

        controller_pd(double _kp, double _kd) : kp(_kp), kd(_kd) {}
        controller_pd() : kp(0.0), kd(0.0) {}

        double step_rt(double p_error, double d_error) {
            return kp*p_error + kd*d_error;
        }
    };

    // Simple moving average filter
    class ma_filter {
    public:
        ma_filter(double cutoff_hz, double dt);
        ~ma_filter();

        double step(double value);
        void seed(double seed_value);
        void cutoff_set(double cutoff_hz);
    private:
        double alpha_;
        double u_prev_;
        double dt_;
    };
} // End namespace helpers


#endif