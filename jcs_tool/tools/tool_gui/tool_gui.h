// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
// 
#ifndef TOOL_GUI_H_
#define TOOL_GUI_H_

#include "jcs_tool_if.h"
#include <vector>
#include <string>
#include "imgui.h"

#include "imgui_impl_glfw.h"
#include <GLFW/glfw3.h>
#include "imgui_impl_opengl2.h"

#include <chrono>
#include <atomic>

#include "gui_device_base.h"
#include "gui_device_host.h"
#include "gui_interface.h"

class tool_gui : public jcs_tool_if, public gui_interface {
public:
    tool_gui(std::string name, jcs::jcs_host* host);
    ~tool_gui();

    int load_config(std::string tool_config);

    int step_startup_rt();
    int step_rt();
    void estop_rt();
    int step_shutdown_rt();
    int step_parameter_startup();
    int step_parameter();
    int step_parameter_shutdown();

    // Interface and helpers
    int start();
    int stop();
    int reset();
    std::vector<std::string>* get_f32_input_signal_names();
    std::vector<std::string>* get_f32_output_signal_names();
    std::vector<std::string>* get_u32_input_signal_names();
    std::vector<std::string>* get_u32_output_signal_names();
    std::vector<std::string>* get_u16_input_signal_names();
    std::vector<std::string>* get_u16_output_signal_names();
    std::vector<std::string>* get_u8_input_signal_names();
    std::vector<std::string>* get_u8_output_signal_names();

private:
    int render_display();
    int render_top_display(ImVec2* w_pos, ImVec2* w_size);

    int build_store();

    std::vector<jcs::jcs_device>* device_tree_;
    std::vector<gui_device_base*> store_;
    gui_device_host* host_ptr_;

    // Signal helpers
    std::vector<std::string> f32_input_signal_names_;
    std::vector<std::string> f32_output_signal_names_;
    std::vector<std::string> u32_input_signal_names_;
    std::vector<std::string> u32_output_signal_names_;
    std::vector<std::string> u16_input_signal_names_;
    std::vector<std::string> u16_output_signal_names_;
    std::vector<std::string> u8_input_signal_names_;
    std::vector<std::string> u8_output_signal_names_;

    // Device selection helpers
    // Written by the gui thread in the device selector, read by the rt thread
    // in step_rt(). Worst case rt steps a different entry for one tick.
    std::atomic<int> device_select_idx_;

    enum class run_status {
        running,
        stopped,
        estop
    };
    // Written by BOTH threads: the rt thread via estop_rt(), and the gui
    // thread from the control buttons. Atomic so the estop write cannot be
    // torn or reordered away.
    std::atomic<run_status> run_status_;

    // Set the run status, but never overwrite a latched estop. Only RESET
    // clears an estop, because only RESET clears it on the host.
    void run_status_set(run_status s);

    bool run_start_script_;
    bool run_stop_script_;

    // Imgui
    GLFWwindow* window_;
    ImVec4 clear_color_;
    std::chrono::system_clock::time_point t_next_;
    bool gui_is_init_;

    void style_dracula_darker();
};

#endif