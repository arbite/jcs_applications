// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "tool_gui.h"
#include "imgui.h"
#include "implot.h"
#include "tool_gui_settings.h"
#include <string>
#include <iostream>
#include <thread>

#include "gui_device_host.h"
#include "gui_device_joint_controller.h"
#include "gui_device_motor_controller.h"
#include "gui_device_encoder_absolute.h"
#include "gui_device_encoder_absolute_slide_by_hall.h"
#include "gui_device_braking_chopper.h"
#include "gui_device_encoder_relative.h"
#include "gui_device_strain_gauge.h"
#include "gui_device_brake_clutch.h"
#include "gui_device_analog.h"
#include "gui_device_load_switch.h"
#include "gui_device_thermal_simple.h"

#include "gui_process_pid.h"
#include "gui_process_pd.h"
#include "gui_process_interpolator.h"
#include "gui_process_transform.h"
#include "gui_process_map.h"

tool_gui::tool_gui(std::string name, jcs::jcs_host* host) :
    // tool gui does not use mem lock just yet
    jcs_tool_if(name, host, false),
    run_start_script_(true), run_stop_script_(true)
{
    host_ptr_ = nullptr;
    device_select_idx_ = 0;
    gui_is_init_ = false;
    run_status_ = run_status::stopped;
}

int tool_gui::load_config(std::string tool_config) {
    return jcs::RET_OK;
}


int tool_gui::step_startup_rt() {
    // if (host_ptr_ == nullptr) {
    //     return jcs::RET_ERROR;
    // }
    return jcs::RET_OK;
}

int tool_gui::step_rt() {
    // Host might have things to always tick over
    // Note: Ok to call on host_ptr_ as this function will not be called
    // until host_ptr_ is attached and store_ is populated
    if (host_ptr_->step_rt_always() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    // Exchange data with graph storage
    if (store_[device_select_idx_]->step_rt() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    return jcs::RET_OK;
}

int tool_gui::step_shutdown_rt() {
    return jcs::RET_OK;
}

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int tool_gui::step_parameter_startup() {
    // Ensure this thread stays on CPU 0
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(0, &cpu_set);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);

    // Start network configuration
    if (host_->start_network() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        return jcs::RET_ERROR;
    }

    // Create window with graphics context
    window_ = glfwCreateWindow(1280, 720, "A R B I T E   [host_interface]", nullptr, nullptr);
    if (window_ == nullptr) {
        return jcs::RET_ERROR;
    }
    glfwMakeContextCurrent(window_);

    // Note: vsync doesnt seem to be very stable. Makes the plots wobbly
    // glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    ImPlot::CreateContext();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL2_Init();

    // We need to call gui shutdown stuff when we exit from now on
    gui_is_init_ = true;

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);

    clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    style_dracula_darker();

    // Build the device store and startup
    device_tree_ = host_->external_info_tree_get();
    build_store();

    // Populate some nice helpers
    // Build combo box names lists
    if (helpers::build_signal_names_list(host_, jcs::signal_type::float32_s, &f32_input_signal_names_, &f32_output_signal_names_) != jcs::RET_OK) { return jcs::RET_ERROR; }
    if (helpers::build_signal_names_list(host_, jcs::signal_type::uint32_s,  &u32_input_signal_names_, &u32_output_signal_names_) != jcs::RET_OK) { return jcs::RET_ERROR; }
    if (helpers::build_signal_names_list(host_, jcs::signal_type::uint16_s,  &u16_input_signal_names_, &u16_output_signal_names_) != jcs::RET_OK) { return jcs::RET_ERROR; }
    if (helpers::build_signal_names_list(host_, jcs::signal_type::uint8_s,   &u8_input_signal_names_,  &u8_output_signal_names_)  != jcs::RET_OK) { return jcs::RET_ERROR; }

    for (int i=0; i<store_.size(); i++) {
        if (store_[i]->startup() != jcs::RET_OK) {
            return jcs::RET_ERROR;
        }
    }

    t_next_ = std::chrono::system_clock::now();

    return jcs::RET_OK;
}

void tool_gui::build_store() {
    for (int i=0; i<device_tree_->size(); i++) {
        gui_interface* gui_if = static_cast<gui_interface*>(this);
        // Add any devices to the store
        if (device_tree_->at(i).node_type == "dev_host") {
            store_.push_back(new gui_device_host(host_, gui_if, device_tree_->at(i).name));
            host_ptr_ = static_cast<gui_device_host*>(store_[0]);
        }
        else if (device_tree_->at(i).node_type == "dev_joint_controller")                { store_.push_back(new gui_device_joint_controller(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_motor_controller")                { store_.push_back(new gui_device_motor_controller(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_encoder_absolute")                { store_.push_back(new gui_device_encoder_absolute(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_encoder_absolute_slide_by_hall")  { store_.push_back(new gui_device_encoder_absolute_slide_by_hall(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_braking_chopper")                 { store_.push_back(new gui_device_braking_chopper(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_encoder_relative")                { store_.push_back(new gui_device_encoder_relative(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_strain_gauge")                    { store_.push_back(new gui_device_strain_gauge(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_brake_clutch")                    { store_.push_back(new gui_device_brake_clutch(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_analog")                          { store_.push_back(new gui_device_analog(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_load_switch")                     { store_.push_back(new gui_device_load_switch(host_, gui_if, device_tree_->at(i).name)); }
        else if (device_tree_->at(i).node_type == "dev_thermal_simple")                  { store_.push_back(new gui_device_thermal_simple(host_, gui_if, device_tree_->at(i).name)); }
        // Nothing to do
        else { }

        // Any processes to add?
        for (int p=0; p<device_tree_->at(i).procs.size(); p++) {
            if (device_tree_->at(i).procs[p].node_type == "proc_pid")                 { store_.push_back(new gui_process_pid(host_, gui_if, device_tree_->at(i).procs[p].name)); }
            else if (device_tree_->at(i).procs[p].node_type == "proc_pd")             { store_.push_back(new gui_process_pd(host_, gui_if, device_tree_->at(i).procs[p].name)); }
            else if (device_tree_->at(i).procs[p].node_type == "proc_interpolator")   { store_.push_back(new gui_process_interpolator(host_, gui_if, device_tree_->at(i).procs[p].name)); }
            else if (device_tree_->at(i).procs[p].node_type == "proc_transform")      { store_.push_back(new gui_process_transform(host_, gui_if, device_tree_->at(i).procs[p].name)); }
            else if (device_tree_->at(i).procs[p].node_type == "proc_map")            { store_.push_back(new gui_process_map(host_, gui_if, device_tree_->at(i).procs[p].name)); }
            // Nothing to do
            else { }
        }
    }
}

int tool_gui::step_parameter() {

    t_next_ += std::chrono::milliseconds(50);

    glfwPollEvents();

    if (glfwWindowShouldClose(window_)) {
        return jcs::RET_ERROR;
    }

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (render_display() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color_.x * clear_color_.w, clear_color_.y * clear_color_.w, clear_color_.z * clear_color_.w, clear_color_.w);
    glClear(GL_COLOR_BUFFER_BIT);

    // If you are using this code with non-legacy OpenGL header/contexts (which you should not, prefer using imgui_impl_opengl3.cpp!!),
    // you may need to backup/reset/restore other state, e.g. for current shader using the commented lines below.
    //GLint last_program;
    //glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
    //glUseProgram(0);
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    //glUseProgram(last_program);

    glfwMakeContextCurrent(window_);
    glfwSwapBuffers(window_);

    std::this_thread::sleep_until(t_next_);

    return jcs::RET_OK;
}

int tool_gui::step_parameter_shutdown() {
    // Cleanup
    if (gui_is_init_) {
        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        ImPlot::DestroyContext();

        glfwDestroyWindow(window_);
        glfwTerminate();
    }

    return jcs::RET_OK;
}

int tool_gui::render_display() {
    int w, h;

    glfwGetFramebufferSize(window_, &w, &h);

    ImVec2 window_pos(0.0f, 0.0f);
    ImVec2 window_size((float)w, (float)h);

    bool p_open = false;
    ImGui::SetNextWindowPos(window_pos);
    ImGui::SetNextWindowSize(window_size);
    if (!ImGui::Begin("A R B I T E   [host_interface]", &p_open, ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return jcs::RET_OK;
    }

    ImGui::Text("A R B I T E    [host_interface]");
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 300);

    ImGui::Text("Application: %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::Separator();

    // Control buttons
    if (ImGui::Button("START")) {
        if (host_->ready_devices() == jcs::RET_OK) {
            if (host_->start(run_start_script_) == jcs::RET_OK) {
                run_status_ = run_status::running;
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("STOP")) {
        run_status_ = run_status::stopped;
        if (host_->stop(run_stop_script_) == jcs::RET_OK) {
            host_->dev_jc_ethercat_timing_print();
            host_->process_timing_print();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("RESET")) {
        run_status_ = run_status::stopped;
        host_->reset();
    }
    ImGui::SameLine();
    if (ImGui::Button("ESTOP")) {
        run_status_ = run_status::stopped;
        host_->trigger_estop();
    }
    ImGui::SameLine();
    if (ImGui::Button("Host Error/Info")) {
        host_->host_overrun_counts_print();
        host_->device_error_counts_print();
        host_->device_error_estop_print();
        host_->process_timing_print();
        host_->dev_jc_ethercat_timing_print();
    }
    ImGui::SameLine();
    if (ImGui::Button("SHUTDOWN")) {
        run_status_ = run_status::stopped;
        host_->stop(run_stop_script_);
        host_->shutdown();
        // Signal to shutdown
        ImGui::End();
        return jcs::RET_ERROR;
    }
    ImGui::SameLine();

    // If an estop is present, has_estop will only return true for one call
    if (host_->has_estop()) {
        run_status_ = run_status::estop;
        host_->device_error_estop_print();
    }

    switch (run_status_) {
        default:
        case run_status::stopped:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Stopped");
            break;
        case run_status::running:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Running");
            break;
        case run_status::estop:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "- ERROR: E-STOP! -");
            break;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Run start script", &run_start_script_);
    ImGui::SameLine();
    ImGui::Checkbox("Run stop script", &run_stop_script_);

    // Nifty Imgui metrics
    // ImGui::SameLine();
    // static bool show_tool_metrics = false;
    // ImGui::Checkbox("Metrics", &show_tool_metrics);
    // if (show_tool_metrics) {
    //     ImGui::ShowMetricsWindow(&show_tool_metrics);
    // }

    ImGui::Separator();

    // Left pane device/proc selector
    {
        ImGui::BeginChild("left pane", ImVec2(150, 0), ImGuiChildFlags_Border | ImGuiChildFlags_ResizeX);
        for (int i=0; i<store_.size(); ++i) {
            const bool is_selected = (device_select_idx_ == i);
            if (ImGui::Selectable(store_[i]->name_get().c_str(), is_selected)) {
                device_select_idx_ = i;
            }
        }
        ImGui::EndChild();
    }
    ImGui::SameLine();

    // Render content section
    ImGui::BeginChild("Content", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
    ImGui::Text("Name: %s", store_[device_select_idx_]->name_get().c_str());
    ImGui::Separator();
    if (store_[device_select_idx_]->render() != jcs::RET_OK) {
        ImGui::EndChild();
        ImGui::End();
        return jcs::RET_ERROR;
    }
    ImGui::EndChild();

    ImGui::End();

    return jcs::RET_OK;

}


int tool_gui::start() {
    if (host_->ready_devices() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    if (host_->start(run_start_script_) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    run_status_ = run_status::running;
    return jcs::RET_OK;
}
int tool_gui::stop() {
    run_status_ = run_status::stopped;
    if (host_->stop(run_stop_script_) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    host_->dev_jc_ethercat_timing_print();
    host_->process_timing_print();
    return jcs::RET_OK;
}
int tool_gui::reset() {
    if (run_status_ != run_status::stopped) {
        if (stop() != jcs::RET_OK) {
            return jcs::RET_ERROR;
        }
    }
    if (host_->reset() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    return jcs::RET_OK;
}
std::vector<std::string>* tool_gui::get_f32_input_signal_names() {
    return &f32_input_signal_names_;
}
std::vector<std::string>* tool_gui::get_f32_output_signal_names() {
    return &f32_output_signal_names_;
}

std::vector<std::string>* tool_gui::get_u32_input_signal_names() {
    return &u32_input_signal_names_;
}
std::vector<std::string>* tool_gui::get_u32_output_signal_names() {
    return &u32_output_signal_names_;
}

std::vector<std::string>* tool_gui::get_u16_input_signal_names() {
    return &u16_input_signal_names_;
}
std::vector<std::string>* tool_gui::get_u16_output_signal_names() {
    return &u16_output_signal_names_;
}

std::vector<std::string>* tool_gui::get_u8_input_signal_names() {
    return &u8_input_signal_names_;
}
std::vector<std::string>* tool_gui::get_u8_output_signal_names() {
    return &u8_output_signal_names_;
}

// Extracted from
// https://github.com/pthom/hello_imgui/tree/master/src/hello_imgui/impl
void tool_gui::style_dracula_darker()
{
    ImGuiStyle* style = &ImGui::GetStyle();
    ImVec4* colors = style->Colors;

    colors[ImGuiCol_Text]                   = ImVec4(0.88f, 0.88f, 0.88f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.14f, 0.14f, 0.15f, 0.86f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.14f, 0.14f, 0.15f, 0.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.14f, 0.14f, 0.15f, 0.86f);
    colors[ImGuiCol_Border]                 = ImVec4(0.33f, 0.33f, 0.33f, 0.46f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.15f, 0.15f, 0.15f, 0.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.42f, 0.42f, 0.42f, 0.50f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.45f, 0.63f, 0.98f, 0.62f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.46f, 0.46f, 0.46f, 0.62f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.04f, 0.04f, 0.04f, 1.00f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.00f, 0.00f, 0.00f, 0.47f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.16f, 0.26f, 0.47f, 1.00f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.27f, 0.27f, 0.28f, 0.74f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.27f, 0.27f, 0.28f, 0.55f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.40f, 0.44f, 0.51f, 0.47f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.22f, 0.28f, 0.41f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.14f, 0.18f, 0.26f, 0.84f);
    colors[ImGuiCol_CheckMark]              = ImVec4(0.88f, 0.88f, 0.88f, 0.76f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(0.68f, 0.68f, 0.68f, 0.57f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.29f, 0.29f, 0.29f, 0.77f);
    colors[ImGuiCol_Button]                 = ImVec4(0.33f, 0.34f, 0.35f, 0.45f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.22f, 0.28f, 0.41f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.14f, 0.18f, 0.26f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.46f, 0.47f, 0.50f, 0.49f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.45f, 0.63f, 0.98f, 0.62f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.46f, 0.46f, 0.46f, 0.62f);
    colors[ImGuiCol_Separator]              = ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(0.98f, 0.98f, 0.98f, 0.78f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.98f, 0.98f, 0.98f, 0.55f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.98f, 0.98f, 0.98f, 0.83f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.18f, 0.31f, 0.57f, 0.79f);
    colors[ImGuiCol_TabHovered]             = ImVec4(0.26f, 0.50f, 0.96f, 0.74f);
    colors[ImGuiCol_TabActive]              = ImVec4(0.20f, 0.36f, 0.67f, 1.00f);
    colors[ImGuiCol_TabUnfocused]           = ImVec4(0.07f, 0.09f, 0.14f, 0.89f);
    colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.13f, 0.23f, 0.42f, 1.00f);
    // colors[ImGuiCol_DockingPreview]         = ImVec4(0.26f, 0.50f, 0.96f, 0.64f);
    // colors[ImGuiCol_DockingEmptyBg]         = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_PlotLines]              = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    colors[ImGuiCol_PlotLinesHovered]       = ImVec4(0.35f, 0.56f, 0.98f, 1.00f);
    colors[ImGuiCol_PlotHistogram]          = ImVec4(0.01f, 0.30f, 0.88f, 1.00f);
    colors[ImGuiCol_PlotHistogramHovered]   = ImVec4(0.01f, 0.34f, 0.98f, 1.00f);
    colors[ImGuiCol_TableHeaderBg]          = ImVec4(0.18f, 0.19f, 0.20f, 1.00f);
    colors[ImGuiCol_TableBorderStrong]      = ImVec4(0.30f, 0.32f, 0.34f, 1.00f);
    colors[ImGuiCol_TableBorderLight]       = ImVec4(0.22f, 0.23f, 0.24f, 1.00f);
    colors[ImGuiCol_TableRowBg]             = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt]          = ImVec4(0.98f, 0.98f, 0.98f, 0.06f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.18f, 0.39f, 0.78f, 0.83f);
    colors[ImGuiCol_DragDropTarget]         = ImVec4(0.01f, 0.34f, 0.98f, 0.83f);
    colors[ImGuiCol_NavHighlight]           = ImVec4(0.26f, 0.50f, 0.96f, 1.00f);
    colors[ImGuiCol_NavWindowingHighlight]  = ImVec4(0.98f, 0.98f, 0.98f, 0.64f);
    colors[ImGuiCol_NavWindowingDimBg]      = ImVec4(0.78f, 0.78f, 0.78f, 0.20f);
    colors[ImGuiCol_ModalWindowDimBg]       = ImVec4(0.78f, 0.78f, 0.78f, 0.35f);
}