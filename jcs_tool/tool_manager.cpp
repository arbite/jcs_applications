// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "tool_manager.h"
#include <iostream>
#include <vector>

// Tools
#include "tools/tool_id/tool_id.h"
#include "tools/tool_mc_current_test/tool_mc_current_test.h"
#include "tools/tool_gui/tool_gui.h"

tool_manager::tool_manager(jcs::jcs_host* host) {
    
    host_ = host;
    active_tool_idx_ = 0;

    // Add our tools
    storage_.push_back(new tool_id("tool_id", host));
    storage_.push_back(new tool_mc_current_test("tool_mc_current_test", host));
    storage_.push_back(new tool_gui("tool_gui", host));

}

tool_manager::~tool_manager() {}

int tool_manager::set_active_tool(std::string tool) {
    for (int i=0; i<storage_.size(); i++) {
        if (tool == storage_[i]->name()) {
            active_tool_idx_ = i;
            return jcs::RET_OK;
        }
    }
    return jcs::RET_ERROR;
}

void tool_manager::print_available_tools() {
    std::cout << "Tool manager - Available tools:\n";
    for (int i=0; i<storage_.size(); i++) {
        std::cout << storage_[i]->name() << "\n";
    }
}

int tool_manager::load_config(std::string tool_config) {
    return storage_[active_tool_idx_]->load_config(tool_config);
}

bool tool_manager::use_mem_lock() {
    return storage_[active_tool_idx_]->use_mem_lock();
}

int tool_manager::step_startup_rt() {
    return storage_[active_tool_idx_]->step_startup_rt();
}

int tool_manager::step_rt() {
    return storage_[active_tool_idx_]->step_rt();
}

void tool_manager::estop_rt() {
    storage_[active_tool_idx_]->estop_rt();
}

int tool_manager::step_shutdown_rt() {
    return storage_[active_tool_idx_]->step_shutdown_rt();
}

int tool_manager::step_parameter_startup() {
    return storage_[active_tool_idx_]->step_parameter_startup();
}

int tool_manager::step_parameter() {
    return storage_[active_tool_idx_]->step_parameter();
}

int tool_manager::step_parameter_shutdown() {
    return storage_[active_tool_idx_]->step_parameter_shutdown();
}