// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef JCS_TOOL_MANAGER_H_
#define JCS_TOOL_MANAGER_H_

#include "jcs_tool_if.h"
#include "jcs_host.h"
#include <vector>
#include <string>

class tool_manager {
public:
    tool_manager(jcs::jcs_host* host);
    ~tool_manager();

    int set_active_tool(std::string tool);
    void print_available_tools();

    int load_config(std::string tool_config);
    bool use_mem_lock();

    int step_startup_rt();
    int step_rt();
    void estop_rt();
    int step_shutdown_rt();
    int step_parameter_startup();
    int step_parameter();
    int step_parameter_shutdown();

protected:
    jcs::jcs_host* host_;

private:
    std::vector<jcs_tool_if*> storage_;
    int active_tool_idx_;

};

#endif