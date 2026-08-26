// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef JCS_TOOL_IF_H_
#define JCS_TOOL_IF_H_

#include "jcs_host.h"
#include <stdint.h>
#include <string>

class jcs_tool_if {
public:
    jcs_tool_if(std::string name, jcs::jcs_host* host, bool use_mem_lock);
    ~jcs_tool_if();

    std::string const name();

    // Load any config. Called before initialising system
    virtual int load_config(std::string tool_config) = 0;

    bool use_mem_lock() { return use_mem_lock_; }

    // Realtime startup function:
    // Called within realtime thread, before entry into cyclic loop
    virtual int step_startup_rt() = 0;
    // Realtime tick function
    // Will only be called once data_is_valid_rt() is true
    // Return jcs::RET_OK if all is well
    // Return jcs::RET_NRDY to shutdown gracefully
    // Return jcs::RET_ERROR for any Estop or error conditions
    virtual int step_rt() = 0;
    // Realtime estop notification:
    // Called within the realtime thread when the host has latched an estop.
    // Default implementation does nothing.
    virtual void estop_rt() {}
    // Realtime shutdown function:
    // Called within realtime thread, after exiting the cyclic loop
    virtual int step_shutdown_rt() = 0;

    // Non realtime parameter startup function
    // Called once system is up and ticking cyclic_ready() becomes true
    // Use this to start the network and ready devices
    // Return jcs:RET_OK to transition to running mode
    // Return jcs::RET_ERROR / jcs::RET_NRDY to shutdown
    virtual int step_parameter_startup() = 0;
    // Non realtime parameter tick function
    // Return jcs:RET_OK if all is well
    // Return jcs::RET_ERROR / jcs::RET_NRDY to shutdown
    virtual int step_parameter() = 0;
    // Non realtime parameter shutdown function
    // Called upon request to shutdown the system.
    virtual int step_parameter_shutdown() = 0;

protected:
    std::string name_;
    jcs::jcs_host* host_;

    bool use_mem_lock_;
};

#endif
