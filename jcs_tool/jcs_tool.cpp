//
// Tool framework
//
#include <iostream>
#include "jcs_host.h"
#include "tool_manager.h"
#include "jcs_user_external.h"
#include "task_rt.h"
#include "cmd_input_parser.h"

using namespace jcs;

void* thread_host_rt(void* arg);
void* thread_host_param(void* arg);

enum class tool_st {
    startup_s,
    running_s,
    shutdown_s
};

struct thread_host_args {
    jcs_host*   host;
    tool_st     run_state;
    bool        do_running;
    bool        debug_enabled;

    thread_host_args() : host(nullptr), 
                         run_state(tool_st::startup_s), 
                         do_running(false), 
                         debug_enabled(false) {}
};

static task_rt::thd_context thread_host;
static thread_host_args host_args;

static tool_manager* tools = NULL;

int main(int argc, char* argv[]) {

    cmd_input_parser cmd_parser(argc, argv);

    // We always need a config path
    std::string config_path = cmd_parser.cmd_option_get("-p");
    if (!config_path.empty()) {
        std::cout << "Read path: " << config_path << std::endl;
    } else {
        std::cout << "ERROR: Need config path with -p\n";
        return -1;
    }

    // Debug messages?
    bool print_debug = false;
    if (cmd_parser.cmd_option_exists("-d")) {
        std::cout << "Read option -d: Printing debug messages\n";
        print_debug = true;
        host_args.debug_enabled = true;
    }

    // RT debug messages?
    bool print_rt_debug = false;
    if (cmd_parser.cmd_option_exists("-drt")) {
        std::cout << "Read option -drt: Printing RT debug messages\n";
        print_rt_debug = true;
        host_args.debug_enabled = true;
    }

    // Start JCS host
    jcs_host host(config_path, print_debug, print_rt_debug);
    
    tools = new tool_manager(&host);
    // Get the tool to use
    std::string active_tool = cmd_parser.cmd_option_get("-t");
    if (!active_tool.empty()) {
        std::cout << "Using tool: " << active_tool << std::endl;
    } else {
        std::cout << "WARNING: Using default tool `tool_gui`. Select a tool to use with -t.\n";
        tools->print_available_tools();
        active_tool = "tool_gui";
    }
    // Set the active tool
    if (tools->set_active_tool(active_tool) != RET_OK) {
        std::cout << "ERROR: Failed to set active tool: " << active_tool << std::endl;
        return -1;
    }

    std::string tool_config = cmd_parser.cmd_option_get("-tc");
    if (tools->load_config(tool_config) != RET_OK) {
        std::cout << "ERROR: Failed to load config for tool: " << active_tool << std::endl;
        return -1;
    }
    
    // Initialise network and devices
    if (host.initialise() != RET_OK) {
        std::cout << "host: Initialising failed\n";
        return -1;
    }

    // Attach the host
    host_args.host = &host;
    // Attach threads and parameters
    thread_host.rt_thread_fn = &thread_host_rt;
    thread_host.thread_fn    = &thread_host_param;
    thread_host.thread_args  = reinterpret_cast<void*>(&host_args);

    host_args.do_running = true;


    // Start non realtime thread
    if (task_rt::task_start(&thread_host) != RET_OK) {
        std::cout << "task_start failed\n";
        return -1;        
    }

    // Start the rt cyclic thread
    if (task_rt::task_start_rt(&thread_host, tools->use_mem_lock()) != RET_OK) {
        std::cout << "task_start_rt failed\n";
        return -1;        
    }

    // Wait for tasks to exit
    task_rt::task_wait_rt(&thread_host);
    task_rt::task_wait(&thread_host);
    return 0;
}

void* thread_host_rt(void* arg) {
    
    thread_host_args* host_args = (thread_host_args*)arg;
    jcs_host* host = host_args->host;

    if (tools->step_startup_rt() != jcs::RET_OK) {
        host_args->do_running = false;
        std::cout<< "Error: step_startup_rt\n";
        return 0;
    }

    // Prepare cycle time. 
    // Noting JCS host recalculates cycle_time_ns each tick
    // to account for Ethercat DC clocks
    // First guess of cycle_time_ns from JCS host:
    int64_t cycle_time_ns = (int64_t)1e9 / (int64_t)host->base_frequency_get();
    task_rt::ready_cycle_rt(&thread_host, cycle_time_ns);

    while (host_args->do_running) {
        // Step JCS cyclic
        if (host->step_rt(&cycle_time_ns) != RET_OK) {
            std::cout << "Error: step_rt\n";
            host_args->do_running = false;
            break;
        }
        // Estop?
        if (host->estop_latched_rt()) {
            host->estop_ack_rt();
            tools->estop_rt();
        }

        if (host->data_is_valid_rt()) {
            // Step cyclic tools
            switch (tools->step_rt()) {
                default:
                case jcs::RET_ERROR:
                    host->trigger_estop();
                    host_args->run_state = tool_st::shutdown_s;
                    break;

                case jcs::RET_NRDY:
                    // Shutdown gracefully
                    host_args->run_state = tool_st::shutdown_s;
                    break;

                case jcs::RET_OK:
                    break;
            }
        }
        task_rt::wait_next_cycle_rt(&thread_host, cycle_time_ns);
    }

    tools->step_shutdown_rt();

    return 0;
}

void* thread_host_param(void* arg) {

    thread_host_args* host_args = (thread_host_args*)arg;
    jcs_host* host = host_args->host;

    // Wait for host_rt cyclic to become ready
    while (!host->cyclic_ready()) {
        if (host_args->do_running == false) {
            // Error while waiting for startup
            return 0;
        }
        // Todo: Timeout.....
        jcs::external::sleep_us(1e6);
    }

    host_args->run_state = tool_st::startup_s;

    while (host_args->do_running) {
        switch (host_args->run_state) {
            case tool_st::startup_s:
                if (tools->step_parameter_startup() != jcs::RET_OK) {
                    host_args->run_state = tool_st::shutdown_s;
                } else {
                    host_args->run_state = tool_st::running_s;
                }
                break;

            case tool_st::running_s:
                if (tools->step_parameter() != jcs::RET_OK) {
                    host_args->run_state = tool_st::shutdown_s;
                }
                break;
            
            default:
            case tool_st::shutdown_s:
                tools->step_parameter_shutdown();
                // Print some debug info
                if (host_args->debug_enabled == true) {
                    host->host_overrun_counts_print();
                }
                // All done
                host_args->do_running = false;
                break;
        }
    }
    

    return 0;
}