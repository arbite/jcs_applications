// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include <cstdio>
#include <ctime>
#include "jcs_user_external.h"
#include "task_rt.h"

namespace jcs {
namespace external {

void sleep_us(long int us) {
    task_rt::sleep_us(us);
}

static const long int nsec_per_sec = 1000000000;

long int time_now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * nsec_per_sec + ts.tv_nsec;
}

// Default implementation: prepend level prefix and print to stdout.
// Replace this to route JCS output to your logging system.
void print_output(log_level level, bool prefix, const char* msg) {
    if (prefix) {
        const char* pfx = "";
        switch (level) {
            case log_level::info:    pfx = "INFO   : "; break;
            case log_level::debug:   pfx = "DEBUG  : "; break;
            case log_level::warning: pfx = "WARNING: "; break;
            case log_level::error:   pfx = "ERROR! : "; break;
            default:                 pfx = "UNHANDLED!: "; break;
        }
        printf("%s%s", pfx, msg);
    } else {
        printf("%s", msg);
    }
}

} // End namespace external
} // End namespace jcs