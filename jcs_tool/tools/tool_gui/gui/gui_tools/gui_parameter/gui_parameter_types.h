// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_PARAMETER_TYPES_H_
#define GUI_PARAMETER_TYPES_H_

#include "imgui.h"
#include <string>
#include "jcs_host.h"
#include <yaml-cpp/yaml.h>


///////////////////////////////////////////////////////////////////////////////////////////
class param_base {
public:
    param_base(jcs::jcs_host* host, std::string const& name, int length) :
        host_(host), name_(name), length_(length), watch_(false) {}
    ~param_base() {}

    virtual void render(std::string const& target_device) = 0;
    virtual int  read(std::string const& target_device) = 0;
    virtual void write_to_file(YAML::Emitter& yemit) = 0; 

    jcs::jcs_host* host_;
    std::string name_;
    int length_;
    bool watch_;
};

///////////////////////////////////////////////////////////////////////////////////////////
class param_none : public param_base {
public:
    param_none(jcs::jcs_host* host, std::string const& name, int length) :
        param_base(host, name, length) {}

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);
};

///////////////////////////////////////////////////////////////////////////////////////////
class param_boolean : public param_base {
public:
    param_boolean(jcs::jcs_host* host, std::string const& name, int length) :
        param_base(host, name, length), val_(false), write_select_(0) {}

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    bool val_;
    int write_select_;
};

///////////////////////////////////////////////////////////////////////////////////////////
class param_float32 : public param_base {
public:
    param_float32(jcs::jcs_host* host, std::string const& name, int length) :
        param_base(host, name, length), write_val_(0.0f), read_val_(0.0f) {}

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    float write_val_;
    float read_val_;
};
///////////////////////////////////////////////////////////////////////////////////////////
class param_float32_vec : public param_base {
public:
    param_float32_vec(jcs::jcs_host* host, std::string const& name, int length);

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    std::vector<float> write_val_;
    std::vector<float> read_val_;
};

///////////////////////////////////////////////////////////////////////////////////////////
class param_uint32 : public param_base {
public:
    param_uint32(jcs::jcs_host* host, std::string const& name, int length) :
        param_base(host, name, length), write_val_(0), read_val_(0) {}

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    uint32_t write_val_;
    uint32_t read_val_;
};
///////////////////////////////////////////////////////////////////////////////////////////
class param_uint32_vec : public param_base {
public:
    param_uint32_vec(jcs::jcs_host* host, std::string const& name, int length);

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    std::vector<uint32_t> write_val_;
    std::vector<uint32_t> read_val_;
};

///////////////////////////////////////////////////////////////////////////////////////////
class param_uint16 : public param_base {
public:
    param_uint16(jcs::jcs_host* host, std::string const& name, int length) :
        param_base(host, name, length), write_val_(0), read_val_(0) {}

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    uint16_t write_val_;
    uint16_t read_val_;
};
///////////////////////////////////////////////////////////////////////////////////////////
class param_uint16_vec : public param_base {
public:
    param_uint16_vec(jcs::jcs_host* host, std::string const& name, int length);

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    std::vector<uint16_t> write_val_;
    std::vector<uint16_t> read_val_;
};

///////////////////////////////////////////////////////////////////////////////////////////
class param_uint8 : public param_base {
public:
    param_uint8(jcs::jcs_host* host, std::string const& name, int length) :
        param_base(host, name, length), write_val_(0), read_val_(0) {}

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    uint8_t write_val_;
    uint8_t read_val_;
};
///////////////////////////////////////////////////////////////////////////////////////////
class param_uint8_vec : public param_base {
public:
    param_uint8_vec(jcs::jcs_host* host, std::string const& name, int length);

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    std::vector<uint8_t> write_val_;
    std::vector<uint8_t> read_val_;
};

///////////////////////////////////////////////////////////////////////////////////////////
class param_enum : public param_base {
public:
    param_enum(jcs::jcs_host* host, std::string const& name, int length, std::vector<std::string> const* enums) :
        param_base(host, name, length), enum_read_idx_(0), enums_(enums) {
            // Set write to first enum entry
            write_val_ = enums_->at(0);
        }

    void render(std::string const& target_device);
    int  read(std::string const& target_device);
    void write_to_file(YAML::Emitter& yemit);

private:
    void enum_get();

    std::vector<std::string> const* enums_;
    int enum_read_idx_;
    std::string write_val_;
    std::string read_val_;
};

#endif