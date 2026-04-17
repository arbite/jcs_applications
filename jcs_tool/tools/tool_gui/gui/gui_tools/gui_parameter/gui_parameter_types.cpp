// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_parameter_types.h"
#include <iostream>
#include <string>
#include "imgui.h"
#include "helpers.h"

///////////////////////////////////////////////////////////////////////////////////////////
void param_none::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    // Name
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());
    // Type
    ImGui::TableNextColumn();
    ImGui::Text("none/cmd");
    // Length - N/A
    ImGui::TableNextColumn();
    ImGui::Text("-");
    // Write - N/A
    ImGui::TableNextColumn();
    // Write
    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        std::string fail_text = "Parameter failed " + name_;
        PARAM_NOTIFY( host_->write_command(target_device, name_), fail_text )
    }
    // Read - N/A
    ImGui::TableNextColumn();
    // Watch - N/A
    ImGui::TableNextColumn();
    // Value
    ImGui::TableNextColumn();

    ImGui::PopID();
}
int param_none::read(std::string const& target_device) {
    return jcs::RET_OK;
}
void param_none::write_to_file(YAML::Emitter& yemit) {}

///////////////////////////////////////////////////////////////////////////////////////////
void param_boolean::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    // Name
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());
    // Type
    ImGui::TableNextColumn();
    ImGui::Text("boolean");
    // Length
    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);
    if (length_ == 0) {
        ImGui::BeginDisabled();
    }
    // Write - True
    std::string fail_text = "Parameter failed " + name_;
    ImGui::TableNextColumn();
    ImGui::PushItemWidth(-FLT_MIN); // Right aligned    
    ImGui::Combo("##True/False", &write_select_, "False\0True\0");
    // Write
    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        bool value = (bool)write_select_;
        PARAM_NOTIFY( host_->write_bool(target_device, name_, value), fail_text )
    }
    // Read
    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        bool value = false;
        PARAM_NOTIFY( host_->read_bool(target_device, name_, &value), fail_text )
        val_ = value;
    }
    // Watch
    ImGui::TableNextColumn();
    ImGui::Checkbox("##watch", &watch_);
    if (watch_) {
        bool value = false;
        if (host_->read_bool(target_device, name_, &value) != jcs::RET_OK) {
            watch_ = false;
        }
        val_ = value;
    }
    // Print Read
    ImGui::TableNextColumn();
    if (val_ == true)  { ImGui::Text("True"); }
    if (val_ == false) { ImGui::Text("False"); }

    if (length_ == 0) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}
int param_boolean::read(std::string const& target_device) {
    if (length_ == 0) {
        // Not supported
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    bool value = false;
    PARAM_NOTIFY_ERROR( host_->read_bool(target_device, name_, &value), fail_text )
    val_ = value;
    write_select_ = (int)value;
    return jcs::RET_OK;
}
void param_boolean::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key << name_ << YAML::Value << val_;
}

///////////////////////////////////////////////////////////////////////////////////////////
void param_float32::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());
    
    ImGui::TableNextColumn();
    ImGui::Text("float32");
    
    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);
    if (length_ == 0) {
        ImGui::BeginDisabled();
    }
    // Write
    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;

    ImGui::PushItemWidth(-FLT_MIN); // Right aligned
    {
        float value = write_val_;
        if (ImGui::InputFloat("##Write", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            write_val_ = value;
        }
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_float(target_device, name_, write_val_), fail_text )
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        float value = 0.0f;
        PARAM_NOTIFY( host_->read_float(target_device, name_, &value), fail_text )
        read_val_ = value;
    }
    // Watch
    ImGui::TableNextColumn();
    ImGui::Checkbox("##watch", &watch_);
    if (watch_) {
        float value = 0.0f;
        if (host_->read_float(target_device, name_, &value) != jcs::RET_OK) {
            watch_ = false;
        }
        read_val_ = value;
    }

    ImGui::TableNextColumn();
    ImGui::Text("%.6f", read_val_);
    if (length_ == 0) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}
int param_float32::read(std::string const& target_device) {
    if (length_ == 0) {
        // Not supported
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    float value = 0.0f;
    PARAM_NOTIFY_ERROR( host_->read_float(target_device, name_, &value), fail_text )
    read_val_ = value;
    write_val_ = value;
    return jcs::RET_OK;
}
void param_float32::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key << name_ << YAML::Value << (double)read_val_;
}

///////////////////////////////////////////////////////////////////////////////////////////
param_float32_vec::param_float32_vec(jcs::jcs_host* host, std::string const& name, int length) :
    param_base(host, name, length)
{
    write_val_.resize(length);
    read_val_.resize(length);
}
void param_float32_vec::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());

    ImGui::TableNextColumn();
    ImGui::Text("float32 vec");

    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);

    // Write
    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;

    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginPopupContextItem("float_input")) {
        char buf[64];
        ImGui::Text("Vector values:");
        for (int i=0; i<write_val_.size(); i++) {
            ImGui::PushID(i);
            sprintf(buf, "Element: %u", i);
            ImGui::DragFloat(buf, &write_val_[i], 0.01f, 0.0f, 0.0f);
            ImGui::PopID();
        }
        ImGui::EndPopup();
    }
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::Button("Set vector values")) {
        ImGui::OpenPopup("float_input");
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_float(target_device, name_, write_val_), fail_text )
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->read_float(target_device, name_, &read_val_), fail_text )
    }

    // Watch
    ImGui::TableNextColumn();
    // Only watch if small enough
    if (read_val_.size() <= 4) {
        ImGui::Checkbox("##watch", &watch_);
        if (watch_) {
            read(target_device);
        }
    }

    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
    // If the vector is small enough, display the elements, otherwise open a popup to display them
    switch (read_val_.size()) {
        case 1: ImGui::Text("%.6f", read_val_[0]); break;
        case 2: ImGui::Text("%.4f, %.4f", read_val_[0], read_val_[1]); break;
        case 3: ImGui::Text("%.4f, %.4f, %.4f", read_val_[0], read_val_[1], read_val_[2]); break;
        case 4: ImGui::Text("%.3f, %.3f, %.3f, %.3f", read_val_[0], read_val_[1], read_val_[2], read_val_[3]); break;
        default:
            if (ImGui::BeginPopupContextItem("float_output")) {
                ImGui::Text("Vector values:");
                for (int i=0; i<read_val_.size(); i++) {
                    ImGui::PushID(i);
                    ImGui::Text("Element: %u, %4.4f", i, read_val_[i]);
                    ImGui::PopID();
                }
                ImGui::EndPopup();
            }
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::Button("Read vector values")) {
                ImGui::OpenPopup("float_output");
            }
            break;
    }

    ImGui::PopID();
}
int param_float32_vec::read(std::string const& target_device) {
    if (length_ == 0) {
        // Not supported
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    PARAM_NOTIFY_ERROR( host_->read_float(target_device, name_, &read_val_), fail_text )
    for (int i=0; i<read_val_.size(); i++) { write_val_[i] = read_val_[i]; }
    return jcs::RET_OK;
}
void param_float32_vec::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }

    yemit << YAML::Key;
    yemit << name_;
    // Emit vector data
    yemit << YAML::Flow;
    yemit << read_val_;
}

///////////////////////////////////////////////////////////////////////////////////////////
void param_uint32::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());
    
    ImGui::TableNextColumn();
    ImGui::Text("uint32");
    
    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);
    if (length_ == 0) {
        ImGui::BeginDisabled();
    }

    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;
    ImGui::PushItemWidth(-FLT_MIN); // Right aligned
    {
        int value = (int)write_val_;
        if (ImGui::InputInt("##Write", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            write_val_ = (uint32_t)value;
        }
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_uint32(target_device, name_, write_val_), fail_text )
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        uint32_t value = 0;
        PARAM_NOTIFY( host_->read_uint32(target_device, name_, &value), fail_text )
        read_val_ = value;
    }
    // Watch
    ImGui::TableNextColumn();
    ImGui::Checkbox("##watch", &watch_);
    if (watch_) {
        uint32_t value = 0;
        if (host_->read_uint32(target_device, name_, &value) != jcs::RET_OK) {
            watch_ = false;
        }
        read_val_ = value;
    }

    ImGui::TableNextColumn();
    ImGui::Text("%u", read_val_);

    if (length_ == 0) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}
int param_uint32::read(std::string const& target_device) {
    if (length_ == 0) {
        // Not supported
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    uint32_t value = 0;
    PARAM_NOTIFY_ERROR( host_->read_uint32(target_device, name_, &value), fail_text )
    read_val_ = value;
    write_val_ = value;
    return jcs::RET_OK;
}
void param_uint32::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key << name_ << YAML::Value << (unsigned int)read_val_;
}

///////////////////////////////////////////////////////////////////////////////////////////
param_uint32_vec::param_uint32_vec(jcs::jcs_host* host, std::string const& name, int length) :
    param_base(host, name, length)
{
    write_val_.resize(length);
    read_val_.resize(length);
}
void param_uint32_vec::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());

    ImGui::TableNextColumn();
    ImGui::Text("uint32 vec");

    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);

    // Write
    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;

    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginPopupContextItem("uint32_input")) {
        char buf[64];
        ImGui::Text("Vector values:");
        for (int i=0; i<write_val_.size(); i++) {
            ImGui::PushID(i);
            sprintf(buf, "Element: %u", i);
            int tmp = (int)write_val_[i];
            if (ImGui::DragInt(buf, &tmp, 1.0f, 0, 0)) {
                write_val_[i] = (uint32_t)tmp;
            }
            ImGui::PopID();
        }
        ImGui::EndPopup();
    }
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::Button("Set vector values")) {
        ImGui::OpenPopup("uint32_input");
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_uint32(target_device, name_, write_val_), fail_text )
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->read_uint32(target_device, name_, &read_val_), fail_text )
    }

    // Watch
    ImGui::TableNextColumn();
    if (read_val_.size() <= 4) {
        ImGui::Checkbox("##watch", &watch_);
        if (watch_) {
            read(target_device);
        }
    }

    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
    switch (read_val_.size()) {
        case 1: ImGui::Text("%u", read_val_[0]); break;
        case 2: ImGui::Text("%u, %u", read_val_[0], read_val_[1]); break;
        case 3: ImGui::Text("%u, %u, %u", read_val_[0], read_val_[1], read_val_[2]); break;
        case 4: ImGui::Text("%u, %u, %u, %u", read_val_[0], read_val_[1], read_val_[2], read_val_[3]); break;
        default:
            if (ImGui::BeginPopupContextItem("uint32_output")) {
                ImGui::Text("Vector values:");
                for (int i=0; i<read_val_.size(); i++) {
                    ImGui::PushID(i);
                    ImGui::Text("Element: %u, %u", i, read_val_[i]);
                    ImGui::PopID();
                }
                ImGui::EndPopup();
            }
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::Button("Read vector values")) {
                ImGui::OpenPopup("uint32_output");
            }
            break;
    }

    ImGui::PopID();
}
int param_uint32_vec::read(std::string const& target_device) {
    if (length_ == 0) {
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    PARAM_NOTIFY_ERROR( host_->read_uint32(target_device, name_, &read_val_), fail_text )
    for (int i=0; i<read_val_.size(); i++) { write_val_[i] = read_val_[i]; }
    return jcs::RET_OK;
}
void param_uint32_vec::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key;
    yemit << name_;
    yemit << YAML::Flow;
    yemit << YAML::BeginSeq;
    for (int i=0; i<read_val_.size(); i++) { yemit << (unsigned int)read_val_[i]; }
    yemit << YAML::EndSeq;
}

///////////////////////////////////////////////////////////////////////////////////////////
void param_uint16::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());
    
    ImGui::TableNextColumn();
    ImGui::Text("uint16");
    
    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);
    if (length_ == 0) {
        ImGui::BeginDisabled();
    }

    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;
    ImGui::PushItemWidth(-FLT_MIN); // Right aligned
    {
        int value = (int)write_val_;
        if (ImGui::InputInt("##Write", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            write_val_ = (uint16_t)value;
        }
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_uint16(target_device, name_, write_val_), fail_text )
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        uint16_t value = 0;
        PARAM_NOTIFY( host_->read_uint16(target_device, name_, &value), fail_text )
        read_val_ = value;
    }
    // Watch
    ImGui::TableNextColumn();
    ImGui::Checkbox("##watch", &watch_);
    if (watch_) {
        uint16_t value = 0;
        if (host_->read_uint16(target_device, name_, &value) != jcs::RET_OK) {
            watch_ = false;
        }
        read_val_ = value;
    }

    ImGui::TableNextColumn();
    ImGui::Text("%u", read_val_);

    if (length_ == 0) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}
int param_uint16::read(std::string const& target_device) {
    if (length_ == 0) {
        // Not supported
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    uint16_t value = 0;
    PARAM_NOTIFY_ERROR( host_->read_uint16(target_device, name_, &value), fail_text )
    read_val_ = value;
    write_val_ = value;
    return jcs::RET_OK;
}
void param_uint16::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key << name_ << YAML::Value << (unsigned int)read_val_;
}

///////////////////////////////////////////////////////////////////////////////////////////
param_uint16_vec::param_uint16_vec(jcs::jcs_host* host, std::string const& name, int length) :
    param_base(host, name, length)
{
    write_val_.resize(length);
    read_val_.resize(length);
}
void param_uint16_vec::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());

    ImGui::TableNextColumn();
    ImGui::Text("uint16 vec");

    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);

    // Write
    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;

    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginPopupContextItem("uint16_input")) {
        char buf[64];
        ImGui::Text("Vector values:");
        for (int i=0; i<write_val_.size(); i++) {
            ImGui::PushID(i);
            sprintf(buf, "Element: %u", i);
            int tmp = (int)write_val_[i];
            if (ImGui::DragInt(buf, &tmp, 1.0f, 0, 65535)) {
                write_val_[i] = (uint16_t)tmp;
            }
            ImGui::PopID();
        }
        ImGui::EndPopup();
    }
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::Button("Set vector values")) {
        ImGui::OpenPopup("uint16_input");
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_uint16(target_device, name_, write_val_), fail_text )
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->read_uint16(target_device, name_, &read_val_), fail_text )
    }

    // Watch
    ImGui::TableNextColumn();
    if (read_val_.size() <= 4) {
        ImGui::Checkbox("##watch", &watch_);
        if (watch_) {
            read(target_device);
        }
    }

    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
    switch (read_val_.size()) {
        case 1: ImGui::Text("%u", (unsigned int)read_val_[0]); break;
        case 2: ImGui::Text("%u, %u", (unsigned int)read_val_[0], (unsigned int)read_val_[1]); break;
        case 3: ImGui::Text("%u, %u, %u", (unsigned int)read_val_[0], (unsigned int)read_val_[1], (unsigned int)read_val_[2]); break;
        case 4: ImGui::Text("%u, %u, %u, %u", (unsigned int)read_val_[0], (unsigned int)read_val_[1], (unsigned int)read_val_[2], (unsigned int)read_val_[3]); break;
        default:
            if (ImGui::BeginPopupContextItem("uint16_output")) {
                ImGui::Text("Vector values:");
                for (int i=0; i<read_val_.size(); i++) {
                    ImGui::PushID(i);
                    ImGui::Text("Element: %u, %u", i, (unsigned int)read_val_[i]);
                    ImGui::PopID();
                }
                ImGui::EndPopup();
            }
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::Button("Read vector values")) {
                ImGui::OpenPopup("uint16_output");
            }
            break;
    }

    ImGui::PopID();
}
int param_uint16_vec::read(std::string const& target_device) {
    if (length_ == 0) {
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    PARAM_NOTIFY_ERROR( host_->read_uint16(target_device, name_, &read_val_), fail_text )
    for (int i=0; i<read_val_.size(); i++) { write_val_[i] = read_val_[i]; }
    return jcs::RET_OK;
}
void param_uint16_vec::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key;
    yemit << name_;
    yemit << YAML::Flow;
    yemit << YAML::BeginSeq;
    for (int i=0; i<read_val_.size(); i++) { yemit << (unsigned int)read_val_[i]; }
    yemit << YAML::EndSeq;
}

///////////////////////////////////////////////////////////////////////////////////////////
void param_uint8::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());
    
    ImGui::TableNextColumn();
    ImGui::Text("uint8");
    
    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);
    if (length_ == 0) {
        ImGui::BeginDisabled();
    }

    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;
    ImGui::PushItemWidth(-FLT_MIN); // Right aligned
    {
        int value = (int)write_val_;
        if (ImGui::InputInt("##Write", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            write_val_ = (uint8_t)value;
        }
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_uint8(target_device, name_, write_val_), fail_text )
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        uint8_t value = 0;
        PARAM_NOTIFY( host_->read_uint8(target_device, name_, &value), fail_text )
        read_val_ = value;
    }
    // Watch
    ImGui::TableNextColumn();
    ImGui::Checkbox("##watch", &watch_);
    if (watch_) {
        uint8_t value = 0;
        if (host_->read_uint8(target_device, name_, &value) != jcs::RET_OK) {
            watch_ = false;
        }
        read_val_ = value;
    }

    ImGui::TableNextColumn();
    ImGui::Text("%u", read_val_);

    if (length_ == 0) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}
int param_uint8::read(std::string const& target_device) {
    if (length_ == 0) {
        // Not supported
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    uint8_t value = 0;
    PARAM_NOTIFY_ERROR( host_->read_uint8(target_device, name_, &value), fail_text )
    read_val_ = value;
    write_val_ = value;
    return jcs::RET_OK;
}
void param_uint8::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key << name_ << YAML::Value << (unsigned int)read_val_;
}

///////////////////////////////////////////////////////////////////////////////////////////
param_uint8_vec::param_uint8_vec(jcs::jcs_host* host, std::string const& name, int length) :
    param_base(host, name, length)
{
    write_val_.resize(length);
    read_val_.resize(length);
}
void param_uint8_vec::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());

    ImGui::TableNextColumn();
    ImGui::Text("uint8 vec");

    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);

    // Write
    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;

    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginPopupContextItem("uint8_input")) {
        char buf[64];
        ImGui::Text("Vector values:");
        for (int i=0; i<write_val_.size(); i++) {
            ImGui::PushID(i);
            sprintf(buf, "Element: %u", i);
            int tmp = (int)write_val_[i];
            if (ImGui::DragInt(buf, &tmp, 1.0f, 0, 255)) {
                write_val_[i] = (uint8_t)tmp;
            }
            ImGui::PopID();
        }
        ImGui::EndPopup();
    }
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::Button("Set vector values")) {
        ImGui::OpenPopup("uint8_input");
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_uint8(target_device, name_, write_val_), fail_text )
    }

    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->read_uint8(target_device, name_, &read_val_), fail_text )
    }

    // Watch
    ImGui::TableNextColumn();
    if (read_val_.size() <= 4) {
        ImGui::Checkbox("##watch", &watch_);
        if (watch_) {
            read(target_device);
        }
    }

    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
    switch (read_val_.size()) {
        case 1: ImGui::Text("%u", (unsigned int)read_val_[0]); break;
        case 2: ImGui::Text("%u, %u", (unsigned int)read_val_[0], (unsigned int)read_val_[1]); break;
        case 3: ImGui::Text("%u, %u, %u", (unsigned int)read_val_[0], (unsigned int)read_val_[1], (unsigned int)read_val_[2]); break;
        case 4: ImGui::Text("%u, %u, %u, %u", (unsigned int)read_val_[0], (unsigned int)read_val_[1], (unsigned int)read_val_[2], (unsigned int)read_val_[3]); break;
        default:
            if (ImGui::BeginPopupContextItem("uint8_output")) {
                ImGui::Text("Vector values:");
                for (int i=0; i<read_val_.size(); i++) {
                    ImGui::PushID(i);
                    ImGui::Text("Element: %u, %u", i, (unsigned int)read_val_[i]);
                    ImGui::PopID();
                }
                ImGui::EndPopup();
            }
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::Button("Read vector values")) {
                ImGui::OpenPopup("uint8_output");
            }
            break;
    }

    ImGui::PopID();
}
int param_uint8_vec::read(std::string const& target_device) {
    if (length_ == 0) {
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    PARAM_NOTIFY_ERROR( host_->read_uint8(target_device, name_, &read_val_), fail_text )
    for (int i=0; i<read_val_.size(); i++) { write_val_[i] = read_val_[i]; }
    return jcs::RET_OK;
}
void param_uint8_vec::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key;
    yemit << name_;
    yemit << YAML::Flow;
    yemit << YAML::BeginSeq;
    for (int i=0; i<read_val_.size(); i++) { yemit << (unsigned int)read_val_[i]; }
    yemit << YAML::EndSeq;
}

///////////////////////////////////////////////////////////////////////////////////////////
void param_enum::render(std::string const& target_device) {
    ImGui::PushID((target_device + name_).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name_.c_str());
    
    ImGui::TableNextColumn();
    ImGui::Text("enum8");
    
    ImGui::TableNextColumn();
    ImGui::Text("%u", length_);

    if (length_ == 0) {
        ImGui::BeginDisabled();
    }

    ImGui::TableNextColumn();
    std::string fail_text = "Parameter failed " + name_;
    ImGui::PushItemWidth(-FLT_MIN); // Right aligned
    enum_get();
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Write", ImVec2(-FLT_MIN, 0.0f))) {
        PARAM_NOTIFY( host_->write_enum(target_device, name_, write_val_), fail_text )
    }
    
    ImGui::TableNextColumn();
    if (ImGui::Button("Read", ImVec2(-FLT_MIN, 0.0f))) {
        std::string value;
        PARAM_NOTIFY( host_->read_enum(target_device, name_, &value), fail_text )
        read_val_ = value;
    }
    // Watch
    ImGui::TableNextColumn();
    ImGui::Checkbox("##watch", &watch_);
    if (watch_) {
        std::string value;
        if (host_->read_enum(target_device, name_, &value) != jcs::RET_OK) {
            watch_ = false;
        }
        read_val_ = value;
    }

    ImGui::TableNextColumn();
    ImGui::Text("%s", read_val_.c_str());

    if (length_ == 0) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}
int param_enum::read(std::string const& target_device) {
    if (length_ == 0) {
        // Not supported
        return jcs::RET_OK;
    }
    std::string fail_text = "Parameter failed " + name_;
    std::string value;
    PARAM_NOTIFY_ERROR( host_->read_enum(target_device, name_, &value), fail_text )
    read_val_ = value;
    write_val_ = value;
    return jcs::RET_OK;
}
void param_enum::enum_get() {
    const char* combo_preview_value = enums_->at(enum_read_idx_).c_str();

    ImGui::PushID("##Write");
    if (ImGui::BeginCombo("##Write", combo_preview_value, 0)) {
        for (int i=0; i<enums_->size(); ++i) {
            const bool is_selected = (enum_read_idx_ == i);
            if (ImGui::Selectable(enums_->at(i).c_str(), is_selected)) {
                enum_read_idx_ = i;
            }
            // Set the initial focus when opening the combo
            // (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        write_val_ = enums_->at(enum_read_idx_);
        // std::cout << "Got " << name << " " << *dest << "\n";
        ImGui::EndCombo();
    }
    ImGui::PopID();
}
void param_enum::write_to_file(YAML::Emitter& yemit) {
    if (length_ == 0) { return; }
    yemit << YAML::Key << name_ << YAML::Value << read_val_;
}