// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_parameter.h"
#include <iostream>
#include "gui_parameter_types.h"
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "ImGuiFileDialog.h"

//////////////////////////////////////////////////////////////////////
// Parameter read all helpers
void gui_parameter::parameter_do_all::tick(std::vector<param_base*>* param_store, std::string const& target_device) {

    switch (state_) {
        case state::stopped:
            // Read all params?
            if (ImGui::Button("Read All Parameters")) {
                param_index_ = 0;
                increment_ = 1.0f / (float)param_store->size();
                fraction_ = 0.0f;
                state_ = state::running;
            }
            break;

        case state::running:
            if (ImGui::Button("Cancel")) {
                cancel();
                break;
            }
            if (param_store->at(param_index_)->read(target_device) != jcs::RET_OK) {
                cancel();
                break;
            }
            param_index_++;
            if (param_index_ >= param_store->size()) {
                cancel();
                break;
            }
            fraction_ += increment_;
            ImGui::SameLine();
            ImGui::ProgressBar(fraction_, ImVec2(0.0f, 0.0f));
            break;
    }
}

//////////////////////////////////////////////////////////////////////
gui_parameter::gui_parameter(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device, 
        std::vector<jcs::parameter> const* params, std::vector<jcs::parameter_enum> const* enums) :
    gui_type_base("Parameter", host, gui_if, target_device), 
    params_(params), enums_(enums), do_all_()
{
    // Note: Cant do anything reliant on host having been initialised here.
    // Host not initialised until after. 
}

int gui_parameter::startup() {
    // Build the parameters
    for (int i=0; i<params_->size(); i++) {
        jcs::parameter const* p = &params_->at(i);
        switch(p->type) {
            default: break;
            case jcs::parameter_type::p_none_t:    param_store_.push_back(new param_none(host_, p->name, p->length));    break;
            case jcs::parameter_type::p_bool_t:    param_store_.push_back(new param_boolean(host_, p->name, p->length)); break;

            case jcs::parameter_type::p_float32_t:
                if (p->length <= 1) {
                    param_store_.push_back(new param_float32(host_, p->name, p->length));
                } else {
                    param_store_.push_back(new param_float32_vec(host_, p->name, p->length));
                }
                break;

            case jcs::parameter_type::p_uint32_t:
                if (p->length <= 1) {
                    param_store_.push_back(new param_uint32(host_, p->name, p->length));
                } else {
                    param_store_.push_back(new param_uint32_vec(host_, p->name, p->length));
                }
                break;

            case jcs::parameter_type::p_uint16_t:
                if (p->length <= 1) {
                    param_store_.push_back(new param_uint16(host_, p->name, p->length));
                } else {
                    param_store_.push_back(new param_uint16_vec(host_, p->name, p->length));
                }
                break;

            case jcs::parameter_type::p_uint8_t:
                if (p->length <= 1) {
                    param_store_.push_back(new param_uint8(host_, p->name, p->length));
                } else {
                    param_store_.push_back(new param_uint8_vec(host_, p->name, p->length));
                }
                break;

            // Enum type gets a pointer to structure of enum strings for validation and naming
            case jcs::parameter_type::p_enum8_t:
                {
                    bool found_enum = false;
                    for (int e=0; e<enums_->size(); e++) {
                        if (p->name == enums_->at(e).name) {
                            param_store_.push_back(new param_enum(host_, p->name, p->length, &enums_->at(e).enums));
                            found_enum = true;
                            break;// break from loop.....
                        }
                    }
                    if (found_enum) {
                        break;
                    }
                }
                std::cout << "gui_parameter: Could not map enum for " << p->name << "\n";
                return jcs::RET_ERROR;
        }
    }
    return jcs::RET_OK;
}

int gui_parameter::step_rt() {
    return jcs::RET_OK;
}

int gui_parameter::render() {

    do_all_.tick(&param_store_, target_device_);
    ImGui::SameLine();
    write_config_to_file();

    // Render table of parameters based on selected device
    static ImGuiTableFlags table_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable;
    if (ImGui::BeginTable("Parameters", 8, table_flags)) {

        ImGui::TableSetupColumn("Name",      ImGuiTableColumnFlags_WidthFixed, 300.0f);
        ImGui::TableSetupColumn("Type",      ImGuiTableColumnFlags_WidthFixed, 50.0f);
        ImGui::TableSetupColumn("Len",       ImGuiTableColumnFlags_WidthFixed, 25.0f);
        ImGui::TableSetupColumn("Write Val", ImGuiTableColumnFlags_WidthFixed, 180.0f);
        ImGui::TableSetupColumn("Write",     ImGuiTableColumnFlags_WidthFixed, 50.0f);
        ImGui::TableSetupColumn("Read",      ImGuiTableColumnFlags_WidthFixed, 50.0f);
        ImGui::TableSetupColumn("Watch",     ImGuiTableColumnFlags_WidthFixed, 30.0f);
        ImGui::TableSetupColumn("Read Val",  ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        for (int i=0; i<param_store_.size(); i++) {            
            param_store_[i]->render(target_device_);
        }

        ImGui::EndTable();
    }

    ImGui::Text("Calibration Parameters");
    ImGui::Text("Warning: Starting calibration will erase ALL caibration values");
    ImGui::Text("Warning: All calibration values must be written in the open session");
    ImGui::Text("- Unlock three times to unlock calibration");
    ImGui::Text("- Start to erase the calibration values and begin the calibration session");
    ImGui::Text("- Finish to end calibration session and write values to internal flash");

    if (ImGui::Button("Unlock")) {
        host_->write_command(target_device_, "calib_flash_unlock");
    }
    ImGui::SameLine();
    if (ImGui::Button("Start")) {
        host_->write_command(target_device_, "calib_flash_start");
    }
    ImGui::SameLine();
    if (ImGui::Button("Finish")) {
        host_->write_command(target_device_, "calib_flash_finish");
    }

    return jcs::RET_OK;
}

int gui_parameter::write_config_to_file() {

    // Choose file to write to
    if (ImGui::Button("Write config to file")) {
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("choose_dir_key", "Choose Directory", nullptr, config);
    }
    // display
    if (ImGuiFileDialog::Instance()->Display("choose_dir_key"))  {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            if (emit_config(ImGuiFileDialog::Instance()->GetCurrentPath()) != jcs::RET_OK) {
                return jcs::RET_ERROR;
            }

        }
        ImGuiFileDialog::Instance()->Close();
    }
    return jcs::RET_OK;
}

int gui_parameter::emit_config(std::string const& file_path) {

    jcs::jcs_device*  potential_device = NULL;
    jcs::jcs_process* potential_process = NULL;

    // Search the device tree for the device/proc
    std::vector<jcs::jcs_device>* device_tree = host_->external_info_tree_get();
    bool found = false;
    for (int i=0; i<device_tree->size(); i++) {
        if (device_tree->at(i).name == target_device_) {
            potential_device = &device_tree->at(i);
            found = true;
            break;
        }
        // Check processes
        for (int p=0; p<device_tree->at(i).procs.size(); p++) {
            if (device_tree->at(i).procs[p].name == target_device_) {
                potential_process = &device_tree->at(i).procs[p];
                found = true;
                break;
            }
        }
        if (found == true) {
            break;
        }
    }

    if (!found) {
        std::cout << "gui_parameter: Cannot find device or process name " << target_device_ << "\n";
        return jcs::RET_ERROR;
    }

    // Write the config file
    YAML::Emitter yemit;

    yemit.SetFloatPrecision(6);
    yemit.SetDoublePrecision(6);

    yemit << YAML::BeginMap;

    yemit << YAML::Comment("Generated config file.\nPlease review before use!");
    yemit << YAML::Newline;
    yemit << YAML::Newline;

    // Write the name and ID
    yemit << YAML::Key << "name" << YAML::Value << target_device_;
    if (potential_device != NULL) {
        yemit << YAML::Key << "device_id";
        yemit << YAML::Flow;
        yemit << YAML::BeginSeq;
        yemit << YAML::Value << potential_device->id.id[0] << potential_device->id.id[1] << potential_device->id.id[2];
        yemit << YAML::EndSeq;
    }

    yemit << YAML::Newline;
    yemit << YAML::Newline;

    // Write the parameters
    yemit << YAML::Key << "parameters";
    yemit << YAML::BeginMap;
    for (int i=0; i<param_store_.size(); i++) {            
        param_store_[i]->write_to_file(yemit);
    }
    yemit << YAML::EndMap;

    yemit << YAML::EndMap;

    // Write to file
    std::string file_name = "";

    if (potential_device != NULL) {
        file_name = "dev_";
    } else if (potential_process != NULL) {
        file_name = "proc_";
    }

    std::string full_file_name = file_path + "/" + file_name + target_device_ + "_generated.yaml";

    std::cout << "Writing to: " << full_file_name << "\n";

    std::ofstream config_file(full_file_name); 
    config_file << yemit.c_str();

    return jcs::RET_OK;
}