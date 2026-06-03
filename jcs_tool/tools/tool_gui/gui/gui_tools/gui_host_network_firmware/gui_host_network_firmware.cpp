
// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_network_firmware.h"
#include "helpers.h"
#include <iostream>

#include "ImGuiFileDialog.h"

gui_host_network_firmware_update::gui_host_network_firmware_update(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) : 
    gui_type_base("Network Firmware", host, gui_if, target_device)
{
    fw_write_active_ = false;
    fw_status_ = status::standby_s;
    fw_selected_ = 0;
    fw_only_write_listed_ = false;

    fl_write_active_ = false;
    fl_status_ = status::standby_s;
    fl_selected_ = 0;
    fl_only_write_listed_ = false;
}

int gui_host_network_firmware_update::startup() {
    return jcs::RET_OK;
}

int gui_host_network_firmware_update::step_rt() {
    return jcs::RET_OK;
}

int gui_host_network_firmware_update::step_rt_always() {
    return jcs::RET_OK;
}

int gui_host_network_firmware_update::render() {

    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
    if (ImGui::BeginTabBar("SELECT", tab_bar_flags)) {
        if (ImGui::BeginTabItem("Firmware")) {
            fw_update_render();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Flashloader")) {
            fl_update_render();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    return jcs::RET_OK;
}

void gui_host_network_firmware_update::fw_update_render() {
    ImGui::Text("Whole network firmware update");
    ImGui::Separator();
    ImGui::Text("Notes:");
    ImGui::Text("- This tool updates the firmware for devices connected to the JCS network");
    ImGui::Text("- Ensure 'base_freq_hz' is 250Hz");
    ImGui::Text("- This may take some time to complete");
    ImGui::Text("- Do NOT power off until complete!");
    ImGui::Text("- GUI will be unresponsive until firmware updates are complete");
    ImGui::Separator();
    ImGui::Text("Select firmware files for device types.");
    ImGui::Separator();

    ImGui::PushID((target_device_ + "FW").c_str());

    ImGuiFileDialog fileDialog;

    IGFD::FileDialogConfig config;
    config.flags =  ImGuiFileDialogFlags_NoDialog |
                    ImGuiFileDialogFlags_DisableCreateDirectoryButton |
                    ImGuiFileDialogFlags_DisableThumbnailMode |
                    ImGuiFileDialogFlags_DisableBookmarkMode |
                    ImGuiFileDialogFlags_DisableQuickPathSelection |
                    ImGuiFileDialogFlags_ReadOnlyFileNameField;
    config.path = ".";
    ImGuiFileDialog::Instance()->OpenDialog("embedded", "Choose File", ".bin", config);

    // display
    if (ImGuiFileDialog::Instance()->Display("embedded", ImGuiWindowFlags_NoCollapse, ImVec2(0,0), ImVec2(0,350))) {
        // action if OK
        if (ImGuiFileDialog::Instance()->IsOk()) {
            fw_status_ = status::standby_s;
            // Only add if not already in the list
            std::string filepath = ImGuiFileDialog::Instance()->GetFilePathName();
            if (std::find(fw_names_.begin(), fw_names_.end(), filepath) == fw_names_.end()) {
                fw_names_.push_back(filepath);
            }
        }
    }
    ImGui::PopID();

    ImGui::Separator();
    ImGui::Text("Device firmware list.");
    ImGui::Text("Ensure firmwares are selected for all device types to be updated");
    ImGui::Text("Any device that matches the firmware in this list will be updated");
    // List box of selected firmwares
    helpers::listbox_select("Device firmwares", &fw_names_, 5, &fw_selected_, NULL);
    // Remove any if not needed
    if (ImGui::Button("Remove selected firmware")) {
        if (fw_selected_ <= fw_names_.size()) {
            fw_names_.erase(fw_names_.begin() + fw_selected_);
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Validate firmware list - see logs")) {
        host_->validate_network_firmware_filenames(fw_names_);
    }

    ImGui::Separator();
    ImGui::Text("Select to update devices with firmware that match the list.");
    ImGui::Text("If NOT selected, all devices must have matching firmware.");
    ImGui::Checkbox("Only write devices with matching firmware", &fw_only_write_listed_);

    ImGui::Separator();
    ImGui::Checkbox("Activate", &fw_write_active_);

    if (fw_write_active_) {
        if (fw_only_write_listed_) {
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "WARNING!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Only write devices with matching firmware is SET.");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "All devices might not be written.");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Double check this is what you want!");
        }
        if (ImGui::Button("Write Network Firmware")) {
            if (host_->write_new_network_firmware(fw_names_, fw_only_write_listed_) == jcs::RET_OK) {
                fw_status_ = status::success_s;
            } else {
                fw_status_ = status::failed_s;
            }
            fw_write_active_ = false;
        }
    }
    switch (fw_status_) {
        default:
        case status::standby_s:
            break;
        case status::success_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Firmware write successful!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "WARNING!");
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Devices will not respond correctly until power cycle!");
            break;
        case status::failed_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "ERROR!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Firmware write unsuccessful. See jcs_host logs.");
            break;
    }
}

void gui_host_network_firmware_update::fl_update_render() {
    ImGui::Text("Whole network flashloader update");
    ImGui::Separator();
    ImGui::Text("Notes:");
    ImGui::Text("- This tool updates the flashloader for devices connected to the JCS network");
    ImGui::Text("- Ensure 'base_freq_hz' is 250Hz");
    ImGui::Text("- This may take some time to complete");
    ImGui::Text("- Do NOT power off until complete!");
    ImGui::Text("- GUI will be unresponsive until firmware updates are complete");
    ImGui::Separator();
    ImGui::Text("Select flashloader files for all device types");
    ImGui::Separator();

    ImGui::PushID((target_device_ + "FL").c_str());

    ImGuiFileDialog fileDialog;

    IGFD::FileDialogConfig config;
    config.flags =  ImGuiFileDialogFlags_NoDialog |
                    ImGuiFileDialogFlags_DisableCreateDirectoryButton |
                    ImGuiFileDialogFlags_DisableThumbnailMode |
                    ImGuiFileDialogFlags_DisableBookmarkMode |
                    ImGuiFileDialogFlags_DisableQuickPathSelection |
                    ImGuiFileDialogFlags_ReadOnlyFileNameField;
    config.path = ".";
    ImGuiFileDialog::Instance()->OpenDialog("embedded", "Choose File", ".bin", config);

    // display
    if (ImGuiFileDialog::Instance()->Display("embedded", ImGuiWindowFlags_NoCollapse, ImVec2(0,0), ImVec2(0,350))) {
        // action if OK
        if (ImGuiFileDialog::Instance()->IsOk()) {
            fl_status_ = status::standby_s;
            // Only add if not already in the list
            std::string filepath = ImGuiFileDialog::Instance()->GetFilePathName();
            if (std::find(fl_names_.begin(), fl_names_.end(), filepath) == fl_names_.end()) {
                fl_names_.push_back(filepath);
            }
        }
    }
    ImGui::PopID();

    ImGui::Separator();
    ImGui::Text("Device flashloader firmware list.");
    ImGui::Text("Ensure flashloader firmwares are selected for all device types to be updated");
    ImGui::Text("Any device that matches the firmware in this list will be updated");
    // List box of selected firmwares
    helpers::listbox_select("Device flashloaders", &fl_names_, 5, &fl_selected_, NULL);
    // Remove any if not needed
    if (ImGui::Button("Remove selected flashloader firmware")) {
        if (fl_selected_ <= fl_names_.size()) {
            fl_names_.erase(fl_names_.begin() + fl_selected_);
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Validate flashloader list - see logs")) {
        host_->validate_network_flashloader_filenames(fw_names_);
    }

    ImGui::Separator();
    ImGui::Text("Select to update devices with flashloader firmware that match the list.");
    ImGui::Text("If NOT selected, all devices must have matching flashloader firmware.");
    ImGui::Checkbox("Only write devices with matching flashloader", &fl_only_write_listed_);

    ImGui::Separator();
    ImGui::Checkbox("Activate", &fl_write_active_);

    if (fl_write_active_) {
        if (fl_only_write_listed_) {
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "WARNING!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Only write devices with matching flashloader is SET.");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "All devices might not be written.");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Double check this is what you want!");
        }
        if (ImGui::Button("Write Network Flashloaders")) {
            if (host_->write_new_network_flashloader(fl_names_, fl_only_write_listed_) == jcs::RET_OK) {
                fl_status_ = status::success_s;
            } else {
                fl_status_ = status::failed_s;
            }
            fl_write_active_ = false;
        }
    }
    switch (fl_status_) {
        default:
        case status::standby_s:
            break;
        case status::success_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Flashloader write successful!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "WARNING!");
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Devices will not respond correctly until power cycle!");
            break;
        case status::failed_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "ERROR!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Flashloader write unsuccessful. See jcs_host logs.");
            break;
    }
}