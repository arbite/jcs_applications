#
# Tool GUI compilation
#

JCS_TOOL_GUI_SRC  = build/tools/tool_gui/tool_gui.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/helpers.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/sampler.o

JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_host.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_joint_controller.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_motor_controller.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_encoder_absolute.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_encoder_absolute_slide_by_hall.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_braking_chopper.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_encoder_relative.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_strain_gauge.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_brake_clutch.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_analog.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_load_switch.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_device/gui_device_thermal_simple.o

JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_process/gui_process_pid.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_process/gui_process_pd.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_process/gui_process_interpolator.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_process/gui_process_transform.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_process/gui_process_map.o

JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_plot/gui_plot.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_plot/plot_sink_int.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_plot/plot_sink_opstate.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_plot/plot_sink_plot.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_plot/plot_source_slider.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_host_oscilloscope/gui_host_oscilloscope.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_host_input_stimulus/gui_host_input_stimulus.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_host_analysis/gui_host_analysis.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_host_network_firmware/gui_host_network_firmware.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/helpers/stimulus.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/helpers/gui_stimulus.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/helpers/plot_measurement_multi.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_oscilloscope/gui_oscilloscope.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_parameter/gui_parameter.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_parameter/gui_parameter_types.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_firmware_update/gui_firmware_update.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_host_statistics/gui_host_statistics.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_host_logger/gui_host_logger.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_tune/gui_mc_tune.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_tune/mc_test_step_response.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_cogging/gui_mc_cogging.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_encoder/gui_mc_encoder.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_current_test/gui_mc_current_test.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_encoder_calib/gui_mc_encoder_calib.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_encoder_calib/mc_encoder_corrector.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_thermal_calib/gui_mc_thermal_calib.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_mc_thermal_calib/mc_thermal_fitter.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_tools/gui_bc_tune/gui_bc_tune.o

# Hoppy robot
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_fun/2d_hopper/gui_host_2d_hopper.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_fun/2d_hopper/hopper_2d_kinematics.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_fun/2d_hopper/hopper_2d.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_fun/2d_hopper/hopper_2d_simple_kine.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_fun/2d_hopper/hopper_2d_vmc_simple_xy_ctl.o
JCS_TOOL_GUI_SRC += build/tools/tool_gui/gui/gui_fun/2d_hopper/hopper_2d_virtual_model_ctl.o


# jcs_host parameter helpers
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_motor_controller.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_joint_controller.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_encoder_absolute.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_encoder_absolute_slide_by_hall.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_braking_chopper.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_encoder_relative.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_strain_gauge.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_brake_clutch.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_analog.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_load_switch.o
DEV_HOST_SRC += build/parameter_helpers/device/jcs_dev_thermal_simple.o

DEV_HOST_SRC += build/parameter_helpers/process/jcs_proc_pid.o
DEV_HOST_SRC += build/parameter_helpers/process/jcs_proc_pd.o
DEV_HOST_SRC += build/parameter_helpers/process/jcs_proc_interpolator.o
DEV_HOST_SRC += build/parameter_helpers/process/jcs_proc_transform.o
DEV_HOST_SRC += build/parameter_helpers/process/jcs_proc_map.o

# Imgui and misc
3RD_PARTY_SRC += build/imgui/imgui/imgui.o
3RD_PARTY_SRC += build/imgui/imgui/imgui_widgets.o
3RD_PARTY_SRC += build/imgui/imgui/imgui_tables.o
3RD_PARTY_SRC += build/imgui/imgui/imgui_draw.o
3RD_PARTY_SRC += build/imgui/imgui/misc/cpp/imgui_stdlib.o
3RD_PARTY_SRC += build/imgui/imgui/backends/imgui_impl_glfw.o
3RD_PARTY_SRC += build/imgui/imgui/backends/imgui_impl_opengl2.o
3RD_PARTY_SRC += build/imgui/implot/implot.o
3RD_PARTY_SRC += build/imgui/implot/implot_items.o
3RD_PARTY_SRC += build/imgui/imgui_filedialog/ImGuiFileDialog.o

# C targets
3RD_PARTY_CSRC += build/imgui/implot_demos/3rdparty/kissfft/kiss_fftr.o
3RD_PARTY_CSRC += build/imgui/implot_demos/3rdparty/kissfft/kiss_fft.o

3RD_PARTY_COPTS	 = -Dkiss_fft_scalar=double


JCS_TOOL_GUI_INC  = -I$(TARGET_PATH)tools/tool_gui/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_device/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_process/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_plot/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_host_statistics/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_host_oscilloscope/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_host_input_stimulus/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_host_analysis/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_host_network_firmware/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/helpers/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_oscilloscope/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_host_logger/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_parameter/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_firmware_update/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_mc_tune/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_mc_cogging/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_mc_encoder/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_mc_current_test/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_mc_encoder_calib/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_mc_thermal_calib/
JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_tools/gui_bc_tune/

JCS_TOOL_GUI_INC += -I$(TARGET_PATH)tools/tool_gui/gui/gui_fun/2d_hopper/

JCS_TOOL_GUI_INC += -I$(3RD_PARTY_PATH)imgui/imgui/
JCS_TOOL_GUI_INC += -I$(3RD_PARTY_PATH)imgui/imgui/misc/cpp/
JCS_TOOL_GUI_INC += -I$(3RD_PARTY_PATH)imgui/imgui/backends/
JCS_TOOL_GUI_INC += -I$(3RD_PARTY_PATH)imgui/implot/
JCS_TOOL_GUI_INC += -I$(3RD_PARTY_PATH)imgui/imgui_filedialog/
JCS_TOOL_GUI_INC += -I$(3RD_PARTY_PATH)imgui/implot_demos/3rdparty/
JCS_TOOL_GUI_INC += -I$(3RD_PARTY_PATH)imgui/implot_demos/3rdparty/kissfft/

JCS_TOOL_GUI_INC += -I$(JCS_DEV_HOST_PATH)parameter_helpers/
JCS_TOOL_GUI_INC += -I$(JCS_DEV_HOST_PATH)parameter_helpers/device/
JCS_TOOL_GUI_INC += -I$(JCS_DEV_HOST_PATH)parameter_helpers/process/

# JCS_TOOL_GUI_LIBEXT = -lGL -ldl `sdl2-config --libs`
# JCS_TOOL_GUI_CXXFLAGS = `sdl2-config --cflags`

JCS_TOOL_GUI_LIBEXT = -lGL `pkg-config --static --libs glfw3`
JCS_TOOL_GUI_CXXFLAGS = `pkg-config --cflags glfw3`