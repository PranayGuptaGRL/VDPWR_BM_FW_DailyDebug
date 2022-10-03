################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=none -me --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_RELEASE/GRL_SSBL_CM" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/device_support/f2838x/common/include" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/flash_api/f2838x/cm/include/FlashAPI" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_RELEASE/GRL_SSBL_CM" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/driverlib/f2838x/driverlib_cm" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x/driverlib_cm" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip/examples/enet_lwip_udp/cm" --define=_FLASH --define=CM --define=ccs -g --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


