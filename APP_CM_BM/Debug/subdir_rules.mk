################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=none -me --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_Flashing/APP_CM_BM" --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip/lwip-2.1.2/src/apps/http" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip" --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_Flashing/APP_CM_BM" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x/driverlib_cm" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip/examples/enet_lwip_udp/cm" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip/lwip-2.1.2" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip/lwip-2.1.2/src/include" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip/lwip-2.1.2/src/apps" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/communications/Ethernet/third_party/lwip/lwip-2.1.2/ports/C2000/include" --define=_FLASH --define=ccs -g --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


