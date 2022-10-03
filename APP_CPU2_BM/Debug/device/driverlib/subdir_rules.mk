################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
device/driverlib/%.obj: ../device/driverlib/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 --vcu_support=vcrc --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_Flashing/APP_CPU2_BM" --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_Flashing/APP_CPU2_BM/device" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/libraries/flash_api/f2838x/c28x/include/FlashAPI" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x/driverlib/inc" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x/driverlib" --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH --define=DEBUG -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="device/driverlib/$(basename $(<F)).d_raw" --obj_directory="device/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


