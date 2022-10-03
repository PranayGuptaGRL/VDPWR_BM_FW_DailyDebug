################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 --vcu_support=vcrc -O4 --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/device_support/f2838x/headers/include" --include_path="C:/ti/c2000/C2000Ware_4_00_00_00/device_support/f2838x/common/include" --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_RELEASE/GRL_SSBL_CPU2_IPC" --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_RELEASE/GRL_SSBL_CPU2_IPC/device" --include_path="C:/ti/c2000/C2000Ware_3_04_00_00/driverlib/f2838x/driverlib" --include_path="C:/ti/c2000/C2000Ware_3_04_00_00/libraries/flash_api/f2838x/c28x/include/FlashAPI" --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --advice:performance=all --define=DEBUG --define=_FLASH --define=CPU2 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="D:/V-TE-PWR/EthBased/TI_SDK_Workspace/BM_RELEASE/GRL_SSBL_CPU2_IPC/CPU1_FLASH/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


