################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./CM_SSBL_Main.obj" \
"./GrlPDComm.obj" \
"./cm.obj" \
"./pinout.obj" \
"./startup_ccs.obj" \
"../2838x_flash_lnk_cm.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/c2000/C2000Ware_4_00_00_00/libraries/flash_api/f2838x/cm/lib/F2838x_CM_FlashAPI.lib" \
-l"C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x/driverlib_cm/ccs/Debug/driverlib_cm.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
GRL_SSBL_CM.out \

EXE_OUTPUTS__QUOTED += \
"GRL_SSBL_CM.out" \

BIN_OUTPUTS += \
GRL_SSBL_CM.txt \

BIN_OUTPUTS__QUOTED += \
"GRL_SSBL_CM.txt" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "GRL_SSBL_CM.out" secondary-outputs

# Tool invocations
GRL_SSBL_CM.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=none -me --define=_FLASH --define=CM --define=ccs -g --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on -z -m"GRL_SSBL_CM.map" --heap_size=4096 --stack_size=4096 -i"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/lib" -i"C:/ti/c2000/C2000Ware_4_00_00_00/libraries/flash_api/f2838x/cm/lib" -i"C:/ti/ccs1040/ccs/tools/compiler" -i"C:/ti/c2000/C2000Ware_4_00_00_00/driverlib/f2838x/driverlib_cm/ccs/Debug" -i"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="GRL_SSBL_CM_linkInfo.xml" --rom_model -o "GRL_SSBL_CM.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

GRL_SSBL_CM.txt: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm Hex Utility'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armhex" "GRL_SSBL_CM.out"  -boot -gpio8 -a -o "GRL_SSBL_CM.txt"
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "CM_SSBL_Main.obj" "GrlPDComm.obj" "cm.obj" "pinout.obj" "startup_ccs.obj" 
	-$(RM) "CM_SSBL_Main.d" "GrlPDComm.d" "cm.d" "pinout.d" "startup_ccs.d" 
	-@echo 'Finished clean'
	-@echo ' '

secondary-outputs: $(BIN_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

