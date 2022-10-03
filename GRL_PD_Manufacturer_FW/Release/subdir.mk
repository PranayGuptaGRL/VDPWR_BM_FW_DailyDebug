################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PDManufacturerFWControl.c \
../PDManufacturerMain.c \
../PDManufacturerPeripheral.c \
../PDManufacturerTimer.c \
../cyfxtx.c \
../cyfxusbenumdscr.c 

S_UPPER_SRCS += \
../cyfx_gcc_startup.S 

OBJS += \
./PDManufacturerFWControl.o \
./PDManufacturerMain.o \
./PDManufacturerPeripheral.o \
./PDManufacturerTimer.o \
./cyfx_gcc_startup.o \
./cyfxtx.o \
./cyfxusbenumdscr.o 

C_DEPS += \
./PDManufacturerFWControl.d \
./PDManufacturerMain.d \
./PDManufacturerPeripheral.d \
./PDManufacturerTimer.d \
./cyfxtx.d \
./cyfxusbenumdscr.d 

S_UPPER_DEPS += \
./cyfx_gcc_startup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=arm926ej-s -marm -mthumb-interwork -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -D__CYU3P_TX__=1 -I"C:\Program Files (x86)\Cypress\EZ-USB FX3 SDK\1.3\/fw_lib/1_3_3/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=arm926ej-s -marm -mthumb-interwork -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -x assembler-with-cpp -I"C:\Program Files (x86)\Cypress\EZ-USB FX3 SDK\1.3\/fw_lib/1_3_3/inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


