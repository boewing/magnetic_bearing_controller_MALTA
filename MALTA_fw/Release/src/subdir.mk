################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LD_SRCS += \
../src/lscript.ld 

C_SRCS += \
../src/2dq_transformation.c \
../src/Control_Handle.c \
../src/PID.c \
../src/adc.c \
../src/cli.c \
../src/commands.c \
../src/filter_malta.c \
../src/iir.c \
../src/interrupt.c \
../src/malta_configure.c \
../src/malta_control.c \
../src/malta_control_interface.c \
../src/malta_current_control.c \
../src/malta_main.c \
../src/malta_position_control.c \
../src/malta_state_machine.c \
../src/matrix_operations.c \
../src/medfilt.c \
../src/platform.c \
../src/pwm.c \
../src/recoder.c \
../src/reference_generation.c \
../src/sine_lookup.c \
../src/softstart.c \
../src/step_optimizer.c \
../src/uartlite_laser_sensor.c \
../src/variable_recorder.c 

OBJS += \
./src/2dq_transformation.o \
./src/Control_Handle.o \
./src/PID.o \
./src/adc.o \
./src/cli.o \
./src/commands.o \
./src/filter_malta.o \
./src/iir.o \
./src/interrupt.o \
./src/malta_configure.o \
./src/malta_control.o \
./src/malta_control_interface.o \
./src/malta_current_control.o \
./src/malta_main.o \
./src/malta_position_control.o \
./src/malta_state_machine.o \
./src/matrix_operations.o \
./src/medfilt.o \
./src/platform.o \
./src/pwm.o \
./src/recoder.o \
./src/reference_generation.o \
./src/sine_lookup.o \
./src/softstart.o \
./src/step_optimizer.o \
./src/uartlite_laser_sensor.o \
./src/variable_recorder.o 

C_DEPS += \
./src/2dq_transformation.d \
./src/Control_Handle.d \
./src/PID.d \
./src/adc.d \
./src/cli.d \
./src/commands.d \
./src/filter_malta.d \
./src/iir.d \
./src/interrupt.d \
./src/malta_configure.d \
./src/malta_control.d \
./src/malta_control_interface.d \
./src/malta_current_control.d \
./src/malta_main.d \
./src/malta_position_control.d \
./src/malta_state_machine.d \
./src/matrix_operations.d \
./src/medfilt.d \
./src/platform.d \
./src/pwm.d \
./src/recoder.d \
./src/reference_generation.d \
./src/sine_lookup.d \
./src/softstart.d \
./src/step_optimizer.d \
./src/uartlite_laser_sensor.d \
./src/variable_recorder.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 gcc compiler'
	arm-none-eabi-gcc -Wall -O3 -c -fmessage-length=0 -MT"$@" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -I../../MALTA_fw_bsp/ps7_cortexa9_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


