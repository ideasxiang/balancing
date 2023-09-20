################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Tasks/Src/buzzing_task.c \
../Core/Tasks/Src/can_msg_processor.c \
../Core/Tasks/Src/control_input_task.c \
../Core/Tasks/Src/error_handler.c \
../Core/Tasks/Src/imu_processing_task.c \
../Core/Tasks/Src/motor_config.c \
../Core/Tasks/Src/motor_control.c \
../Core/Tasks/Src/motor_feedback_task.c \
../Core/Tasks/Src/movement_control_task.c \
../Core/Tasks/Src/startup_task.c 

OBJS += \
./Core/Tasks/Src/buzzing_task.o \
./Core/Tasks/Src/can_msg_processor.o \
./Core/Tasks/Src/control_input_task.o \
./Core/Tasks/Src/error_handler.o \
./Core/Tasks/Src/imu_processing_task.o \
./Core/Tasks/Src/motor_config.o \
./Core/Tasks/Src/motor_control.o \
./Core/Tasks/Src/motor_feedback_task.o \
./Core/Tasks/Src/movement_control_task.o \
./Core/Tasks/Src/startup_task.o 

C_DEPS += \
./Core/Tasks/Src/buzzing_task.d \
./Core/Tasks/Src/can_msg_processor.d \
./Core/Tasks/Src/control_input_task.d \
./Core/Tasks/Src/error_handler.d \
./Core/Tasks/Src/imu_processing_task.d \
./Core/Tasks/Src/motor_config.d \
./Core/Tasks/Src/motor_control.d \
./Core/Tasks/Src/motor_feedback_task.d \
./Core/Tasks/Src/movement_control_task.d \
./Core/Tasks/Src/startup_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Tasks/Src/%.o Core/Tasks/Src/%.su Core/Tasks/Src/%.cyclo: ../Core/Tasks/Src/%.c Core/Tasks/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Tasks-2f-Src

clean-Core-2f-Tasks-2f-Src:
	-$(RM) ./Core/Tasks/Src/buzzing_task.cyclo ./Core/Tasks/Src/buzzing_task.d ./Core/Tasks/Src/buzzing_task.o ./Core/Tasks/Src/buzzing_task.su ./Core/Tasks/Src/can_msg_processor.cyclo ./Core/Tasks/Src/can_msg_processor.d ./Core/Tasks/Src/can_msg_processor.o ./Core/Tasks/Src/can_msg_processor.su ./Core/Tasks/Src/control_input_task.cyclo ./Core/Tasks/Src/control_input_task.d ./Core/Tasks/Src/control_input_task.o ./Core/Tasks/Src/control_input_task.su ./Core/Tasks/Src/error_handler.cyclo ./Core/Tasks/Src/error_handler.d ./Core/Tasks/Src/error_handler.o ./Core/Tasks/Src/error_handler.su ./Core/Tasks/Src/imu_processing_task.cyclo ./Core/Tasks/Src/imu_processing_task.d ./Core/Tasks/Src/imu_processing_task.o ./Core/Tasks/Src/imu_processing_task.su ./Core/Tasks/Src/motor_config.cyclo ./Core/Tasks/Src/motor_config.d ./Core/Tasks/Src/motor_config.o ./Core/Tasks/Src/motor_config.su ./Core/Tasks/Src/motor_control.cyclo ./Core/Tasks/Src/motor_control.d ./Core/Tasks/Src/motor_control.o ./Core/Tasks/Src/motor_control.su ./Core/Tasks/Src/motor_feedback_task.cyclo ./Core/Tasks/Src/motor_feedback_task.d ./Core/Tasks/Src/motor_feedback_task.o ./Core/Tasks/Src/motor_feedback_task.su ./Core/Tasks/Src/movement_control_task.cyclo ./Core/Tasks/Src/movement_control_task.d ./Core/Tasks/Src/movement_control_task.o ./Core/Tasks/Src/movement_control_task.su ./Core/Tasks/Src/startup_task.cyclo ./Core/Tasks/Src/startup_task.d ./Core/Tasks/Src/startup_task.o ./Core/Tasks/Src/startup_task.su

.PHONY: clean-Core-2f-Tasks-2f-Src

