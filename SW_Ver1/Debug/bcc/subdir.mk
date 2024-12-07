################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bcc/bcc.c \
../bcc/bcc_communication.c \
../bcc/monitoring.c 

OBJS += \
./bcc/bcc.o \
./bcc/bcc_communication.o \
./bcc/monitoring.o 

C_DEPS += \
./bcc/bcc.d \
./bcc/bcc_communication.d \
./bcc/monitoring.d 


# Each subdirectory must supply rules for building sources it contributes
bcc/%.o bcc/%.su bcc/%.cyclo: ../bcc/%.c bcc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Nam_4_Ki_1/Project_2/source-code/Software/Battery-Management-System/SW_Ver1/bcc" -I"D:/Nam_4_Ki_1/Project_2/source-code/Software/Battery-Management-System/SW_Ver1/bcc_stm32" -I"D:/Nam_4_Ki_1/Project_2/source-code/Software/Battery-Management-System/SW_Ver1/Core/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bcc

clean-bcc:
	-$(RM) ./bcc/bcc.cyclo ./bcc/bcc.d ./bcc/bcc.o ./bcc/bcc.su ./bcc/bcc_communication.cyclo ./bcc/bcc_communication.d ./bcc/bcc_communication.o ./bcc/bcc_communication.su ./bcc/monitoring.cyclo ./bcc/monitoring.d ./bcc/monitoring.o ./bcc/monitoring.su

.PHONY: clean-bcc

