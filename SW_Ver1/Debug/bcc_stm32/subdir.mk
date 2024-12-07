################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bcc_stm32/bcc_peripheries.c \
../bcc_stm32/bcc_wait.c 

OBJS += \
./bcc_stm32/bcc_peripheries.o \
./bcc_stm32/bcc_wait.o 

C_DEPS += \
./bcc_stm32/bcc_peripheries.d \
./bcc_stm32/bcc_wait.d 


# Each subdirectory must supply rules for building sources it contributes
bcc_stm32/%.o bcc_stm32/%.su bcc_stm32/%.cyclo: ../bcc_stm32/%.c bcc_stm32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Nam_4_Ki_1/Project_2/source-code/Software/Battery-Management-System/SW_Ver1/bcc" -I"D:/Nam_4_Ki_1/Project_2/source-code/Software/Battery-Management-System/SW_Ver1/bcc_stm32" -I"D:/Nam_4_Ki_1/Project_2/source-code/Software/Battery-Management-System/SW_Ver1/Core/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bcc_stm32

clean-bcc_stm32:
	-$(RM) ./bcc_stm32/bcc_peripheries.cyclo ./bcc_stm32/bcc_peripheries.d ./bcc_stm32/bcc_peripheries.o ./bcc_stm32/bcc_peripheries.su ./bcc_stm32/bcc_wait.cyclo ./bcc_stm32/bcc_wait.d ./bcc_stm32/bcc_wait.o ./bcc_stm32/bcc_wait.su

.PHONY: clean-bcc_stm32

