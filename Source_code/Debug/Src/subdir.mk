################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/bno055.c \
../Src/it.c \
../Src/main.c \
../Src/msp.c \
../Src/orientation.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/bno055.o \
./Src/it.o \
./Src/main.o \
./Src/msp.o \
./Src/orientation.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/bno055.d \
./Src/it.d \
./Src/main.d \
./Src/msp.d \
./Src/orientation.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/HQLap/Documents/stm32/workspace/MCU2/Bno555_Implementation/Inc" -I"C:/Users/HQLap/Documents/stm32/workspace/MCU2/Bno555_Implementation/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/HQLap/Documents/stm32/workspace/MCU2/Bno555_Implementation/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I"C:/Users/HQLap/Documents/stm32/workspace/MCU2/Bno555_Implementation/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/HQLap/Documents/stm32/workspace/MCU2/Bno555_Implementation/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


