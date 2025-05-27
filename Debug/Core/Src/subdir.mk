################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ADS1293.c \
../Core/Src/AFE.c \
../Core/Src/Bluetooth.c \
../Core/Src/IIR_Filter.c \
../Core/Src/LSM6DSLTR.c \
../Core/Src/app_debug.c \
../Core/Src/app_entry.c \
../Core/Src/debug.c \
../Core/Src/exti.c \
../Core/Src/hw_timerserver.c \
../Core/Src/hw_uart.c \
../Core/Src/kernel.c \
../Core/Src/main.c \
../Core/Src/stm32_lpm_if.c \
../Core/Src/stm32wbxx_hal_msp.c \
../Core/Src/stm32wbxx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wbxx.c \
../Core/Src/timer.c 

OBJS += \
./Core/Src/ADS1293.o \
./Core/Src/AFE.o \
./Core/Src/Bluetooth.o \
./Core/Src/IIR_Filter.o \
./Core/Src/LSM6DSLTR.o \
./Core/Src/app_debug.o \
./Core/Src/app_entry.o \
./Core/Src/debug.o \
./Core/Src/exti.o \
./Core/Src/hw_timerserver.o \
./Core/Src/hw_uart.o \
./Core/Src/kernel.o \
./Core/Src/main.o \
./Core/Src/stm32_lpm_if.o \
./Core/Src/stm32wbxx_hal_msp.o \
./Core/Src/stm32wbxx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wbxx.o \
./Core/Src/timer.o 

C_DEPS += \
./Core/Src/ADS1293.d \
./Core/Src/AFE.d \
./Core/Src/Bluetooth.d \
./Core/Src/IIR_Filter.d \
./Core/Src/LSM6DSLTR.d \
./Core/Src/app_debug.d \
./Core/Src/app_entry.d \
./Core/Src/debug.d \
./Core/Src/exti.d \
./Core/Src/hw_timerserver.d \
./Core/Src/hw_uart.d \
./Core/Src/kernel.d \
./Core/Src/main.d \
./Core/Src/stm32_lpm_if.d \
./Core/Src/stm32wbxx_hal_msp.d \
./Core/Src/stm32wbxx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wbxx.d \
./Core/Src/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ADS1293.cyclo ./Core/Src/ADS1293.d ./Core/Src/ADS1293.o ./Core/Src/ADS1293.su ./Core/Src/AFE.cyclo ./Core/Src/AFE.d ./Core/Src/AFE.o ./Core/Src/AFE.su ./Core/Src/Bluetooth.cyclo ./Core/Src/Bluetooth.d ./Core/Src/Bluetooth.o ./Core/Src/Bluetooth.su ./Core/Src/IIR_Filter.cyclo ./Core/Src/IIR_Filter.d ./Core/Src/IIR_Filter.o ./Core/Src/IIR_Filter.su ./Core/Src/LSM6DSLTR.cyclo ./Core/Src/LSM6DSLTR.d ./Core/Src/LSM6DSLTR.o ./Core/Src/LSM6DSLTR.su ./Core/Src/app_debug.cyclo ./Core/Src/app_debug.d ./Core/Src/app_debug.o ./Core/Src/app_debug.su ./Core/Src/app_entry.cyclo ./Core/Src/app_entry.d ./Core/Src/app_entry.o ./Core/Src/app_entry.su ./Core/Src/debug.cyclo ./Core/Src/debug.d ./Core/Src/debug.o ./Core/Src/debug.su ./Core/Src/exti.cyclo ./Core/Src/exti.d ./Core/Src/exti.o ./Core/Src/exti.su ./Core/Src/hw_timerserver.cyclo ./Core/Src/hw_timerserver.d ./Core/Src/hw_timerserver.o ./Core/Src/hw_timerserver.su ./Core/Src/hw_uart.cyclo ./Core/Src/hw_uart.d ./Core/Src/hw_uart.o ./Core/Src/hw_uart.su ./Core/Src/kernel.cyclo ./Core/Src/kernel.d ./Core/Src/kernel.o ./Core/Src/kernel.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32_lpm_if.cyclo ./Core/Src/stm32_lpm_if.d ./Core/Src/stm32_lpm_if.o ./Core/Src/stm32_lpm_if.su ./Core/Src/stm32wbxx_hal_msp.cyclo ./Core/Src/stm32wbxx_hal_msp.d ./Core/Src/stm32wbxx_hal_msp.o ./Core/Src/stm32wbxx_hal_msp.su ./Core/Src/stm32wbxx_it.cyclo ./Core/Src/stm32wbxx_it.d ./Core/Src/stm32wbxx_it.o ./Core/Src/stm32wbxx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wbxx.cyclo ./Core/Src/system_stm32wbxx.d ./Core/Src/system_stm32wbxx.o ./Core/Src/system_stm32wbxx.su ./Core/Src/timer.cyclo ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/timer.su

.PHONY: clean-Core-2f-Src

