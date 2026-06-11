################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Filters.c \
../Core/Src/HC05.c \
../Core/Src/ICM20948_I2C.c \
../Core/Src/MS5611_SPI.c \
../Core/Src/MTF01P.c \
../Core/Src/NMEA.c \
../Core/Src/PIDControl.c \
../Core/Src/RC.c \
../Core/Src/Ringbuffer.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/Filters.o \
./Core/Src/HC05.o \
./Core/Src/ICM20948_I2C.o \
./Core/Src/MS5611_SPI.o \
./Core/Src/MTF01P.o \
./Core/Src/NMEA.o \
./Core/Src/PIDControl.o \
./Core/Src/RC.o \
./Core/Src/Ringbuffer.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/Filters.d \
./Core/Src/HC05.d \
./Core/Src/ICM20948_I2C.d \
./Core/Src/MS5611_SPI.d \
./Core/Src/MTF01P.d \
./Core/Src/NMEA.d \
./Core/Src/PIDControl.d \
./Core/Src/RC.d \
./Core/Src/Ringbuffer.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Filters.cyclo ./Core/Src/Filters.d ./Core/Src/Filters.o ./Core/Src/Filters.su ./Core/Src/HC05.cyclo ./Core/Src/HC05.d ./Core/Src/HC05.o ./Core/Src/HC05.su ./Core/Src/ICM20948_I2C.cyclo ./Core/Src/ICM20948_I2C.d ./Core/Src/ICM20948_I2C.o ./Core/Src/ICM20948_I2C.su ./Core/Src/MS5611_SPI.cyclo ./Core/Src/MS5611_SPI.d ./Core/Src/MS5611_SPI.o ./Core/Src/MS5611_SPI.su ./Core/Src/MTF01P.cyclo ./Core/Src/MTF01P.d ./Core/Src/MTF01P.o ./Core/Src/MTF01P.su ./Core/Src/NMEA.cyclo ./Core/Src/NMEA.d ./Core/Src/NMEA.o ./Core/Src/NMEA.su ./Core/Src/PIDControl.cyclo ./Core/Src/PIDControl.d ./Core/Src/PIDControl.o ./Core/Src/PIDControl.su ./Core/Src/RC.cyclo ./Core/Src/RC.d ./Core/Src/RC.o ./Core/Src/RC.su ./Core/Src/Ringbuffer.cyclo ./Core/Src/Ringbuffer.d ./Core/Src/Ringbuffer.o ./Core/Src/Ringbuffer.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

