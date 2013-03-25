# List of L3GD
GYROSRC = \
	${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro/l3gdcomp.c  \
	${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro/stm32f30x_rcc.c \
	${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro/stm32f30x_gpio.c
#	${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro/stm32f30x_spi.c
#	${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro/l3gd20.c
#	${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro/L3GDInitChi.c

	  
# Required include directories
GYROINC = ${CHIBIOS}/os/hal/platforms/stm32f30x
#GYROINC = ${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro \
#	  ${CHIBIOS}/Boards/St_stm32f3_discovery
