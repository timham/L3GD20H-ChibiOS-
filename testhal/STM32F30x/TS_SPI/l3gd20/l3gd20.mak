# List of L3GD
GYROSRC = \
	${CHIBIOS}/testhal/STM32F30x/TS_SPI/l3gd20/l3gdcomp.c
#	${CHIBIOS}/Testhal/STM32F30x/TS_SPI/l3gd20/stm32f30x_rcc.c \
#	${CHIBIOS}/Testhal/STM32F30x/TS_SPI/l3gd20/stm32f30x_gpio.c
#	${CHIBIOS}/Testhal/STM32F30x/TS_SPI/l3gd20/stm32f30x_spi.c
#	${CHIBIOS}/Testhal/STM32F30x/TS_SPI/l3gd20/l3gd20.c
#	${CHIBIOS}/Testhal/STM32F30x/TS_SPI/l3gd20/L3GDInitChi.c

	  
# Required include directories
GYROINC = ${CHIBIOS}/os/hal/platforms/stm32f30x
#GYROINC = ${CHIBIOS}/Testhal_wrk/STM32F30x/Gyro \
#	  ${CHIBIOS}/Boards/St_stm32f3_discovery
