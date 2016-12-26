# stm32-bme280
Bosch BME280 driver for STM32 using HAL<br>
SPI or I2C methods available

This was originally written on an STM32F446RE-Nucleo so its header file is stm32f4xx_hal.h<br>
If you use something like an STM32F103RB, you would need to change that to stm32f1xx_hal.h, or stm32l4xx_hal.h for an STM32L476, et cetera.

This code also assumes that you route printf() to usart2.

My changes were made to bme280_support.c, and the other two files should be identical to the factory Bosch files.

To get SPI working:<br>
comment out the I2C line and uncomment the SPI line in bme280_support.c lines 157/158<br>
set prescaler to 8 or higher, clock polarity to high, and phase to two-edge:<br>

  hspi2.Instance = SPI2;<br>
  hspi2.Init.Mode = SPI_MODE_MASTER;<br>
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;<br>
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;<br>
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;<br>
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;<br>
  hspi2.Init.NSS = SPI_NSS_SOFT;<br>
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;<br>
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;<br>
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;<br>
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;<br>
  hspi2.Init.CRCPolynomial = 10;<br>
 
  Basically we need to get the SPI clock speed under 10MHz (according to the data sheet, pg.36)<br>
  For for a 64mhz stm32f103 running APB1 peripheral bus clock at 32Mhz, use a prescaler of 4 to yield 8 Mb/s SPI clock speed. <br>
  It is very helpful to know what clock speed your peripherals run at.
  
I2C is not nearly as finicky and just works.

Tested and working on an STM32F446RE-Nucleo, and with the minor changes mentioned above (header file and SPI baud rate prescaler), an STM32F103RB-Nucleo.
