# stm32-bme280
Bosch BME280 driver for STM32 using HAL
SPI or I2C methods available

This was originally written on an STM32F446RE-Nucleo so its header file is stm32f4xx_hal.h
If you use something like an STM32F103RB-Nucleo, you would need to change that to stm32f1xx_hal.h, or stm32l4xx_hal.h for an STM32L476, et cetera.

My changes were made to bme280_support.c, and the other two files should be identical to the factory Bosch files.

To get SPI working:<br>
comment out the I2C line and uncomment the SPI line in bme280_support.c lines 157/158<br>
set prescaler to 8 or larger, clock polarity to high and phase to two-edge:<br>

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
  For for an stm32f103, use a prescaler of 4 to yield 8 Mb/s clock speed.
  
