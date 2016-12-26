# stm32-bme280
Bosch BME280 driver for STM32 using HAL
SPI or I2C methods available

My changes were made to bme280_support.c, and the other two files should be identical to the factory Bosch files.

To get SPI working, set prescaler to 8 or larger, clock polarity to high and phase to two-edge:

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
 
  
  
