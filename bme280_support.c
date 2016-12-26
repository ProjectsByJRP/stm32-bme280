/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bme280_support.c
* Date: 2016/07/04
* Revision: 1.0.6 $
*
* Usage: Sensor Driver support file for BME280 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "bme280.h"

#define BME280_API
/*Enable the macro BME280_API to use this support file */
/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BME280_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BME280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read */
s8 BME280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 I2C_routine(void);
s8 SPI_routine(void);
#endif
/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BME280_delay_msek(u32 msek);
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bme280_data_readout_template(void);
/*----------------------------------------------------------------------------*
 *  struct bme280_t parameters can be accessed by using bme280
 *	bme280_t having the following parameters
 *	Bus write function pointer: BME280_WR_FUNC_PTR
 *	Bus read function pointer: BME280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bme280_t bme280;
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bme280_data_readout_template(void)
{
	/* The variable used to assign the standby time*/
	u8 v_stand_by_time_u8 = BME280_INIT_VALUE;
	/* The variable used to read uncompensated temperature*/
	s32 v_data_uncomp_temp_s32 = BME280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_pres_s32 = BME280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_hum_s32 = BME280_INIT_VALUE;
	/* The variable used to read compensated temperature*/
	s32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
	/* The variable used to read compensated pressure*/
	u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
	/* The variable used to read compensated humidity*/
	u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};

	/* result of communication results*/
	s32 com_rslt = ERROR;

 /*********************** START INITIALIZATION ************************/
  /*	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bme280 API*/
 	#ifdef BME280_API
	//I2C_routine();
	SPI_routine();
	#endif
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
*-------------------------------------------------------------------------*/
	com_rslt = bme280_init(&bme280);

	/*	For initialization it is required to set the mode of
	 *	the sensor as "NORMAL"
	 *	data acquisition/read/write is possible in this mode
	 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	com_rslt += bme280_set_power_mode(BME280_FORCED_MODE);
	/*	For reading the pressure, humidity and temperature data it is required to
	 *	set the OSS setting of humidity, pressure and temperature
	 * The "BME280_CTRLHUM_REG_OSRSH" register sets the humidity
	 * data acquisition options of the device.
	 * changes to this registers only become effective after a write operation to
	 * "BME280_CTRLMEAS_REG" register.
	 * In the code automated reading and writing of "BME280_CTRLHUM_REG_OSRSH"
	 * register first set the "BME280_CTRLHUM_REG_OSRSH" and then read and write
	 * the "BME280_CTRLMEAS_REG" register in the function*/
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	/* set the pressure oversampling*/
	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
	/* set the temperature oversampling*/
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
/*--------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the standby time of the sensor input
	 *	value have to be given
	 *	Normal mode comprises an automated perpetual cycling between an (active)
	 *	Measurement period and an (inactive) standby period.
	 *	The standby time is determined by the contents of the register t_sb.
	 *	Standby time can be set using BME280_STANDBYTIME_125_MS.
	 *	Usage Hint : bme280_set_standbydur(BME280_STANDBYTIME_125_MS)*/
	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);

	/* This API used to read back the written value of standby time*/
	//com_rslt += bme280_get_standby_durn(&v_stand_by_time_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*------------------------------------------------------------------*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------*
************ START READ UNCOMPENSATED PRESSURE, TEMPERATURE
AND HUMIDITY DATA ********
*---------------------------------------------------------------------*/

	/* API is used to read the uncompensated humidity*/
	com_rslt += bme280_read_uncomp_humidity(&v_data_uncomp_hum_s32);

	/* API is used to read the uncompensated temperature*/
	com_rslt += bme280_read_uncomp_temperature(&v_data_uncomp_temp_s32);

	/* API is used to read the uncompensated pressure*/
	com_rslt += bme280_read_uncomp_pressure(&v_data_uncomp_pres_s32);
	HAL_Delay(10);
	/* API is used to read the uncompensated temperature,pressure
	and humidity data */
	com_rslt += bme280_read_uncomp_pressure_temperature_humidity(
	&v_data_uncomp_temp_s32, &v_data_uncomp_pres_s32, &v_data_uncomp_hum_s32);

/*--------------------------------------------------------------------*
************ END READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
*-------------------------------------------------------------------------*/

/*------------------------------------------------------------------*
************ START READ COMPENSATED PRESSURE, TEMPERATURE
AND HUMIDITY DATA ********
*---------------------------------------------------------------------*/
	/* API is used to compute the compensated temperature*/
	v_comp_temp_s32[0] = bme280_compensate_temperature_int32(
			v_data_uncomp_temp_s32);

	/* API is used to compute the compensated pressure*/
	v_comp_press_u32[0] = bme280_compensate_pressure_int32(
			v_data_uncomp_pres_s32);

	/* API is used to compute the compensated humidity*/
	v_comp_humidity_u32[0] = bme280_compensate_humidity_int32(
			v_data_uncomp_hum_s32);

	/* API is used to read the compensated temperature, humidity and pressure*/
	com_rslt += bme280_read_pressure_temperature_humidity(
	&v_comp_press_u32[1], &v_comp_temp_s32[1],  &v_comp_humidity_u32[1]);

	float imp_temp = ((float)(v_comp_temp_s32[1])/100)*1.8+32;		// convert to fahrenheit
	//float temp = ((float)v_comp_temp_s32[1]/100);
	float imp_press = ((float)(v_comp_press_u32[1])/100)*.0295300; 	// convert to inches of mercury
	//float press = ((float)(v_comp_press_u32[1])/100);
	float imp_humi = ((float)(v_comp_humidity_u32[1])/1024);		// relative humidity
	float dewpt = ((float)v_comp_temp_s32[1]/100) - ((100 - imp_humi) / 5. );
	dewpt = dewpt * 1.8 + 32;
	printf("Temp: %.f DegF,  Press: %.2f inHg,  Humi: %.f%% rH,  DewPt: %.f DegF\r\n",
								imp_temp,
								imp_press,
								imp_humi,
								dewpt);
/*--------------------------------------------------------------------*
************ END READ COMPENSATED PRESSURE, TEMPERATURE AND HUMIDITY ********
*-------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
	/*	For de-initialization it is required to set the mode of
	 *	the sensor as "SLEEP"
	 *	the device reaches the lowest power consumption only
	 *	In SLEEP mode no measurements are performed
	 *	All registers are accessible
	 *	by using the below API able to set the power mode as SLEEP*/
	 /* Set the power mode as SLEEP*/
	com_rslt += bme280_set_power_mode(BME280_SLEEP_MODE);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
return com_rslt;
}

#ifdef BME280_API
#define SPI_READ	0x80
#define SPI_WRITE	0x7F
#define BME280_DATA_INDEX	1
#define BME280_ADDRESS_INDEX	2
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bme280
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bme280 the following structure parameter can be accessed
 *	Bus write function pointer: BME280_WR_FUNC_PTR
 *	Bus read function pointer: BME280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bme280.bus_write = BME280_I2C_bus_write;
	bme280.bus_read = BME280_I2C_bus_read;
	bme280.dev_addr = BME280_I2C_ADDRESS2;
	bme280.delay_msec = BME280_delay_msek;

	return BME280_INIT_VALUE;
}

/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bme280
 *--------------------------------------------------------------------------*/
s8 SPI_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bme280 the following structure parameter can be accessed
 *	Bus write function pointer: BME280_WR_FUNC_PTR
 *	Bus read function pointer: BME280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bme280.bus_write = BME280_SPI_bus_write;
	bme280.bus_read = BME280_SPI_bus_read;
	bme280.delay_msec = BME280_delay_msek;

/*--------------------------------------------------------------------------*
 *  Drop the chip select pin to low - this tells the BME280 to
 *  use SPI mode. It will not respond to I2C commands until you
 *  reset the power to it.
 */
	HAL_GPIO_WritePin(GPIOB, SPI2_CS1_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, SPI2_CS1_Pin, GPIO_PIN_SET);

	return BME280_INIT_VALUE;
}

/************** I2C/SPI buffer length ******/

#define	I2C_BUFFER_LEN 28
#define SPI_BUFFER_LEN 28

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bme280.h file
*-----------------------------------------------------------------------*/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    HAL_StatusTypeDef status = HAL_OK;
	s32 iError = BME280_INIT_VALUE;

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint8_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(&hi2c1,						// i2c handle
    						  (uint8_t)(dev_addr<<1),		// i2c address, left aligned
							  (uint8_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(&reg_data),		// write returned data to reg_data
							  cnt,							// write how many bytes
							  100);							// timeout

	if (status != HAL_OK)
    {
        // The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	iError = (-1);
    }
	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data byte of to be read
 */
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    HAL_StatusTypeDef status = HAL_OK;
	s32 iError = BME280_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BME280_INIT_VALUE};
	u8 stringpos = BME280_INIT_VALUE;
	array[BME280_INIT_VALUE] = reg_addr;

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Read(&hi2c1,						// i2c handle
    						  (uint8_t)(dev_addr<<1),		// i2c address, left aligned
							  (uint8_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(&array),			// write returned data to this variable
							  cnt,							// how many bytes to expect returned
							  100);							// timeout

    if (status != HAL_OK)
    {
    	// The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	iError = (-1);
    }
	for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}

	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BME280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	HAL_StatusTypeDef status = HAL_OK;
	s32 iError=BME280_INIT_VALUE;
	u8 txarray[SPI_BUFFER_LEN]={0,};
	u8 rxarray[SPI_BUFFER_LEN] = {0,};
	u8 stringpos;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as BME280_INIT_VALUE)*/
	txarray[BME280_INIT_VALUE] = reg_addr|SPI_READ;/*read routine is initiated register address is mask with 0x80*/

	HAL_GPIO_WritePin( GPIOB, SPI2_CS1_Pin, GPIO_PIN_RESET );
	status = HAL_SPI_TransmitReceive( &hspi2, (uint8_t *)(&txarray), (uint8_t *)(&rxarray), cnt+1, 5);
	while( hspi2.State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin( GPIOB, SPI2_CS1_Pin, GPIO_PIN_SET );
	HAL_Delay(5); // since i can't find a buffer flushing command
	for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = rxarray[stringpos+BME280_DATA_INDEX];
	}

    if (status != HAL_OK)
    {
    	// The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	iError = (-1);
    }
	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BME280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	HAL_StatusTypeDef status = HAL_OK;
	s32 iError = BME280_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN * BME280_ADDRESS_INDEX];
	u8 stringpos = BME280_INIT_VALUE;
	u8 index = BME280_INIT_VALUE;
	for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done as per the
		SPI communication protocol specified in the data sheet*/
		index = stringpos * BME280_ADDRESS_INDEX;
		array[index] = (reg_addr++) & SPI_WRITE;
		//printf("%X ", array[stringpos]);
		array[index + BME280_DATA_INDEX] = *(reg_data + stringpos);
	}

	HAL_GPIO_WritePin( GPIOB, SPI2_CS1_Pin, GPIO_PIN_RESET );
	status = HAL_SPI_Transmit( &hspi2, (uint8_t*)(&array), cnt*2, 100);
	while( hspi2.State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin( GPIOB, SPI2_CS1_Pin, GPIO_PIN_SET );

    if (status != HAL_OK)
    {
    	// The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	iError = (-1);
    }
	return (s8)iError;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BME280_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	HAL_Delay(10*msek);
}
#endif
