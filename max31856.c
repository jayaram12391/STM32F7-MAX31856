/*
 * max31856.c
 *
 *  Created on: Mar 2, 2021
 *      Author: Jayaram Subramanian
 *      SPI Library
 *
 *      Update:
 *      3/2/21 - Finished write_max31856(), read_max31856(), MAX31865_readTemp(),
 *      Thermocouple type, readCJ()
 *
 *      To Do:
 *      Need to write conditions for other functions
 *      One shot
 *      auto convert
 *      Faults
 *       etc,. Check other libraries in mBed
 */


#include "MAX31856.h"
extern SPI_HandleTypeDef hspi1;
uint8_t TxBuffer[2] = {0,0};
uint8_t RxBuffer[20];
uint8_t data;

//------------------------------------------------
#define cs_reset() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
#define cs_set() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
#define cs_strobe() cs_reset(); cs_set();
#define TIMEOUTVAL 250

void write_max31856(uint8_t addr, uint8_t dt)
{
	TxBuffer[0] = addr;
	TxBuffer[1] = dt;
	cs_strobe();
	HAL_SPI_Transmit(&hspi1, &TxBuffer[0],1,TIMEOUTVAL);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi1, &TxBuffer[1],1,TIMEOUTVAL);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	cs_reset();
}


int read_max31856(uint8_t addr, uint8_t len)
{
	RxBuffer[0] = 0;
	uint8_t status1;
	cs_set();
	HAL_SPI_Transmit(&hspi1, &addr,len,TIMEOUTVAL);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi1, (uint8_t*)RxBuffer,len,TIMEOUTVAL);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	cs_reset();
	status1 = RxBuffer[0];
	return status1;
}

float MAX31865_readTemp()
{
	uint8_t temp_reg[3], TxBuffer[2];
	int temp;
	float val;

	TxBuffer[0] = MAX31856_CR0_WREG;
	TxBuffer[1] = MAX31856_CR0_AUTOCONVERT;
	write_max31856(TxBuffer[0], TxBuffer[1]);

	temp_reg[0] = read_max31856(MAX31856_LTCBH_REG, 1);
	temp_reg[1] = read_max31856(MAX31856_LTCBM_REG, 1);
	temp_reg[2] = read_max31856(MAX31856_LTCBL_REG, 1);

	// Temperature calculation
    //Convert the registers contents into the correct value
    temp =((temp_reg[0] & 0xFF) << 11);       //Shift Byte 2 into place
    temp|=((temp_reg[1] & 0xFF) << 3);        //Shift Byte 1 into place
    temp|=((temp_reg[2] & 0xFF) >> 5);        //Shift Byte 0 into place
    val=(temp/128.0f);                  //Divide the binary string by 2 to the 7th power
    return val;
}

float MAX31865_readCJ()
{
    int32_t temp;
    uint8_t buf_read[3], buf_write[2]={MAX31856_CJTH_REG, MAX31856_CJTL_REG};

    buf_read[0] = read_max31856(buf_write[0], 1);
    buf_read[1] = read_max31856(buf_write[1], 1);

    //Convert the registers contents into the correct value
    temp =((int32_t)(buf_read[0] << 6));        //Shift the MSB into place
    temp|=((int32_t)(buf_read[1] >> 2));        //Shift the LSB into place
    float val=((float)(temp/64.0));             //Divide the binary string by 2 to the 6th power

    return val;
}

void set_thermocouple(uint8_t type)
{
	uint8_t status, read_addr, write_addr, clear_bits;
	read_addr = MAX31856_CR1_RREG;
	write_addr = MAX31856_CR1_WREG;
	status = read_max31856(read_addr, 1);
	clear_bits = 0xF0;
	status = status & clear_bits;
	status = status | type;
	write_max31856(write_addr, status);
}
