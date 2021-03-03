/*
 * max31856.h
 *
 *  Created on: Mar 2, 2021
 *      Author: Jayaram
 */

#include "stm32f7xx_hal.h"

#ifndef INC_MAX31856_H_
#define INC_MAX31856_H_

#define MAX31856_CR0_RREG           0x00    ///< Config 0 read register
#define MAX31856_CR0_WREG           0x80    ///< Config 0 write register
#define MAX31856_CR0_AUTOCONVERT   0x80    ///< Config 0 Auto convert flag
#define MAX31856_CR0_1SHOT         0x40    ///< Config 0 one shot convert flag
#define MAX31856_CR0_OCFAULT1      0x20    ///< Config 0 open circuit fault 1 flag
#define MAX31856_CR0_OCFAULT0      0x10    ///< Config 0 open circuit fault 0 flag
#define MAX31856_CR0_CJ            0x08    ///< Config 0 cold junction disable flag
#define MAX31856_CR0_FAULT         0x04    ///< Config 0 fault mode flag
#define MAX31856_CR0_FAULTCLR      0x02    ///< Config 0 fault clear flag

#define MAX31856_CR1_RREG           0x01    ///< Config 1 read register
#define MAX31856_CR1_WREG           0x81    ///< Config 1 write register
#define MAX31856_MASK_REG          0x02    ///< Fault Mask register
#define MAX31856_CJHF_REG          0x03    ///< Cold junction High temp fault register
#define MAX31856_CJLF_REG          0x04    ///< Cold junction Low temp fault register
#define MAX31856_LTHFTH_REG        0x05    ///< Linearized Temperature High Fault Threshold Register, MSB
#define MAX31856_LTHFTL_REG        0x06    ///< Linearized Temperature High Fault Threshold Register, LSB
#define MAX31856_LTLFTH_REG        0x07    ///< Linearized Temperature Low Fault Threshold Register, MSB
#define MAX31856_LTLFTL_REG        0x08    ///< Linearized Temperature Low Fault Threshold Register, LSB
#define MAX31856_CJTO_REG          0x09    ///< Cold-Junction Temperature Offset Register
#define MAX31856_CJTH_REG          0x0A    ///< Cold-Junction Temperature Register, MSB
#define MAX31856_CJTL_REG          0x0B    ///< Cold-Junction Temperature Register, LSB
#define MAX31856_LTCBH_REG         0x0C    ///< Linearized TC Temperature, Byte 2
#define MAX31856_LTCBM_REG         0x0D    ///< Linearized TC Temperature, Byte 1
#define MAX31856_LTCBL_REG         0x0E    ///< Linearized TC Temperature, Byte 0
#define MAX31856_SR_REG            0x0F    ///< Fault Status Register

#define ON 1
#define OFF 0

#define MAX31856_FAULT_CJRANGE     0x80    ///< Fault status Cold Junction Out-of-Range flag
#define MAX31856_FAULT_TCRANGE     0x40    ///< Fault status Thermocouple Out-of-Range flag
#define MAX31856_FAULT_CJHIGH      0x20    ///< Fault status Cold-Junction High Fault flag
#define MAX31856_FAULT_CJLOW       0x10    ///< Fault status Cold-Junction Low Fault flag
#define MAX31856_FAULT_TCHIGH      0x08    ///< Fault status Thermocouple Temperature High Fault flag
#define MAX31856_FAULT_TCLOW       0x04    ///< Fault status Thermocouple Temperature Low Fault flag
#define MAX31856_FAULT_OVUV        0x02    ///< Fault status Overvoltage or Undervoltage Input Fault flag
#define MAX31856_FAULT_OPEN        0x01    ///< Fault status Thermocouple Open-Circuit Fault flag

/** Noise filtering options enum. Use with setNoiseFilter() */
typedef enum {
MAX31856_NOISE_FILTER_50HZ,
MAX31856_NOISE_FILTER_60HZ
} max31856_noise_filter_t;

/** Multiple types of thermocouples supported */
typedef enum
{
  MAX31856_TCTYPE_B  = 0x00,//0b0000,
  MAX31856_TCTYPE_E  = 0x01,//0b0001,
  MAX31856_TCTYPE_J  = 0x02,//0b0010,
  MAX31856_TCTYPE_K  = 0x03,//0b0011,
  MAX31856_TCTYPE_N  = 0x04,//0b0100,
  MAX31856_TCTYPE_R  = 0x05,//0b0101,
  MAX31856_TCTYPE_S  = 0x06,//0b0110,
  MAX31856_TCTYPE_T  = 0x07,//0b0111,
  MAX31856_VMODE_G8  = 0x08,//0b1000,
  MAX31856_VMODE_G32 = 0x0C,//0b1100,
} max31856_thermocoupletype_t;

void write_max31856(uint8_t addr, uint8_t dt);
int read_max31856(uint8_t , uint8_t);
float MAX31865_readTemp();
float MAX31865_readCJ();
void set_thermocouple(uint8_t type);



#endif /* INC_MAX31856_H_ */
