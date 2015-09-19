/*
 * Unlimited_BMU_Shunt.h
 *
 *  Created on: Sep 10, 2015
 *      Author: Paul
 */

#ifndef UNLIMITED_BMU_SHUNT_H_
#define UNLIMITED_BMU_SHUNT_H_

#define IIR_GAIN_ELECTRICAL 1000

#define V_DIVIDER 12.2
#define IR_DIVIDER
#define IF_DIVIDER

#define STATUS_ON     LPC_GPIO0->FIOSET |= (1<<3);
#define STATUS_OFF    LPC_GPIO0->FIOCLR |= (1<<3);

#define PORT_USED 1

/// EEPROM Addresses ///
#define AddressBMUWHR   0
#define AddressBMUWHRI  4
#define AddressBMUWHRO  8

uint32_t EE_Read(uint16_t _EEadd);
uint32_t EE_Seq_Read(uint16_t _EEadd, int _len);
void EE_Write(uint16_t _EEadd, uint32_t data);
uint32_t I2C_Read(uint16_t _EEadd);
void I2C_Seq_Read(uint16_t _EEadd, int read_len);
void I2C_Write(uint16_t _EEadd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3);
void shunt_read(void);
void load_nonpersistent (void);
void load_persistent (void);
void store_persistent (void);
void init_GPIO (void);
uint32_t  iir_filter_uint     (uint32_t _data_in, uint32_t _cur_data, uint16_t _gain);
int32_t   iir_filter_int      (int32_t _data_in, int32_t _cur_data, uint16_t _gain);
float iir_filter_float (float _data_in, float _cur_data, uint16_t _gain);

#endif /* UNLIMITED_BMU_SHUNT_H_ */
