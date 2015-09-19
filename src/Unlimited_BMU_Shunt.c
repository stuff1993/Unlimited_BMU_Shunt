/*
===============================================================================
 Name        : Unlimited_BMU_Shunt.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

#include "type.h"
#include "inttofloat.h"
#include "adc.h"
#include "i2c.h"
#include "can.h"
#include "timer.h"
#include "struct.h"
#include "Unlimited_BMU_Shunt.h"

CAN_MSG MsgBuf_TX1, MsgBuf_RX1;

extern volatile uint8_t I2CMasterBuffer[I2C_PORT_NUM][BUFSIZE];
extern volatile uint32_t I2CWriteLength[I2C_PORT_NUM];
extern volatile uint32_t I2CReadLength[I2C_PORT_NUM];
extern volatile uint8_t I2CSlaveBuffer[I2C_PORT_NUM][BUFSIZE];

/******************************************************************************
** Function name:		SysTick_Handler
**
** Description:			System clock event handler. Fires every 100mS
**
** Parameters:			None
** Returned value:	None
**
******************************************************************************/
void SysTick_Handler (void)
{
	CLOCK.T_mS++;

	/// Time sensitive calculations
	BMU_DATA.WattHrs += BMU_DATA.Watts / 36000.0;
	if(BMU_DATA.Watts > 0)
	{
	  BMU_DATA.WattHrsOut += BMU_DATA.Watts / 36000.0;
	}
	else
	{
    BMU_DATA.WattHrsIn -= BMU_DATA.Watts / 36000.0;
  }

	/// CAN Tx
	if((LPC_CAN1->SR & 0x00040404) == 0x00040404){LPC_CAN1->CMR = 0x22;} // Abort from buffer 1

	MsgBuf_TX1.Frame = 0x00080000;
  MsgBuf_TX1.MsgID = BMU_SHUNT;
  MsgBuf_TX1.DataA = BMU_DATA.BusV;
  MsgBuf_TX1.DataB = BMU_DATA.BusI;
  CAN1_SendMessage( &MsgBuf_TX1 );

  if((LPC_CAN1->SR & 0x00040404) == 0x00040404){LPC_CAN1->CMR = 0x42;} // Abort from buffer 2

  MsgBuf_TX1.Frame = 0x00080000;
  MsgBuf_TX1.MsgID = BMU_SHUNT + 1;
  MsgBuf_TX1.DataA = conv_float_uint(BMU_DATA.WattHrsOut);
  MsgBuf_TX1.DataB = conv_float_uint(BMU_DATA.WattHrsIn);
  CAN1_SendMessage( &MsgBuf_TX1 );

  if((LPC_CAN1->SR & 0x00040404) == 0x00040404){LPC_CAN1->CMR = 0x82;} // Abort from buffer 3

  MsgBuf_TX1.Frame = 0x00080000;
  MsgBuf_TX1.MsgID = BMU_SHUNT + 2;
  MsgBuf_TX1.DataA = conv_float_uint(BMU_DATA.WattHrs);
  MsgBuf_TX1.DataB = 0x0;
  CAN1_SendMessage( &MsgBuf_TX1 );

	if(CLOCK.T_mS >= 10) // Calculate time
	{
		CLOCK.T_mS = 0;CLOCK.T_S++;

		STATUS_ON;
		store_persistent(); // Store data in eeprom every second
		STATUS_OFF;

		if(CLOCK.T_S >= 60){CLOCK.T_S = 0;CLOCK.T_M++;
		if(CLOCK.T_M >= 60){CLOCK.T_M = 0;CLOCK.T_H++;
		if(CLOCK.T_H >= 24){CLOCK.T_H = 0;CLOCK.T_D++;}}}
	}
}

/******************************************************************************
** Function name:		shunt_read
**
** Description:			Reads ADC values for V and I
**
** Parameters:			None
** Returned value:	None
**
******************************************************************************/
void shunt_read (void)
{
  float ADC_B, ADC_C;

	BMU_DATA.BusV = iir_filter_float((float)(ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0))/(8.0 * V_DIVIDER), BMU_DATA.BusV, IIR_GAIN_ELECTRICAL);
	ADC_B = (float)(ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1))/8.0;
	ADC_C = -((float)(ADCRead(2) + ADCRead(2) + ADCRead(2) + ADCRead(2) + ADCRead(2) + ADCRead(2) + ADCRead(2) + ADCRead(2))/8.0);

	if(ADC_B > ADC_C)
	{
		BMU_DATA.BusI = iir_filter_float(ADC_B, BMU_DATA.BusI, IIR_GAIN_ELECTRICAL);
	}
	else

	{
		BMU_DATA.BusI = iir_filter_float(ADC_C, BMU_DATA.BusI, IIR_GAIN_ELECTRICAL);
	}

	BMU_DATA.Watts = BMU_DATA.BusI * BMU_DATA.BusV;
}

/******************************************************************************
** Function name:   EE_Read
**
** Description:     Reads a word from EEPROM (Uses I2CRead)
**
** Parameters:      Address to read from
** Returned value:    Data at address
**
******************************************************************************/
uint32_t EE_Read (uint16_t _EEadd)
{
  uint32_t retDATA = 0;

  retDATA = I2C_Read(_EEadd+3);
  retDATA = (retDATA << 8) + I2C_Read(_EEadd+2);
  retDATA = (retDATA << 8) + I2C_Read(_EEadd+1);
  retDATA = (retDATA << 8) + I2C_Read(_EEadd+0);

  return retDATA;
}

/******************************************************************************
** Function name:   EE_Seq_Read
**
** Description:     Reads a word from EEPROM (Uses I2CRead)
**
** Parameters:      1. Address to read from
**            2. Byte length of read
** Returned value:    Data at address
**
******************************************************************************/
uint32_t EE_Seq_Read (uint16_t _EEadd, int _len)
{
  uint32_t _ret = 0;

  I2C_Seq_Read(_EEadd, _len);
  while(_len--)
  {
    _ret += I2CSlaveBuffer[PORT_USED][_len] << (_len * 8);
  }

  return _ret;
}

/******************************************************************************
** Function name:   EE_Write
**
** Description:     Saves a word to EEPROM (Uses I2CWrite)
**
** Parameters:      1. Address to save to
**            2. Data to save (convert to uint with converter first)
** Returned value:    None
**
******************************************************************************/
void EE_Write (uint16_t _EEadd, uint32_t _EEdata)
{
  uint8_t temp0 = (_EEdata & 0x000000FF);
  uint8_t temp1 = (_EEdata & 0x0000FF00) >> 8;
  uint8_t temp2 = (_EEdata & 0x00FF0000) >> 16;
  uint8_t temp3 = (_EEdata & 0xFF000000) >> 24;

  I2C_Write(_EEadd, temp0,temp1,temp2,temp3);
}

/******************************************************************************
** Function name:   I2C_Read
**
** Description:     Reads a byte from EEPROM
**
** Parameters:      Address to read from
** Returned value:    Data at address
**
******************************************************************************/
uint32_t I2C_Read (uint16_t _EEadd)
{
  int i;

  for ( i = 0; i < BUFSIZE; i++ )     // clear buffer
  {
    I2CMasterBuffer[PORT_USED][i] = 0;
  }

  I2CWriteLength[PORT_USED] = 3;
  I2CReadLength[PORT_USED] = 1;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = 0x00;   // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd;   // address
  I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
  I2CEngine( PORT_USED );
  I2CStop(PORT_USED);

  return (uint32_t)I2CSlaveBuffer[PORT_USED][0];
}

/******************************************************************************
** Function name:   I2C_Seq_Read
**
** Description:     Reads a byte from EEPROM
**
** Parameters:      1. Address to read from
**            2. Byte length of read
** Returned value:    None
**
******************************************************************************/
void I2C_Seq_Read (uint16_t _EEadd, int read_len)
{
  int i;
  for ( i = 0; i < BUFSIZE; i++ )     // clear buffer
  {
    I2CSlaveBuffer[PORT_USED][i] = 0;
  }

  I2CWriteLength[PORT_USED] = 3;
  I2CReadLength[PORT_USED] = read_len;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = (_EEadd & 0x0f00) >> 8; // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd & 0x00ff;    // address
  I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
  I2CEngine( PORT_USED );
  I2CStop(PORT_USED);
}

/******************************************************************************
** Function name:   I2C_Write
**
** Description:     Saves a word to EEPROM
**
** Parameters:      1. Address to save to
**            2. Data to save
** Returned value:    None
**
******************************************************************************/
void I2C_Write (uint16_t _EEadd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
  I2CWriteLength[PORT_USED] = 7;
  I2CReadLength[PORT_USED] = 0;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = 0x00;   // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd;   // address
  I2CMasterBuffer[PORT_USED][3] = data0;
  I2CMasterBuffer[PORT_USED][4] = data1;
  I2CMasterBuffer[PORT_USED][5] = data2;
  I2CMasterBuffer[PORT_USED][6] = data3;
  I2CEngine( PORT_USED );

  delayMs(1,2);
}

/******************************************************************************
** Function name:   store_persistent
**
** Description:     Saves persistent variables to EEPROM
**
** Parameters:      None
** Returned value:    None
**
******************************************************************************/
void store_persistent (void)
{
  EE_Write(AddressBMUWHR, conv_float_uint(BMU_DATA.WattHrs));
  EE_Write(AddressBMUWHRI, conv_float_uint(BMU_DATA.WattHrsIn));
  EE_Write(AddressBMUWHRO, conv_float_uint(BMU_DATA.WattHrsOut));
}

/******************************************************************************
** Function name:		load_nonpersistent
**
** Description:			Configures pins to be used for GPIO
**
** Parameters:			None
** Returned value:	None
**
******************************************************************************/
void load_nonpersistent (void)
{
	BMU_DATA.BusI = 0;
	BMU_DATA.BusV = 0;
	BMU_DATA.Watts = 0;

	CLOCK.T_mS = 0;
	CLOCK.T_S = 0;
	CLOCK.T_M = 0;
	CLOCK.T_H = 0;
	CLOCK.T_D = 0;
}

/******************************************************************************
** Function name:		init_GPIO
**
** Description:			Configures pins to be used for GPIO
**
** Parameters:			None
** Returned value:	None
**
******************************************************************************/
void init_GPIO (void)
{
	/*
	 * GPIO0:
	 * 	PINSEL0:
	 * 		3 - OUT - Status LED
	 */
	LPC_GPIO0->FIODIR = (1<<3);
	LPC_GPIO0->FIOCLR = (1<<3);
}

/******************************************************************************
 ** Function name:  iir_filter_uint
 **
 ** Description:    Filter to flatten out erratic data reads
 **
 ** Parameters:     1. Input data
 **                 2. Existing data
 **                 3. Gain factor
 ** Returned value: Smoothed value
 **
 ******************************************************************************/
uint32_t iir_filter_uint (uint32_t _data_in, uint32_t _cur_data, uint16_t _gain)
{return (((_gain-1)*_cur_data)+_data_in)/_gain;}

/******************************************************************************
 ** Function name:  iir_filter_int
 **
 ** Description:    Filter to flatten out erratic data reads
 **
 ** Parameters:     1. Input data
 **                 2. Existing data
 **                 3. Gain factor
 ** Returned value: Smoothed value
 **
 ******************************************************************************/
int32_t iir_filter_int (int32_t _data_in, int32_t _cur_data, uint16_t _gain)
{return (((_gain-1)*_cur_data)+_data_in)/_gain;}

/******************************************************************************
 ** Function name:  iir_filter_float
 **
 ** Description:    Filter to flatten out erratic data reads
 **
 ** Parameters:     1. Input data
 **                 2. Existing data
 **                 3. Gain factor
 ** Returned value: Smoothed value
 **
 ******************************************************************************/
float iir_filter_float (float _data_in, float _cur_data, uint16_t _gain)
{return (((_gain-1)*_cur_data)+_data_in)/_gain;}

/******************************************************************************
** Function name:   main
**
** Description:     Program entry point. Contains initializations and menu loop
**
** Parameters:      None
** Returned value:  Program exit value
**
******************************************************************************/
int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();

	setCANBUS1();

	load_nonpersistent();

	SysTick_Config(SystemCoreClock / 10);	// 100mS Systicker.

	I2C1Init();

	ADCInit(ADC_CLK);

	init_GPIO();

	while(1){shunt_read();}

	return 0;
}
