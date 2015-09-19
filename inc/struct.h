/*
 * struct.h
 *
 *  Created on: 29 May 2015
 *      Author: Stuff
 */

#ifndef STRUCT_H_
#define STRUCT_H_

struct CLOCK_STRUCT
{
	uint8_t		T_mS; // (mS * 100)
	uint8_t		T_S;
	uint8_t		T_M;
	uint8_t		T_H;
	uint32_t	T_D;
}CLOCK;

struct BMU_BUS_DATA
{
	float BusV;
	float BusI;
	float Watts;
	float WattHrsIn;
	float WattHrsOut;
	float WattHrs;
}BMU_DATA;

#endif /* STRUCT_H_ */
