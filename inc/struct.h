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
	float bus_v;
	float bus_i;
	float watts;
	float watt_hrs_in;
	float watt_hrs_out;
	float watt_hrs;
}bmu_data;

#endif /* STRUCT_H_ */
