/*
 * WF282.h
 *
 *  Created on: 2020年8月4日
 *      Author: ITry
 */

#ifndef INC_WF282_H_
#define INC_WF282_H_


void WF282_Init(void);

uint8_t WF282_Init_Calib_Param(void);

uint8_t WF282_Read_ID(uint8_t *id);

uint8_t WF282_Get_Data_Press_Temp(uint8_t data_num, double *readData);


#endif /* INC_WF282_H_ */
