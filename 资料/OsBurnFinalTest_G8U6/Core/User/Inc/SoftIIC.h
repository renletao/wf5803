/*
 * SoftIIC.h
 *
 *  Created on: 2020年8月4日
 *      Author: ITry
 */

#ifndef INC_SOFTIIC_H_
#define INC_SOFTIIC_H_
#include "../../Inc/main.h"

#define IIC_WRITE 	0
#define IIC_READ 	1

typedef struct structI2CPin {
    GPIO_TypeDef *portSda;
    GPIO_TypeDef *portScl;
    uint16_t pinSda;
    uint16_t pinScl;
} I2CPin;

uint8_t I2C_ReadNByte(I2CPin *info, uint8_t addr, uint8_t *readData, uint8_t length);

uint8_t I2C_WriteNByte(I2CPin *info, const uint8_t *reg_data, uint32_t length);

void IIC_Init(I2CPin *info);

void IIC_DeInit(I2CPin *info);

void iic_start(I2CPin *info);

void iic_stop(I2CPin *info);

void iic_ack(I2CPin *info);

void iic_nack(I2CPin *info);

void iic_sendbyte(I2CPin *info, uint8_t byte);

uint8_t iic_wait_ack(I2CPin *info);

uint8_t iic_readbyte(I2CPin *info, uint8_t ack);

void I2C_IDLE(I2CPin *info);
#endif /* INC_SOFTIIC_H_ */
