/*
 * SoftIIC.c
 *
 *  Created on: Aug 3, 2020
 *      Author: ITry
 */
#include "../Inc/SoftIIC.h"
#include <string.h>

///- 100kHz I2C：位周期 10us，SCL 高 5us，SCL 低 5us → `delay_us(5)`
///- 400kHz I2C：位周期 2.5us，SCL 高 2us，SCL 低 2us → `delay_us(2)`
#define IIC_SPEED 20

/**
 *IO初始化
 */
void IIC_MspInit(GPIO_TypeDef *SDA_PORT, uint16_t SDA_PIN, GPIO_TypeDef *SCL_PORT, uint16_t SCL_PIN) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
}

/**
 *IO取消初始化
 */
void IIC_DeMspInit(GPIO_TypeDef *SDA_PORT, uint16_t SDA_PIN, GPIO_TypeDef *SCL_PORT, uint16_t SCL_PIN) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);
}

/**
 *iic初始化
 */
void IIC_Init(I2CPin *info) {
    IIC_MspInit(info->portSda, info->pinSda, info->portScl, info->pinScl);
}

/**
 *iic取消初始化
 */
void IIC_DeInit(I2CPin *info) {
    IIC_DeMspInit(info->portSda, info->pinSda, info->portScl, info->pinScl);
    info->portSda = NULL;
    info->portScl = NULL;
    info->pinSda = 0;
    info->pinScl = 0;
}

/**
 *iic启动:当SCL处于高电平状态时，SDA出现一个下降沿,即产生IIC启动信号
 */
void iic_start(I2CPin *info) {
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_SET);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_RESET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_RESET);
}

/**
 *iic停止:当SCL处于高电平状态时，SDA出现一个上升沿,即产生IIC停止信号
 */
void iic_stop(I2CPin *info) {
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
}

/**
 *IIC等待应答信号
 *ack_status： 应答状态，0表示应答，1表示设备无响应
 */
uint8_t iic_wait_ack(I2CPin *info) {
    uint8_t err_time = 0;
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
    while ((HAL_GPIO_ReadPin(info->portSda, info->pinSda) == GPIO_PIN_SET)) {
        err_time++;
        delay_us(1);
        if (err_time > 250) {
            iic_stop(info);
            return 1;
        }
    }
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_RESET);
    delay_us(IIC_SPEED);
    return 0;
}

/**
 *主机（主控制器）产生应答信号
 */
void iic_ack(I2CPin *info) {
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_RESET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_RESET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_SET);
}

/**
 * 主机（主控制器）产生不应答信号
 */
void iic_nack(I2CPin *info) {
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_SET);
    delay_us(IIC_SPEED);
    HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_RESET);
    delay_us(IIC_SPEED);
}

/**
 *IIC发送1byte数据
 */
void iic_sendbyte(I2CPin *info, uint8_t byte) {
    uint8_t i;
    /** 发送一个字节的高7位 */
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_RESET);
        }
        byte <<= 1;
        delay_us(IIC_SPEED);
        HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_SET);
        delay_us(IIC_SPEED);
        HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_RESET);
        delay_us(IIC_SPEED);
    }
}

/**
 *IIC读取1byte数据
 */
uint8_t iic_readbyte(I2CPin *info, uint8_t ack) {
    uint8_t i;
    uint8_t recv_value = 0;
    HAL_GPIO_WritePin(info->portSda, info->pinSda, GPIO_PIN_SET);
    delay_us(20);
    for (i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_SET);
        delay_us(IIC_SPEED);
        recv_value <<= 1;
        if ((HAL_GPIO_ReadPin(info->portSda, info->pinSda) == GPIO_PIN_SET)) {
            recv_value++;
        }
        delay_us(IIC_SPEED);
        HAL_GPIO_WritePin(info->portScl, info->pinScl, GPIO_PIN_RESET);
        delay_us(IIC_SPEED);
    }
    if (!ack) {
        iic_nack(info);
    } else {
        iic_ack(info);
    }
    return recv_value;
}

/**
 *iic多字节读取
 */
uint8_t I2C_ReadNByte(I2CPin *info, const uint8_t addr, uint8_t *readData, const uint8_t length) {
    uint8_t i = 0;
    uint8_t err = 0;
    if (length > 0) memset(readData, 0, (length * sizeof(uint8_t)));
    else return err;
    iic_start(info);
    iic_sendbyte(info, addr | IIC_READ);
    if ((err=iic_wait_ack(info)))return err;
    for (i = 0; i < length; i++) {
        if (i != (length - 1)) {
            readData[i] = iic_readbyte(info, 1);
        } else {
            readData[i] = iic_readbyte(info, 0);
        }
    }
    iic_stop(info);
    return err;
}

/**
 *iic多字节发送
 */
uint8_t I2C_WriteNByte(I2CPin *info, const uint8_t *writeData, const uint32_t length) {
    uint8_t err = 0;
    iic_start(info);
    for (uint8_t i = 0; i < length; i++) {
        iic_sendbyte(info, writeData[i]);
        err = iic_wait_ack(info);
        if (err) break;
    }
    iic_stop(info);
    return err;
}

void I2C_IDLE(I2CPin *info) {
    IIC_DeInit(info);
}
