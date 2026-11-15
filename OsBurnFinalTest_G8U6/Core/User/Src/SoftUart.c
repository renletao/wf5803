/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "../Inc/SoftUart.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
/* USER CODE END 0 */


/* USER CODE BEGIN 1 */
UartInfo uart1;
/* USER CODE END 1 */

/**
 *IO��ʼ��
 */
void UART_MspInit(GPIO_TypeDef *Uart_Tx_Port, uint16_t Uart_Tx_Pin, GPIO_TypeDef *Uart_Rx_Port, uint16_t Uart_Rx_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(Uart_Tx_Port, Uart_Tx_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = Uart_Tx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Uart_Tx_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Uart_Rx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(Uart_Rx_Port, &GPIO_InitStruct);
}

/**
 *IOȡ����ʼ��
 */
void UART_DeMspInit(GPIO_TypeDef *Uart_Tx_Port, uint16_t Uart_Tx_Pin, GPIO_TypeDef *Uart_Rx_Port,
                    uint16_t Uart_Rx_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(Uart_Tx_Port, Uart_Tx_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = Uart_Tx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Uart_Tx_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Uart_Rx_Pin;
    HAL_GPIO_Init(Uart_Rx_Port, &GPIO_InitStruct);
}

/**
 *uart��ʼ��
 */
void UART_Init(UartInfo *uart, uint32_t buadrate, GPIO_TypeDef *Uart_Tx_Port, uint16_t Uart_Tx_Pin,
               GPIO_TypeDef *Uart_Rx_Port, uint16_t Uart_Rx_Pin) {
    uart->Tx_Port = Uart_Tx_Port;
    uart->Rx_Port = Uart_Rx_Port;
    uart->Tx_Pin = Uart_Tx_Pin;
    uart->Rx_Pin = Uart_Rx_Pin;
    uart->BuadRate = 1000000 / buadrate;
    UART_MspInit(uart->Tx_Port, uart->Tx_Pin, uart->Rx_Port, uart->Rx_Pin);
}

/**
 *uartȡ����ʼ��
 */
void UART_DeInit(UartInfo *uart) {
    UART_DeMspInit(uart->Tx_Port, uart->Tx_Pin, uart->Rx_Port, uart->Rx_Pin);
    uart->Tx_Port = NULL;
    uart->Rx_Port = NULL;
    uart->Tx_Pin = 0;
    uart->Rx_Pin = 0;
    uart->BuadRate = 0;
}

/**
 *uart����1byte����
 */
void Uart_Send_Bit(UartInfo *uart, uint8_t Data) {
    uart->Tx_Port->BRR = (uint32_t) uart->Tx_Pin;
    delay_us(uart->BuadRate);
    for (uint8_t i = 0; i < 8; i++) {
        if (Data & 0x01) uart->Tx_Port->BSRR = (uint32_t) uart->Tx_Pin;
        else uart->Tx_Port->BRR = (uint32_t) uart->Tx_Pin;
        delay_us(uart->BuadRate);
        Data = Data >> 1;
    }
    uart->Tx_Port->BSRR = (uint32_t) uart->Tx_Pin;
    delay_us(uart->BuadRate);
}

/**
 *�ȴ���Ҫ��ƽ
 */
uint16_t Wiat_For_Level(UartInfo *uart, uint8_t level, uint32_t timout) {
    uint32_t Count = 0;
    HAL_TIM_Base_Start(&htim2);
    __HAL_TIM_SetCounter(&htim2, 0);
    while (HAL_GPIO_ReadPin(uart->Rx_Port, uart->Rx_Pin) != level) {
        Count++;
        if (__HAL_TIM_GET_COUNTER(&htim2) >= timout)break;
    }
    HAL_TIM_Base_Stop(&htim2);
    return Count;
}

/**
 *��ȡ��ε�ƽ
 */
uint16_t Read_Level(UartInfo *uart, uint32_t timout) {
    uint8_t level = 0;
    uint32_t Count = 0;
    HAL_TIM_Base_Start(&htim2);
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) <= timout) {
        Count++;
        if (HAL_GPIO_ReadPin(uart->Rx_Port, uart->Rx_Pin) == GPIO_PIN_SET)level++;
    }
    if (level >= Count / 2)return 1;
    else return 0;
}

/**
 *uart��ȡ1byte����
 */
uint8_t Soft_Uart_Read_Bit(UartInfo *uart) {
    uint8_t data = 0;
    HAL_TIM_Base_Start(&htim2);
    Wiat_For_Level(uart, GPIO_PIN_RESET, 65535); //�ȴ��͵�ƽ��ʼ
    __HAL_TIM_SetCounter(&htim2, 0);
    delay_us(uart->BuadRate - 10); //������ʼλ
    for (uint8_t i = 0; i < 8; i++) {
        data >>= 1;
        if (Read_Level(uart, uart->BuadRate) == 1) {
            if (uart->Save_BuadRate == 0) {
                uart->Save_BuadRate = __HAL_TIM_GET_COUNTER(&htim2) / i + 1;
                HAL_TIM_Base_Stop(&htim2);
            }
            data |= 0x80;
        }
    }
    Wiat_For_Level(uart, GPIO_PIN_SET, 65535); //�ȴ��ӻ��ͷ�
    return data;
}

/**
 *uart���Ͷ��ֽ�
 */
void Soft_Uart_Send(UartInfo *uart, uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        Uart_Send_Bit(uart, buf[i]);
        delay_us(uart->BuadRate);
    }
}

/**
 *uart��ȡ���ֽ�
 */
void Soft_Uart_Recv(UartInfo *uart, uint8_t *buf, uint16_t len) {
    uart->Save_BuadRate = 0;
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = Soft_Uart_Read_Bit(uart);
    }
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
