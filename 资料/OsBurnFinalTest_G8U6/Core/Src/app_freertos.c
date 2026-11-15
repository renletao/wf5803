/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
#include "mb.h"
#include "mbutils.h"
#include <string.h>
#include "../User/Inc/CmdHelper.h"
#include "flash.h"
#include "../User/Inc/SoftIIC.h"
#include "tim.h"
#include "iwdg.h"
#include "MCP4725.h"
#include "sensor2200.h"
#include "WF282.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SLAVE_ADDRESS 1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REG_INPUT_START 30001
#define SENDBUF_LENS 120
#define REG_HOLDING_START 40001
#define RECVBUF_LENS 64

#define REG_INPUT2_START 20001
//上位机下发的指令缓存
uint16_t recvBuf[RECVBUF_LENS];
//运行缓存，提供给上位机读取
uint16_t sendBuf[SENDBUF_LENS]; //常规指令
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ModbusTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, ModbusTask, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* definition and creation of myTask02 */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    uint8_t i = 0;
    /* Infinite loop */
    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);
        if (i == 1)HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        else if (i == 6) {
            i = 0;
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        i++;
        osDelay(500);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ModbusTask */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModbusTask */
void ModbusTask(void const * argument)
{
  /* USER CODE BEGIN ModbusTask */
    //放这里是因为它们都用到了osDelay，必须等rtos初始化完成才能进行
    MCP4725_Init();
    Sensor2200_Init();
    //WF282_Init();
    //WF282_Init_Calib_Param();
    /* Infinite loop */
    eMBInit(MB_RTU, SLAVE_ADDRESS, SLAVE_ADDRESS, 9600, MB_PAR_NONE);
    eMBEnable();
    for (;;) {
        eMBPoll();

        osDelay(1);
    }
  /* USER CODE END ModbusTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs) {
	eMBErrorCode eStatus = MB_ENOERR;
	int i = 0;
	if ((usAddress/* >=*/== REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + SENDBUF_LENS)) {
		i = (int) (usAddress - REG_INPUT_START);
		while (usNRegs > 0) {
			*pucRegBuffer++ = (uint8_t) (sendBuf[i] >> 8);
			*pucRegBuffer++ = (uint8_t) (sendBuf[i] & 0xFF);
			i++;
			usNRegs--;
		}
	}
	/*else if ((usAddress >= REG_INPUT2_START) && (usAddress + usNRegs <= REG_INPUT2_START + DATABUF_LENS)) {
	 i = (int) (usAddress - REG_INPUT2_START);
	 while (usNRegs > 0) {
	 *pucRegBuffer++ = (uint8_t) (dataBuf[i] >> 8);
	 *pucRegBuffer++ = (uint8_t) (dataBuf[i] & 0xFF);
	 i++;
	 usNRegs--;
	 }
	 } */else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode) {
	eMBErrorCode eStatus = MB_ENOERR;
	int i = 0;
	if ((usAddress /*>=*/== REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + RECVBUF_LENS)) {
		i = (int) (usAddress - REG_HOLDING_START);
		if (eMode == MB_REG_READ) {
			while (usNRegs > 0) {
				*pucRegBuffer++ = (UCHAR) (recvBuf[i] >> 8);
				*pucRegBuffer++ = (UCHAR) (recvBuf[i] & 0xFF);
				i++;
				usNRegs--;
			}
		} else if (eMode == MB_REG_WRITE) {
			while (usNRegs > 0) {
				recvBuf[i] = *pucRegBuffer++ << 8;
				recvBuf[i] |= *pucRegBuffer++;
				i++;
				usNRegs--;
			}
			memset(sendBuf, 0, sizeof(sendBuf));
			Parse(recvBuf, i, sendBuf, SENDBUF_LENS);
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode) {
	return MB_ENOERR;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete) {
	return MB_ENOERR;
}
/* USER CODE END Application */

