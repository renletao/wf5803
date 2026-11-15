//
// Created by WFH-ITRY on 2026/10/13.
//
#include "../Inc/OsCheck.h"

#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32g0xx_hal_gpio.h"
#include "../Inc/MCP4725.h"
#include "../Inc/sensor2200.h"

void ADC_MspInit() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    PA11 [PA9]     ------> ADC1_IN15
    PA12 [PA10]     ------> ADC1_IN16
    */
    GPIO_InitStruct.Pin = PGND_Pin | EX_TX_Pin | EX_RX_Pin | EX_VDD_Pin
                          | EX_VPP_Pin | EX_SCL_Pin | EX_SDA_Pin | VDD_FB_Pin
                          | VOLT_6V2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = EX_OWI_Pin | EX_SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static uint8_t adc_done_flag = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        adc_done_flag = 1;
    }
}

HAL_StatusTypeDef ADC_DMA_Read_Block_NDTR(uint16_t *dst, uint8_t num, uint32_t timeout_ms) {
    HAL_StatusTypeDef ret = HAL_OK;
    MCP4725_WriteData_Voltage(0);
    ADC_MspInit();
    SetIoStatus(1, 0, 0);
    osDelay(50);
    adc_done_flag = 0;
    uint32_t tick_start = HAL_GetTick();
    // 启动一次：扫描全部选中通道，DMA搬运一轮，完成后停止
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) dst, num) != HAL_OK) {
        ret = HAL_ERROR;
        goto exit_end;
    }
    while (adc_done_flag == 0) {
        if ((HAL_GetTick() - tick_start) >= timeout_ms) {
            HAL_ADC_Stop_DMA(&hadc1);
            ret = HAL_TIMEOUT;
            goto exit_end;
        }
    }
    // 正常采集完成，停止DMA
    HAL_ADC_Stop_DMA(&hadc1);
exit_end:
    SetIoStatus(0, 0, 0);
    Sensor2200_Init();
    return ret;
}
