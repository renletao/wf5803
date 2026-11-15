//
// Created by WFH-ITRY on 2026/10/13.
//

#ifndef OSBURNFINALTEST_G8U6_OSCHECK_H
#define OSBURNFINALTEST_G8U6_OSCHECK_H
#include <stdint.h>

#include "../../Inc/main.h"

void ADC_MspInit();

void Get_ADC_Value(uint32_t *value);

HAL_StatusTypeDef ADC_DMA_Read_Block_NDTR(uint16_t *dst, uint8_t num, uint32_t timeout_ms);
#endif //OSBURNFINALTEST_G8U6_OSCHECK_H
