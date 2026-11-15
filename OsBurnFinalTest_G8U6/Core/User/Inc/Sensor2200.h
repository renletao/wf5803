//
// Created by WFH-ITRY on 2026/10/20.
//

#ifndef OSBURNFINALTEST_G8U6_SENSOR2200_H
#define OSBURNFINALTEST_G8U6_SENSOR2200_H

#include "../../Inc/main.h"

void Sensor2200_Init(void);

void Get2200FinalTestData(uint16_t *dst, uint8_t cmpMode);

uint8_t Set2200Offset(uint8_t offset);

#endif //OSBURNFINALTEST_G8U6_SENSOR2200_H
