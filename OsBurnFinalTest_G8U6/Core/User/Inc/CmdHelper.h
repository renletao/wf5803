/*
 * CmdHelper.h
 *
 *  Created on: Feb 17, 2022
 *      Author: ITry
 */

#ifndef INC_CMDHELPER_H_
#define INC_CMDHELPER_H_

#include "../../Inc/main.h"

#define TYPE1_GET_VERSION 					0x104
#define TYPE1_OS_CHECK                      0x205
#define TYPE1_2200_FT                       0x206
#define TYPE1_SET_VDD                       0x207

#define TYPE1_2200_OFFSET                   0x999

void Parse(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);

void ParseSystemCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);

void CopyAndClear(uint16_t *src, uint16_t *dest, size_t len);

void ParseI2CCmd(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);

void ParseSerialCmd(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);

void ParseProductCmd(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);
#endif /* INC_CMDHELPER_H_ */
