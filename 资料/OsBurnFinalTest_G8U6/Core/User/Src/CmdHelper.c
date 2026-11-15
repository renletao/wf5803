/*
 * CmdHelper.c
 *
 *  Created on: Feb 17, 2022
 *      Author: ITry
 */

#include <string.h>
#include "cmsis_os.h"
#include "../Inc/CmdHelper.h"
#include "../Inc/owi.h"
#include "../Inc/OsCheck.h"
#include "../Inc/Sensor2200.h"
#include "../Inc/MCP4725.h"

#define DATA_LENGTH                 64
//一类：系统指令
//二类：串口通讯方式指令
//三类: IIC通讯方式指令
//四类: SPI通讯方式指令

static const uint8_t FIRMWARE_VERSION[] = {0x01, 0x02};

void ParseSystemCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens) {
    uint16_t cmdHead = cmd[1];
    switch (cmdHead) {
        case TYPE1_GET_VERSION: //104:获取版本号
            out[0] = FIRMWARE_VERSION[0];
            out[1] = FIRMWARE_VERSION[1];
            break;
        case TYPE1_OS_CHECK:
            ADC_DMA_Read_Block_NDTR(out, 11, 200);
            break;
        case TYPE1_2200_FT:
            Get2200FinalTestData(out, cmd[2]);
            break;
        case TYPE1_2200_OFFSET:
            out[0] = Set2200Offset((uint8_t) cmd[2]);
            break;

        case TYPE1_SET_VDD:
        	MCP4725_WriteData_Voltage(cmd[2]);
        	break;

        default:
            break;
    }
}

void ParseSerialCmd(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens) {
    // uint16_t sendLens = cmd[2];
    // uint16_t recvLens = cmd[4];
    // uint8_t SendBuff[DATA_LENGTH] = {0};
    // uint8_t RecvBuff[DATA_LENGTH] = {0};
    // for (uint8_t i = 0; i < sendLens; i++) {
    //     SendBuff[i] = cmd[i + 6];
    // }
    // UART_Init(&uart1, 9600, UART_RX_GPIO_Port, UART_RX_Pin, UART_TX_GPIO_Port, UART_TX_Pin);
    // osDelay(10);
    // Soft_Uart_Send(&uart1, SendBuff, sendLens);
    // Soft_Uart_Recv(&uart1, RecvBuff, recvLens);
    // out[1] = (uint8_t) (1000000 / uart1.Save_BuadRate);
    // out[0] = (uint8_t) ((1000000 / uart1.Save_BuadRate) >> 8);
    // for (uint8_t i = 0; i < recvLens; i++) {
    //     out[i + 2] = RecvBuff[i];
    // }
    // UART_DeInit(&uart1);
}

void ParseI2CCmd(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens) {
    // uint16_t sendLens = cmd[2];
    // uint16_t recvLens = cmd[4];
    // uint8_t SendBuff[DATA_LENGTH] = {0};
    // uint8_t RecvBuff[DATA_LENGTH] = {0};
    // for (uint8_t i = 0; i < sendLens; i++) {
    //     SendBuff[i] = cmd[i + 6];
    // }
    // IIC_Init(&i2c1, I2C_SDA_GPIO_Port, I2C_SDA_Pin, I2C_SCL_GPIO_Port, I2C_SCL_Pin);
    // osDelay(10);
    // I2C_WriteNByte(&i2c1, SendBuff, sendLens);
    // I2C_ReadNByte(&i2c1, SendBuff[0], RecvBuff, recvLens);
    // for (uint8_t i = 0; i < recvLens; i++) {
    //     out[i] = RecvBuff[i];
    // }
    // IIC_DeInit(&i2c1);
}

void ParseOwiCmd(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens) {
    uint8_t RecvBuff[DATA_LENGTH] = {0};
    OWI_Device.cycle = 0;
    OWI(RecvBuff, 200);
    osDelay(100);
    OWI(RecvBuff, 200);
    out[0] = OWI_Device.cycle;
    out[1] = RecvBuff[0];
    out[2] = RecvBuff[1];
    out[3] = RecvBuff[2];
}

/*
 *
 */
void Parse(uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens) {
    uint16_t cmdType = cmd[0];
    switch (cmdType) {
        case 1:
            ParseSystemCmd(cmd, cmd_lens, out, out_lens);
            break;
        case 2:
            ParseSerialCmd(cmd, cmd_lens, out, out_lens);
            break;
        case 3:
            ParseI2CCmd(cmd, cmd_lens, out, out_lens);
            break;
        case 5:
            ParseOwiCmd(cmd, cmd_lens, out, out_lens);
            break;
        default:
            break;
    }
}

// 拷贝 src 到 dest，然后清零 src
void CopyAndClear(uint16_t *src, uint16_t *dest, size_t len) {
    if (!src || !dest || len == 0) return;
    size_t bytes = len * sizeof(uint16_t);
    memcpy(dest, src, bytes);
    memset(src, 0, bytes);
}
