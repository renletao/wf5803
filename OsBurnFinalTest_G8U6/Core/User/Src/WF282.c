/*
 * WF282.c
 *
 *  Created on: 2020年8月4日
 *      Author: ITry
 */
#include <string.h>
#include "SoftIIC.h"
#include "WF282.h"

#include "cmsis_os.h"
#include "MCP4725.h"

/****************** compensation scale factors***************************************/
#define WF282_KP_1      524288      //single
#define WF282_KP_2      1572864     //low power
#define WF282_KP_4      3670016
#define WF282_KP_8      7864320
#define WF282_KP_16     253952      //standard
#define WF282_KP_32     516096
#define WF282_KP_64     1040384     //high precision
#define WF282_KP_128    2088960

#define WF282_KT_1      524288      //single
#define WF282_KT_2      1572864     //low power
#define WF282_KT_4      3670016
#define WF282_KT_8      7864320
#define WF282_KT_16     253952      //standard
#define WF282_KT_32     516096
#define WF282_KT_64     1040384     //high precision
#define WF282_KT_128    2088960
/****************** var***************************************/
#define IIC_DEVICE_ADDR		0xEE
#define WF282_MEAS_CFG    0x08
#define WF282_PSR_BX    0x00
#define WF282_TMP_BX    0x03
#define WF282_PRS_CFG   0x06
#define WF282_TMP_CFG   0x07
#define WF282_CFG_REG   0x09
#define WF282_SOFT_RST    0x0c
#define WF282_ID_CODE   0x0d
/****************** var***************************************/
long WF282_C0;
long WF282_C1;
long WF282_C00;
long WF282_C01;
long WF282_C10;
long WF282_C11;
long WF282_C20;
long WF282_C21;
long WF282_C30;

static I2CPin info282;


void WF282_Init(void) {
    info282.pinSda = EX_TX_Pin;
    info282.pinScl = EX_RX_Pin;
    info282.portSda = EX_TX_GPIO_Port;
    info282.portScl = EX_RX_GPIO_Port;
    IIC_Init(&info282);
}

static uint8_t WriteReg(const uint8_t reg, uint8_t value) {
    uint8_t wdata[0x03];
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = reg;
    wdata[2] = value;
    return I2C_WriteNByte(&info282, wdata, 3);
}

static uint8_t ReadReg(const uint8_t reg, uint8_t *out, const uint8_t num) {
    uint8_t err;
    uint8_t wdata[0x02];
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = reg;
    if ((err = I2C_WriteNByte(&info282, wdata, 2))) goto exit_end;
    err = I2C_ReadNByte(&info282, IIC_DEVICE_ADDR, out, num);
exit_end:
    return err;
}

uint8_t WF282_Init_Calib_Param() {
    uint8_t err = 0;
uint8_t buf[18];
uint8_t buf2[18];
//    MCP4725_WriteData_Voltage(3300);
//    SetIoStatus(0, 0, 1);
//    osDelay(50);
//    //复位
//    if ((err = WriteReg(WF282_SOFT_RST, 0x09))) goto exit_end;
//    osDelay(50);
    //读系数
    buf[0] = 0x11;
    buf[1] = 0x22;
    buf[2] = 0x33;
    buf[3] = 0x44;
    buf2[0] = buf[0];
    buf2[1] = buf[1];
    buf2[2] = buf[2];
    buf2[3] = buf[3];
    err = buf[3];
    err = buf2[3];
    return 0;
    if ((err = ReadReg(0x10, buf, 18))) goto exit_end;
    WF282_C0 = (((long) buf[0]) << 4) + ((buf[1] >> 4) & 0x0F);
    if (WF282_C0 > 0x7ff) //> 0x800
        WF282_C0 = WF282_C0 - 0x1000;
    WF282_C1 = buf[2] + (((long) (buf[1] & 0x0F)) << 8);
    if (WF282_C1 > 0x7ff) //> 0x800
        WF282_C1 = WF282_C1 - 0x1000;
    WF282_C00 = (((long) ((buf[3] << 8) + buf[4])) << 4) + ((buf[5] >> 4) & 0x0F);
    if (WF282_C00 > 0x7ffff) //> 0x80000
        WF282_C00 = WF282_C00 - 0x100000;
    WF282_C10 = (((long) ((buf[5] & 0x0F) << 8) + buf[6]) << 8) + buf[7];
    if (WF282_C10 > 0x7ffff) //> 0x80000
        WF282_C10 = WF282_C10 - 0x100000;
    WF282_C01 = buf[9] + (((long) buf[8]) << 8);
    if (WF282_C01 > 0x7fff) //> 0x8000
        WF282_C01 = WF282_C01 - 0x10000;
    WF282_C11 = buf[11] + (((long) buf[10]) << 8);
    if (WF282_C11 > 0x7fff) //> 0x8000
        WF282_C11 = WF282_C11 - 0x10000;
    WF282_C20 = buf[13] + (((long) buf[12]) << 8);
    if (WF282_C20 > 0x7fff) //> 0x8000
        WF282_C20 = WF282_C20 - 0x10000;
    WF282_C21 = buf[15] + (((long) buf[14]) << 8);
    if (WF282_C21 > 0x7fff) //> 0x8000
        WF282_C21 = WF282_C21 - 0x10000;
    WF282_C30 = buf[17] + (((long) buf[16]) << 8);
    if (WF282_C30 > 0x7fff) //> 0x8000
        WF282_C30 = WF282_C30 - 0x10000;
exit_end:
    //MCP4725_WriteData_Voltage(0);
    //SetIoStatus(0, 0, 0);
    return err;
}

uint8_t WF282_Read_ID(uint8_t *id) {
    return ReadReg(WF282_ID_CODE, id, 1);
}

uint8_t WF282_Get_Data_Press_Temp(uint8_t data_num, double *readData) {
    uint8_t err = 0;
    uint8_t buf[3];
    long WF282_NewPressData, WF282_NewTempData;
    double tmpData, Traw_sc, Praw_sc;
    // if (readData == NULL || data_num == 0) {
    //     err = 0xFE;
    //     goto exit_end;
    // }
    MCP4725_WriteData_Voltage(3300);
    osDelay(50);
    SetIoStatus(0, 0, 1);
    //初始化系数
    if ((err = WriteReg(WF282_PRS_CFG, 0x07))) goto exit_end;
    osDelay(1);
    if ((err = WriteReg(WF282_TMP_CFG, 0x07))) goto exit_end;
    osDelay(1);
    if ((err = WriteReg(WF282_CFG_REG, 0x0C))) goto exit_end;
    osDelay(1);
    //API_WF282_Read_Temperature
    if ((err = WriteReg(WF282_MEAS_CFG, 0x02))) goto exit_end;
    osDelay(250);
    if ((err = ReadReg(WF282_TMP_BX, buf, 3))) goto exit_end;
    WF282_NewTempData = buf[0];
    WF282_NewTempData <<= 8;
    WF282_NewTempData |= buf[1];
    WF282_NewTempData <<= 8;
    WF282_NewTempData |= buf[2];
    if (WF282_NewTempData > 0x7FFFFF) {
        //(2^23-1)  > 0x800000
        WF282_NewTempData = WF282_NewTempData - 0x1000000; //2^24
    }
    //API_WF282_Read_Press
    if ((err = WriteReg(WF282_MEAS_CFG, 0x01))) goto exit_end;
    osDelay(250);
    if ((err = ReadReg(WF282_PSR_BX, buf, 3))) goto exit_end;
    WF282_NewPressData = buf[0];
    WF282_NewPressData <<= 8;
    WF282_NewPressData |= buf[1];
    WF282_NewPressData <<= 8;
    WF282_NewPressData |= buf[2];
    if (WF282_NewPressData > 0x7FFFFF) {
        //(2^23-1)  > 0x800000
        WF282_NewPressData = WF282_NewPressData - 0x1000000; //2^24
    }
    //
    Traw_sc = ((double) WF282_NewTempData) / WF282_KT_128;
    tmpData = WF282_C0 * 0.5 + WF282_C1 * Traw_sc;
    readData[1] = tmpData;
    Praw_sc = ((double) WF282_NewPressData) / WF282_KP_128;
    tmpData = (WF282_C00) + Praw_sc * (WF282_C10 + Praw_sc * (WF282_C20 + Praw_sc * WF282_C30))
              + Traw_sc * WF282_C01 + Traw_sc * Praw_sc * (WF282_C11 + Praw_sc + WF282_C21);
    readData[0] = tmpData;
exit_end:
    MCP4725_WriteData_Voltage(0);
    SetIoStatus(0, 0, 0);
    return err;
}
