//
// Created by WFH-ITRY on 2026/10/20.
//

#include "../Inc/Sensor2200.h"
#include "cmsis_os.h"
#include "../Inc/MCP4725.h"
#include "../Inc/SoftIIC.h"

#define IIC_DEVICE_ADDR		0xDA
static I2CPin info2200;

void Sensor2200_Init(void) {
    info2200.pinSda = EX_SDA_Pin;
    info2200.pinScl = EX_SCL_Pin;
    info2200.portSda = EX_SDA_GPIO_Port;
    info2200.portScl = EX_SCL_GPIO_Port;
    IIC_Init(&info2200);
}

///
/// @param dst 19bytes
/// @return
static uint8_t ReadCoefs(uint16_t *dst) {
    uint8_t err;
    uint8_t wdata[0x02];
    uint8_t rdata[0x0E];
    uint8_t i = 0;
    uint8_t n = 0;
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = 0xA4;
    if ((err = I2C_WriteNByte(&info2200, wdata, 2))) goto exit_end;
    err = I2C_ReadNByte(&info2200, IIC_DEVICE_ADDR, rdata, 0x05);
    for (i = 0; i < 0x05; i++) dst[n++] = rdata[i];
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = 0xAE;
    if ((err = I2C_WriteNByte(&info2200, wdata, 2))) goto exit_end;
    if ((err = I2C_ReadNByte(&info2200, IIC_DEVICE_ADDR, rdata, 0x0E))) goto exit_end;
    for (i = 0; i < 0x0E; i++) dst[n++] = rdata[i];
exit_end:
    return err;
}

static uint8_t WriteReg(uint8_t reg, uint8_t value) {
    uint8_t wdata[0x03];
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = reg;
    wdata[2] = value;
    return I2C_WriteNByte(&info2200, wdata, 3);
}

static uint8_t ReadReg(uint8_t reg, uint8_t *out, const uint8_t num) {
    uint8_t err;
    uint8_t wdata[0x02];
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = reg;
    if ((err = I2C_WriteNByte(&info2200, wdata, 2))) goto exit_end;
    err = I2C_ReadNByte(&info2200, IIC_DEVICE_ADDR, out, num);
exit_end:
    return err;
}

///
/// @param dst 5bytes
/// @return
static uint8_t ReadCalOut(uint16_t *dst) {
    uint8_t err;
    uint8_t wdata[0x03];
    uint8_t rdata[0x05];
    uint8_t i = 0;
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = 0x30;
    wdata[2] = 0x0A;
    if ((err = I2C_WriteNByte(&info2200, wdata, 3))) goto exit_end;
    osDelay(10);
    wdata[0] = IIC_DEVICE_ADDR;
    wdata[1] = 0x06;
    if ((err = I2C_WriteNByte(&info2200, wdata, 2))) goto exit_end;
    if ((err = I2C_ReadNByte(&info2200, IIC_DEVICE_ADDR, rdata, 0x05))) goto exit_end;
    for (i = 0; i < 0x05; i++) dst[i] = rdata[i];
exit_end:
    return err;
}

void Get2200FinalTestData(uint16_t *dst, uint8_t cmpMode) {
    uint8_t err;
    uint8_t n = 1;
    MCP4725_WriteData_Voltage(cmpMode ? 2000 : 3300);
    SetIoStatus(0, 0, 0);
    osDelay(50);
    SetIoStatus(0, 0, 1);
    osDelay(50);
    if (cmpMode) {
        if ((err = ReadCoefs(&dst[n]))) goto exit_end;
        MCP4725_WriteData_Voltage(3300);
        osDelay(50);
    }
    n += 19;
    if ((err = ReadCoefs(&dst[n]))) goto exit_end;
    n += 19;
    if ((err = ReadCalOut(&dst[n]))) goto exit_end;
    n += 5;
    //TODO:SPI
exit_end:
    dst[0] = err;
    MCP4725_WriteData_Voltage(0);
    SetIoStatus(0, 0, 0);
}

uint8_t Set2200Offset(uint8_t offset) {
    uint8_t err;
    uint8_t a8;
    MCP4725_WriteData_Voltage(3300);
    osDelay(50);
    SetIoStatus(0, 0, 1);
    if ((err = ReadReg(0xA8, &a8, 1))) goto exit_end;
    if (a8 != 0 || a8 == offset) {
        err = 0xFE;
        goto exit_end;
    }
    if ((err = WriteReg(0xA8, offset))) goto exit_end;
    osDelay(5);
    //otp
    MCP4725_WriteData_Voltage(6200);
    osDelay(50);
    if ((err = WriteReg(0x6B, 0xC2))) goto exit_end;
    osDelay(10);
    if ((err = WriteReg(0x30, 0x0D))) goto exit_end;
    osDelay(10);
    if ((err = WriteReg(0x6C, 0x6A))) goto exit_end;
    osDelay(300);
    err = WriteReg(0x30, 0x00);
exit_end:
    MCP4725_WriteData_Voltage(0);
    SetIoStatus(0, 0, 0);
    return err;
}
