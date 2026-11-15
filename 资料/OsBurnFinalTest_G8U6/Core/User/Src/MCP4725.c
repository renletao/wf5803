/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "../Inc/MCP4725.h"

/* USER CODE BEGIN 0 */
#include "../Inc/SoftIIC.h"
#include "cmsis_os.h"
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
static I2CPin info4725;
/* USER CODE END 1 */

void MCP4725_Init(void) {
    info4725.pinSda = DAC_SDA_Pin;
    info4725.pinScl = DAC_SCL_Pin;
    info4725.portSda = DAC_SDA_GPIO_Port;
    info4725.portScl = DAC_SCL_GPIO_Port;
    IIC_Init(&info4725);
    osDelay(10);
    MCP4725_WriteData_Voltage(0);
}

void MCP4725_WriteData_Voltage(uint16_t mV) //��ѹ��λmV
{
    uint8_t data[3];
    uint8_t temp;
    uint16_t Dn;
    if (mV > 0) mV += 200; //补二极管的压降
    Dn = (4096 * mV / 2) / VREF_3V3; //�����VREF_3V3�궨��Ϊ3300
    temp = (0x0F00 & Dn) >> 8; //12λ���ݡ�0000XXXX XXXXXXXX
    data[0] = 0xC0;
    data[1] = temp; //12λ���ݡ�0000XXXX XXXXXXXX
    data[2] = Dn; //�����VREF_5V�궨��Ϊ5000
    I2C_WriteNByte(&info4725, data, 3);
    osDelay(10);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
