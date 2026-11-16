/**
  ******************************************************************************
  * @file    soft_spi.h
  * @brief   软件模拟 SPI（GPIO 翻转），Mode0 / MSB 先行 / 8 位
  *
  * 纯传输层驱动：只负责把字节按 SPI 时序搬出去、搬回来。
  * 不包含任何具体传感器的寄存器协议，那些留在应用层（main.c）。
  *
  * 特点：
  *   - 只用 HAL_GPIO_WritePin / HAL_GPIO_ReadPin，不占用任何 SPI 外设
  *   - 换引脚只需要改下面 "硬件配置" 这一段
  *   - 没有超时参数：时钟由指令产生，不存在等标志位超时的情况
  *
  * 分层：
  *   soft_spi_txrx()     1 个字节
  *   soft_spi_transfer() 一整笔事务（CS 自动包住）
  ******************************************************************************
  */
#ifndef __SOFT_SPI_H__
#define __SOFT_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================================================================
 * 硬件配置：换引脚只改这一段
 * 每个信号的端口时钟单独使能，这样即使四个脚分布在不同的 GPIO 端口也能工作
 * ==========================================================================*/
/* SCK 复用 EX_SCL(PA6)：传感器只有一根时钟脚（焊盘 1 SCLK，"Serial data clock"，
 * I2C 的 SCL 与 SPI 的 SCLK 是同一个焊盘），12 芯排线（J2）里能接这根脚的只有
 * I2C_SCL 网络 → PA6。本板没有独立的 SPI_SCK 引脚，所以 SCK 必须落在 PA6。
 *
 * 由此带来两条使用约束：
 *   1) 同一个焊盘先跑 I2C、再切 4 线 SPI，所以 soft_spi_init() 必须在 I2C 通信
 *      （Sensor2200_Init 里的 IIC_Init）和 ADC_MspInit 之后调用 —— 它会把这几个
 *      脚从 OD/ANALOG 重新切成推挽输出。当前 Sensor2200.c 每次读之前都调一次。
 *   2) I2C 的 SDA(PA7)/SCL(PA6) 都是开漏输出，切到 SPI 后 PA7 保持释放（高阻），
 *      不会和 MOSI(PC6) 抢同一根数据线。 */
#define SOFT_SPI_SCK_PORT          GPIOA
#define SOFT_SPI_SCK_PIN           GPIO_PIN_6
#define SOFT_SPI_SCK_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

/* 以下三根是本板实测引脚，对应 main.h 的 EX_SPI_MOSI / EX_SPI_MISO / EX_SPI_CS */
#define SOFT_SPI_MOSI_PORT         GPIOC
#define SOFT_SPI_MOSI_PIN          GPIO_PIN_6
#define SOFT_SPI_MOSI_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define SOFT_SPI_MISO_PORT         GPIOA
#define SOFT_SPI_MISO_PIN          GPIO_PIN_8
#define SOFT_SPI_MISO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define SOFT_SPI_CS_PORT           GPIOB
#define SOFT_SPI_CS_PIN            GPIO_PIN_1
#define SOFT_SPI_CS_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

/* 半个时钟周期的延时。
 * 用 HAL_GPIO_WritePin 时，一个边沿本身就要 ~1us（函数调用开销），
 * 实际 SCK 大约 300~400kHz，这里的 NOP 影响很小，留作微调用。
 * 想跑到 1MHz 以上，把电平宏换成直接写 BSRR：
 *     #define SOFT_SPI_SCK_H()  (SOFT_SPI_SCK_PORT->BSRR = SOFT_SPI_SCK_PIN)
 *     #define SOFT_SPI_SCK_L()  (SOFT_SPI_SCK_PORT->BSRR = (uint32_t)SOFT_SPI_SCK_PIN << 16) */
#define SOFT_SPI_DELAY()           do { __NOP(); __NOP(); } while (0)

/* 片选建立/保持时间。
 * CS 拉低后到第一个时钟沿之间必须留空隙，从机要时间开始驱动 MISO；
 * CS 拉高前也要留一点，最后一位才算真正传完。
 * 用 HAL_GPIO_WritePin 时天然有几微秒，够用；
 * 一旦换成 BSRR 直写（几 MHz），这两段空隙就是必需的。 */
#define SOFT_SPI_CS_SETUP()        do { __NOP(); __NOP(); __NOP(); __NOP(); } while (0)
#define SOFT_SPI_CS_HOLD()         do { __NOP(); __NOP(); __NOP(); __NOP(); } while (0)

/* ============================================================================
 * 引脚电平操作
 * ==========================================================================*/
#define SOFT_SPI_SCK_H()    HAL_GPIO_WritePin(SOFT_SPI_SCK_PORT,  SOFT_SPI_SCK_PIN,  GPIO_PIN_SET)
#define SOFT_SPI_SCK_L()    HAL_GPIO_WritePin(SOFT_SPI_SCK_PORT,  SOFT_SPI_SCK_PIN,  GPIO_PIN_RESET)
#define SOFT_SPI_MOSI_H()   HAL_GPIO_WritePin(SOFT_SPI_MOSI_PORT, SOFT_SPI_MOSI_PIN, GPIO_PIN_SET)
#define SOFT_SPI_MOSI_L()   HAL_GPIO_WritePin(SOFT_SPI_MOSI_PORT, SOFT_SPI_MOSI_PIN, GPIO_PIN_RESET)
#define SOFT_SPI_CS_H()     HAL_GPIO_WritePin(SOFT_SPI_CS_PORT,   SOFT_SPI_CS_PIN,   GPIO_PIN_SET)
#define SOFT_SPI_CS_L()     HAL_GPIO_WritePin(SOFT_SPI_CS_PORT,   SOFT_SPI_CS_PIN,   GPIO_PIN_RESET)
#define SOFT_SPI_MISO_READ() (HAL_GPIO_ReadPin(SOFT_SPI_MISO_PORT, SOFT_SPI_MISO_PIN) != GPIO_PIN_RESET)

/* ============================================================================
 * 接口
 * ==========================================================================*/

/* 初始化：SCK/MOSI/CS 配成推挽输出，MISO 配成上拉输入。CS 上电默认拉高 */
void soft_spi_init(void);

/* 片选：level = 0 拉低（选中），非 0 拉高（释放） */
void soft_spi_cs(uint8_t level);

/* 收发一个字节（全双工：发的同时收），返回收到的字节 */
uint8_t soft_spi_txrx(uint8_t tx);

/* ----------------------------------------------------------------------------
 * 事务级接口：一次调用完成一整笔 len 字节的连续收发
 * CS 自动包住整笔事务（拉低 -> 收发 -> 拉高），不需要手动控制
 * 对应硬件 SPI 的 HAL_SPI_TransmitReceive()，但没有超时参数
 *   —— 软件 SPI 的时钟由指令自己产生，不存在"等不到标志位"的情况，不需要超时
 * -------------------------------------------------------------------------- */

/* 连续收发：tx 发出去，rx 收回来
 *   tx == NULL  只读，内部自动发 0xFF 产生时钟
 *   rx == NULL  只写，收到的数据丢弃
 *   两个都不为 NULL 就是全双工，长度都是 len */
void soft_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

/* 连续写 len 字节（等价于 HAL_SPI_Transmit，无超时） */
void soft_spi_write(const uint8_t *tx, uint16_t len);

/* 连续读 len 字节（等价于 HAL_SPI_Receive，无超时） */
void soft_spi_read(uint8_t *rx, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_SPI_H__ */
