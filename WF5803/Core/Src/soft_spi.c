/**
  ******************************************************************************
  * @file    soft_spi.c
  * @brief   软件模拟 SPI（GPIO 翻转），Mode0 / MSB 先行 / 8 位
  ******************************************************************************
  */
#include <stddef.h>     /* NULL */
#include "soft_spi.h"

/* ----------------------------------------------------------------------------
 * 初始化
 *   SCK  / MOSI / CS ：推挽输出（CS 先置高，避免上电瞬间误选中）
 *   MISO             ：上拉输入（CS 为高时从机 SDO 是高阻，不能悬空）
 * 注意：如果这些引脚原来被 SPI1 复用（AF_PP），这里重新配置会把它切成普通 GPIO，
 *       SPI1 外设就不再控制这几个脚了。
 * -------------------------------------------------------------------------- */
void soft_spi_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  SOFT_SPI_SCK_CLK_ENABLE();
  SOFT_SPI_MOSI_CLK_ENABLE();
  SOFT_SPI_MISO_CLK_ENABLE();
  SOFT_SPI_CS_CLK_ENABLE();

  /* 先把 CS 置高，再切成输出，避免出现一个低电平毛刺 */
  SOFT_SPI_CS_H();
  SOFT_SPI_SCK_L();          /* Mode0：时钟空闲为低 */

  /* SCK / MOSI / CS 推挽输出 */
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio.Pin = SOFT_SPI_SCK_PIN;
  HAL_GPIO_Init(SOFT_SPI_SCK_PORT, &gpio);

  gpio.Pin = SOFT_SPI_MOSI_PIN;
  HAL_GPIO_Init(SOFT_SPI_MOSI_PORT, &gpio);

  gpio.Pin = SOFT_SPI_CS_PIN;
  HAL_GPIO_Init(SOFT_SPI_CS_PORT, &gpio);

  /* MISO 上拉输入 */
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Pin  = SOFT_SPI_MISO_PIN;
  HAL_GPIO_Init(SOFT_SPI_MISO_PORT, &gpio);

  SOFT_SPI_CS_H();
}

/* ----------------------------------------------------------------------------
 * 片选：0 = 拉低选中，非 0 = 拉高释放
 * 拉低后留一段建立时间，拉高前留一段保持时间，保证从机有时间响应
 * -------------------------------------------------------------------------- */
void soft_spi_cs(uint8_t level)
{
  if (level)
  {
    SOFT_SPI_CS_HOLD();      /* 最后一位真正传完再释放片选 */
    SOFT_SPI_CS_H();
  }
  else
  {
    SOFT_SPI_CS_L();
    SOFT_SPI_CS_SETUP();     /* 给从机时间开始驱动 MISO */
  }
}

/* ----------------------------------------------------------------------------
 * 收发一个字节
 *
 * Mode0 的时序：
 *   SCK 空闲为低；数据在 SCK 低电平期间准备好；
 *   上升沿器件锁存 MOSI，同时主机采样 MISO；
 *   下降沿器件更新 MISO。
 *
 * 注意读取顺序：先左移 rx 再置最低位，这样第一次读到的才是最高位。
 * -------------------------------------------------------------------------- */
uint8_t soft_spi_txrx(uint8_t tx)
{
  uint8_t i;
  uint8_t rx = 0;

  for (i = 0; i < 8; i++)
  {
    /* 1. 先把当前最高位摆到 MOSI 上（MSB 先行） */
    if (tx & 0x80)
    {
      SOFT_SPI_MOSI_H();
    }
    else
    {
      SOFT_SPI_MOSI_L();
    }
    tx <<= 1;

    /* 2. 上升沿：写出去，同时把 MISO 采回来 */
    SOFT_SPI_SCK_H();

    rx <<= 1;
    if (SOFT_SPI_MISO_READ())
    {
      rx |= 0x01;
    }

    /* 3. 下降沿：器件更新下一位数据 */
    SOFT_SPI_SCK_L();

    SOFT_SPI_DELAY();
  }

  return rx;
}

/* ----------------------------------------------------------------------------
 * 一次完整的事务：CS 拉低 -> 连续收发 len 字节 -> CS 拉高
 *
 * 对应硬件 SPI 的 HAL_SPI_TransmitReceive()，区别是没有超时参数：
 * 软件 SPI 的时钟是 CPU 自己翻引脚产生的，不存在"等 TXE/RXNE 标志位超时"
 * 这回事，所以根本不需要 Timeout。
 *
 *   tx == NULL  只读：内部固定发 0xFF 来产生时钟
 *   rx == NULL  只写：收到的字节直接丢弃
 * -------------------------------------------------------------------------- */
void soft_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  uint16_t i;
  uint8_t  rxbyte;

  soft_spi_cs(0);

  for (i = 0; i < len; i++)
  {
    rxbyte = soft_spi_txrx((tx != NULL) ? tx[i] : 0xFF);
    if (rx != NULL)
    {
      rx[i] = rxbyte;
    }
  }

  soft_spi_cs(1);
}

/* 连续写 len 字节 */
void soft_spi_write(const uint8_t *tx, uint16_t len)
{
  soft_spi_transfer(tx, NULL, len);
}

/* 连续读 len 字节（内部发 0xFF 产生时钟） */
void soft_spi_read(uint8_t *rx, uint16_t len)
{
  soft_spi_transfer(NULL, rx, len);
}
