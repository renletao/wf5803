/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "soft_spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* newlib-nano 默认不带浮点格式化，printf("%f") 会输出空，
 * 这句强制链接浮点 printf，等价于链接参数 -u _printf_float */
__asm__(".global _printf_float");

/* ------------------------------------------------------------------
 * WF5803 寄存器读写（协议归应用层，驱动只管传输）
 *   写：{ 0x00, 地址, 数据 }   bit7=0 表示写
 *   读：{ 0x80, 地址, 0xFF }   bit7=1 表示读、bit6:5=00 表示读 1 字节
 * CS 由 soft_spi_transfer() 自动包住整笔事务
 * ------------------------------------------------------------------ */

/* 写一个寄存器 */
static void wf_write_reg(uint8_t addr, uint8_t value)
{
  const uint8_t tx[3] = { 0x00, addr, value };

  soft_spi_transfer(tx, NULL, 3);
}

/* 读一个寄存器 */
static uint8_t wf_read_reg(uint8_t addr)
{
  const uint8_t tx[3] = { 0x80, addr, 0xFF };
  uint8_t rx[3];

  soft_spi_transfer(tx, rx, 3);

  return rx[2];        /* 前两个字节是命令/地址的回环，第 3 个才是数据 */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t reg;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* MX_SPI1_Init();  硬件 SPI 已停用，改用软件模拟 SPI */
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t i;

  printf("\r\nWF5803 test | soft SPI (GPIO) | Mode0 | CS=PA4\r\n");

  soft_spi_init();          /* PA5=SCK / PA7=MOSI / PA4=CS 推挽输出，PA6=MISO 上拉输入 */
  HAL_Delay(10);            /* 等传感器上电稳定，太早发命令会被忽略 */

  /* ---- 写 0x00 = 0x01：SDO_active 置 1，从 3 线切成 4 线 SPI ----
   * 这一笔如果没被芯片接受，SDO 会一直是输入，之后所有读都是 0xFF。
   * 所以写 5 次，每次都回读确认，通了就跳出 */
  for (i = 0; i < 5; i++)
  {
    wf_write_reg(0x00, 0x01);
    HAL_Delay(1);
    reg = wf_read_reg(0x00);
    if (reg == 0x01)
    {
      break;
    }
    HAL_Delay(10);
  }

  if (reg == 0x01)
  {
    printf("reg 0x00 = 0x%02X OK (try %u)\r\n", reg, (unsigned)(i + 1));
  }
  else
  {
    printf("reg 0x00 = 0x%02X MISMATCH (try %u) -> no sensor on MISO?\r\n",
           reg, (unsigned)(i));
  }

  /* ---- 读 Part_ID（0x01）：确认芯片型号 ---- */
  printf("part id  = 0x%02X\r\n", wf_read_reg(0x01));

  /* 注意：ino 里那句读改写 0xA6（设过采样率）已经去掉。
   * 手册的普通寄存器只有 0x00/0x01/0x02/0x06~0x08/0x09~0x0A/0x30，
   * 0xA6 属于 OTP 区，不是过采样率寄存器，写它可能把芯片配到奇怪的状态。
   * 过采样率是在厂家驱动 C 代码的 oversampling_setting 里设的。 */

  /* ---- 写 0x30 = 0xFB：进睡眠模式，等 1 秒稳定 ---- */
  wf_write_reg(0x30, 0xFB);
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t  RawData[5];
    uint8_t  status;
    int32_t  raw_p;
    int32_t  raw_t;
    float    fDat;
    float    press;
    float    temp;

    /* ---- 温度：触发一次转换，等 50ms（256X 过采样下不能省），再读高低字节 ---- */
    wf_write_reg(0x30, 0x08);          /* 0x08 = 触发温度转换 */
    HAL_Delay(50);
    RawData[3] = wf_read_reg(0x09);    /* 温度高字节 */
    RawData[4] = wf_read_reg(0x0A);    /* 温度低字节 */

    /* ---- 气压：触发一次转换，等 50ms，再读 24 位 ---- */
    wf_write_reg(0x30, 0x09);          /* 0x09 = 触发气压转换 */
    HAL_Delay(50);
    status     = wf_read_reg(0x02);    /* bit0 = DRDY，1 表示转换完成、数据可读 */
    RawData[0] = wf_read_reg(0x06);    /* 气压高字节 */
    RawData[1] = wf_read_reg(0x07);    /* 气压中字节 */
    RawData[2] = wf_read_reg(0x08);    /* 气压低字节 */

    /* ---- 气压：沿用 ino 的换算（24 位有符号数 / 2^23 归一化，7bar 量程） ----
     * 注意：这是 ino 的原始公式，未做温补，只是线性近似 */
    raw_p = ((int32_t)RawData[0] << 16) | ((int32_t)RawData[1] << 8) | (int32_t)RawData[2];
    fDat = (raw_p > 8388608) ? ((float)(raw_p - 16777216) / 8388608.0f)
                             : ((float)raw_p / 8388608.0f);
    press = (1000.0f * (fDat - 0.1f) + 50.0f) * 10.0f;

    /* ---- 温度：16 位有符号数 / 256，手册规定 LSB = 1/256 ℃ ---- */
    raw_t = ((int32_t)RawData[3] << 8) | (int32_t)RawData[4];
    temp = (raw_t > 32768) ? ((float)(raw_t - 65536) / 256.0f)
                           : ((float)raw_t / 256.0f);

    printf("press=%.1f,temp=%.2f  raw=%02X%02X%02X status=0x%02X\r\n",
           press, temp, RawData[0], RawData[1], RawData[2], status);

    HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &c, 1, 100);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
