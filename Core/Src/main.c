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
#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADS1115_ADDR_MIN_7BIT    0x48U
#define ADS1115_ADDR_MAX_7BIT    0x4BU
#define ADS1115_ADDR_7BIT        0x48U
#define ADS1115_ADDR             (ADS1115_ADDR_7BIT << 1)   /* Shifted for HAL */
#define I2C_SCAN_MIN_7BIT        0x48U
#define I2C_SCAN_MAX_7BIT        0x4BU
#define DISCOVERY_INTERVAL_MS    1000U
#define ADS1115_REG_CONV     0x00
#define ADS1115_REG_CONFIG   0x01
#define ADS1115_NUM_CH       4

#define I2C1_SCL_PORT        GPIOB
#define I2C1_SCL_PIN         GPIO_PIN_8
#define I2C1_SDA_PORT        GPIOB
#define I2C1_SDA_PIN         GPIO_PIN_9

#define ADS1115_OS_SINGLE    0x8000
#define ADS1115_MUX_AIN0     0x4000
#define ADS1115_MUX_AIN1     0x5000
#define ADS1115_MUX_AIN2     0x6000
#define ADS1115_MUX_AIN3     0x7000
#define ADS1115_PGA_4V096    0x0200
#define ADS1115_MODE_SINGLE  0x0100
#define ADS1115_DR_860SPS    0x00E0
#define ADS1115_COMP_DISABLE 0x0003
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t g_ads1115_addr_7bit = ADS1115_ADDR_7BIT;
static uint32_t g_last_i2c_shout_tick = 0U;
static uint8_t g_i2c1_active_mask = 0U;
static uint8_t g_i2c2_active_mask = 0U;
static uint32_t g_last_discovery_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);



static void MX_USART1_UART_Init(void);
static void I2C_BusReset(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * Read one single-ended channel from ADS1115.
 * channel: 0=AIN0, 1=AIN1, 2=AIN2, 3=AIN3
 * Config: single-shot, PGA ±4.096V, 128 SPS, comparator off.
 */
static HAL_StatusTypeDef ADS1115_IsReady(I2C_HandleTypeDef *hi2c)
{
  return HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(g_ads1115_addr_7bit << 1), 3, 100);
}

static const char *HAL_StatusToString(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK:
      return "HAL_OK";
    case HAL_ERROR:
      return "HAL_ERROR";
    case HAL_BUSY:
      return "HAL_BUSY";
    case HAL_TIMEOUT:
      return "HAL_TIMEOUT";
    default:
      return "HAL_UNKNOWN";
  }
}

static const char *HAL_I2C_StateToString(HAL_I2C_StateTypeDef state)
{
  switch (state)
  {
    case HAL_I2C_STATE_RESET:
      return "RESET";
    case HAL_I2C_STATE_READY:
      return "READY";
    case HAL_I2C_STATE_BUSY:
      return "BUSY";
    case HAL_I2C_STATE_BUSY_TX:
      return "BUSY_TX";
    case HAL_I2C_STATE_BUSY_RX:
      return "BUSY_RX";
    case HAL_I2C_STATE_LISTEN:
      return "LISTEN";
    case HAL_I2C_STATE_BUSY_TX_LISTEN:
      return "BUSY_TX_LISTEN";
    case HAL_I2C_STATE_BUSY_RX_LISTEN:
      return "BUSY_RX_LISTEN";
    case HAL_I2C_STATE_ABORT:
      return "ABORT";
    case HAL_I2C_STATE_TIMEOUT:
      return "TIMEOUT";
    case HAL_I2C_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

static void I2C_ErrorFlagsToString(uint32_t error, char *buf, size_t buf_len)
{
  size_t used = 0U;

  if ((buf == NULL) || (buf_len == 0U))
  {
    return;
  }

  buf[0] = '\0';

  if (error == HAL_I2C_ERROR_NONE)
  {
    (void)snprintf(buf, buf_len, "NONE");
    return;
  }

#define APPEND_I2C_FLAG(flag, label) \
  do \
  { \
    if ((error & (flag)) != 0U) \
    { \
      used += (size_t)snprintf(&buf[used], (used < buf_len) ? (buf_len - used) : 0U, \
                               "%s%s", (used > 0U) ? "|" : "", (label)); \
    } \
  } while (0)

  APPEND_I2C_FLAG(HAL_I2C_ERROR_BERR, "BERR");
  APPEND_I2C_FLAG(HAL_I2C_ERROR_ARLO, "ARLO");
  APPEND_I2C_FLAG(HAL_I2C_ERROR_AF, "AF/NACK");
  APPEND_I2C_FLAG(HAL_I2C_ERROR_OVR, "OVR");
  APPEND_I2C_FLAG(HAL_I2C_ERROR_DMA, "DMA");
  APPEND_I2C_FLAG(HAL_I2C_ERROR_TIMEOUT, "TIMEOUT");
  APPEND_I2C_FLAG(HAL_I2C_ERROR_SIZE, "SIZE");
  APPEND_I2C_FLAG(HAL_I2C_ERROR_DMA_PARAM, "DMA_PARAM");

#undef APPEND_I2C_FLAG
}

static HAL_StatusTypeDef USB_SendString(const char *text)
{
  uint32_t start_tick = HAL_GetTick();
  uint16_t len = (uint16_t)strlen(text);

  while (CDC_Transmit_FS((uint8_t *)text, len) == USBD_BUSY)
  {
    if ((HAL_GetTick() - start_tick) > 50U)
    {
      return HAL_TIMEOUT;
    }
    HAL_Delay(1);
  }

  return HAL_OK;
}

static uint16_t ADS1115_GetMuxForChannel(uint8_t channel)
{
  switch (channel)
  {
    case 0:
      return ADS1115_MUX_AIN0;
    case 1:
      return ADS1115_MUX_AIN1;
    case 2:
      return ADS1115_MUX_AIN2;
    case 3:
      return ADS1115_MUX_AIN3;
    default:
      return ADS1115_MUX_AIN0;
  }
}

static HAL_StatusTypeDef ADS1115_LogAddressScan(I2C_HandleTypeDef *hi2c)
{
  char buf[128];
  uint8_t found = 0U;
  uint8_t addr;

  (void)snprintf(buf, sizeof(buf),
                 "I2C1 scan: checking ADS1115 addresses 0x48..0x4B on PB8/PB9, current=0x%02X\r\n",
                 g_ads1115_addr_7bit);
  USB_SendString(buf);

  for (addr = ADS1115_ADDR_MIN_7BIT; addr <= ADS1115_ADDR_MAX_7BIT; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 50) == HAL_OK)
    {
      found = 1U;
      g_ads1115_addr_7bit = addr;
      (void)snprintf(buf, sizeof(buf), "I2C1 device ACK at 0x%02X\r\n", addr);
      USB_SendString(buf);
      break;
    }
  }

  if (found == 0U)
  {
    USB_SendString("I2C1 scan: no device ACK at ADS1115 addresses 0x48..0x4B\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

static void I2C1_LogWhoAnswered(I2C_HandleTypeDef *hi2c)
{
  char buf[96];
  uint8_t addr;
  uint8_t any_found = 0U;

  USB_SendString("I2C1 shout: anybody there? scanning 0x08..0x77\r\n");

  for (addr = 0x08U; addr <= 0x77U; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 10) == HAL_OK)
    {
      any_found = 1U;
      (void)snprintf(buf, sizeof(buf), "I2C1 ACK at 0x%02X\r\n", addr);
      USB_SendString(buf);
    }
  }

  if (any_found == 0U)
  {
    USB_SendString("I2C1 shout result: no ACK from any 7-bit address\r\n");
  }
}

static void ADS1115_LogReadFailure(I2C_HandleTypeDef *hi2c, uint8_t channel, HAL_StatusTypeDef status)
{
  char buf[96];
  char error_flags[64];
  uint32_t i2c_error = HAL_I2C_GetError(hi2c);
  HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(hi2c);

  I2C_ErrorFlagsToString(i2c_error, error_flags, sizeof(error_flags));
  (void)snprintf(buf, sizeof(buf),
                 "ADS1115 A%u read failed at 0x%02X\r\n",
                 (unsigned int)channel,
                 g_ads1115_addr_7bit);
  USB_SendString(buf);

  (void)snprintf(buf, sizeof(buf),
                 "status=%s(%lu) state=%s(0x%02X)\r\n",
                 HAL_StatusToString(status),
                 (unsigned long)status,
                 HAL_I2C_StateToString(i2c_state),
                 (unsigned int)i2c_state);
  USB_SendString(buf);

  (void)snprintf(buf, sizeof(buf),
                 "i2c_error=0x%08lX [%s]\r\n",
                 (unsigned long)i2c_error,
                 error_flags);
  USB_SendString(buf);

  if (status == HAL_BUSY)
  {
    USB_SendString("Hint: BUSY means I2C1 never became idle. Check for stuck SDA/SCL, wrong pins, or another transfer left active.\r\n");
  }
  else if ((i2c_error & HAL_I2C_ERROR_AF) != 0U)
  {
    USB_SendString("Hint: AF/NACK usually means the slave did not acknowledge; check ADS1115 ADDR wiring, power, ground, pull-ups, and that SCL/SDA are on PB8/PB9.\r\n");
  }
  else if ((i2c_error & HAL_I2C_ERROR_TIMEOUT) != 0U)
  {
    USB_SendString("Hint: TIMEOUT usually means the bus is stuck or the slave did not complete the transfer; inspect SCL/SDA levels and pull-ups.\r\n");
  }
  else if (status == HAL_ERROR)
  {
    USB_SendString("Hint: HAL_ERROR is generic. The error flags above are the useful clue.\r\n");
  }
}

static HAL_StatusTypeDef ADS1115_ReadChannel(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t channel, int16_t *value)
{
  uint16_t config;
  uint8_t cfg[2];
  uint8_t raw[2];
  uint32_t start_tick;
  HAL_StatusTypeDef io_status;

  if ((value == NULL) || (channel >= ADS1115_NUM_CH))
  {
    return HAL_ERROR;
  }

  config = ADS1115_OS_SINGLE |
           ADS1115_GetMuxForChannel(channel) |
           ADS1115_PGA_4V096 |
           ADS1115_MODE_SINGLE |
           ADS1115_DR_860SPS |
           ADS1115_COMP_DISABLE;
  cfg[0] = (uint8_t)(config >> 8);
  cfg[1] = (uint8_t)(config & 0xFF);

  io_status = HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr_7bit << 1), ADS1115_REG_CONFIG, 1, cfg, 2, 100);
  if (io_status != HAL_OK)
  {
    if (io_status == HAL_BUSY)
    {
      if (hi2c == &hi2c1)
      {
        I2C_BusReset();
      }
      else if (hi2c == &hi2c2)
      {
        (void)HAL_I2C_DeInit(&hi2c2);
        MX_I2C2_Init();
      }
    }
    return io_status;
  }

  start_tick = HAL_GetTick();
  do
  {
    io_status = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr_7bit << 1), ADS1115_REG_CONFIG, 1, cfg, 2, 100);
    if (io_status != HAL_OK)
    {
      return io_status;
    }

    config = ((uint16_t)cfg[0] << 8) | cfg[1];
    if ((config & ADS1115_OS_SINGLE) != 0U)
    {
      break;
    }
  } while ((HAL_GetTick() - start_tick) < 20U);

  if ((config & ADS1115_OS_SINGLE) == 0U)
  {
    return HAL_TIMEOUT;
  }

  io_status = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr_7bit << 1), ADS1115_REG_CONV, 1, raw, 2, 100);
  if (io_status != HAL_OK)
  {
    return io_status;
  }

  *value = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  return HAL_OK;
}

static void PollAdsRangeAndAppend(const char *bus_name, I2C_HandleTypeDef *hi2c, char *line, size_t line_size, size_t *used, uint8_t *any_ok)
{
  uint8_t addr;
  uint8_t channel;
  int16_t adc_values[ADS1115_NUM_CH];
  HAL_StatusTypeDef status = HAL_OK;
  int written;

  if ((line == NULL) || (used == NULL) || (*used >= line_size))
  {
    return;
  }

  for (addr = I2C_SCAN_MIN_7BIT; addr <= I2C_SCAN_MAX_7BIT; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 10) != HAL_OK)
    {
      continue;
    }

    for (channel = 0; channel < ADS1115_NUM_CH; channel++)
    {
      status = ADS1115_ReadChannel(hi2c, addr, channel, &adc_values[channel]);
      if (status != HAL_OK)
      {
        break;
      }
    }

    if (status == HAL_OK)
    {
      *any_ok = 1U;
      written = snprintf(&line[*used], line_size - *used,
                         "%s%s@0x%02X[A0:%d,A1:%d,A2:%d,A3:%d]",
                         (*used == 0U) ? "" : " | ",
                         bus_name, addr, adc_values[0], adc_values[1], adc_values[2], adc_values[3]);
    }
    else
    {
      written = snprintf(&line[*used], line_size - *used,
                         "%s%s@0x%02X[ACK-ERR]",
                         (*used == 0U) ? "" : " | ",
                         bus_name, addr);
    }

    if (written > 0)
    {
      *used += (size_t)written;
      if (*used >= line_size)
      {
        line[line_size - 1U] = '\0';
        *used = line_size - 1U;
        return;
      }
    }
  }
}

static uint8_t DiscoverBusMask(I2C_HandleTypeDef *hi2c)
{
  uint8_t addr;
  uint8_t mask = 0U;

  for (addr = ADS1115_ADDR_MIN_7BIT; addr <= ADS1115_ADDR_MAX_7BIT; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 5) == HAL_OK)
    {
      mask |= (uint8_t)(1U << (addr - ADS1115_ADDR_MIN_7BIT));
    }
  }
  return mask;
}

static void PollActiveMaskAndAppend(const char *bus_name,
                                    I2C_HandleTypeDef *hi2c,
                                    uint8_t *mask,
                                    char *line,
                                    size_t line_size,
                                    size_t *used,
                                    uint8_t *any_ok,
                                    uint8_t *had_failure)
{
  uint8_t addr;
  uint8_t channel;
  int16_t adc_values[ADS1115_NUM_CH];
  HAL_StatusTypeDef status = HAL_OK;
  int written;

  if ((mask == NULL) || (*mask == 0U) || (line == NULL) || (used == NULL) || (*used >= line_size))
  {
    return;
  }

  for (addr = ADS1115_ADDR_MIN_7BIT; addr <= ADS1115_ADDR_MAX_7BIT; addr++)
  {
    uint8_t bit = (uint8_t)(1U << (addr - ADS1115_ADDR_MIN_7BIT));
    if (((*mask) & bit) == 0U)
    {
      continue;
    }

    for (channel = 0; channel < ADS1115_NUM_CH; channel++)
    {
      status = ADS1115_ReadChannel(hi2c, addr, channel, &adc_values[channel]);
      if (status != HAL_OK)
      {
        break;
      }
    }

    if (status == HAL_OK)
    {
      *any_ok = 1U;
      written = snprintf(&line[*used], line_size - *used,
                         "%s%s@0x%02X[A0:%d,A1:%d,A2:%d,A3:%d]",
                         (*used == 0U) ? "" : " | ",
                         bus_name, addr, adc_values[0], adc_values[1], adc_values[2], adc_values[3]);
    }
    else
    {
      *had_failure = 1U;
      *mask &= (uint8_t)(~bit);
      written = snprintf(&line[*used], line_size - *used,
                         "%s%s@0x%02X[LOST]",
                         (*used == 0U) ? "" : " | ",
                         bus_name, addr);
    }

    if (written > 0)
    {
      *used += (size_t)written;
      if (*used >= line_size)
      {
        line[line_size - 1U] = '\0';
        *used = line_size - 1U;
        return;
      }
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();

  // 操作PA11和PA12（STM32F103的USB D-和D+）
  // 1. 将PA11、PA12配置为推挽输出
  GPIOA->CRH &= 0XFFF00FFF;  // 清空配置
  GPIOA->CRH |= 0X00033000;  // 设为推挽输出

  // 2. 拉低D+和D-，模拟断开
  GPIOA->ODR &= ~(GPIO_PIN_11 | GPIO_PIN_12);
  HAL_Delay(10);  // 保持10ms，电脑检测到断开

  // 3. 恢复引脚为USB功能（在MX_USB_DEVICE_Init中会自动配置）

  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  char line[512];

  HAL_Delay(50);
  USB_SendString("Fast mode: discovery(1Hz) + acquire(each loop), ADS1115 scan range 0x48..0x4B on I2C1/I2C2.\r\n");
  g_i2c1_active_mask = DiscoverBusMask(&hi2c1);
  g_i2c2_active_mask = DiscoverBusMask(&hi2c2);
  g_last_discovery_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t any_ok = 0U;
    uint8_t had_failure = 0U;
    size_t used = 0U;
    uint32_t now = HAL_GetTick();

    line[0] = '\0';
    PollActiveMaskAndAppend("I2C1", &hi2c1, &g_i2c1_active_mask, line, sizeof(line), &used, &any_ok, &had_failure);
    PollActiveMaskAndAppend("I2C2", &hi2c2, &g_i2c2_active_mask, line, sizeof(line), &used, &any_ok, &had_failure);

    if (any_ok != 0U)
    {
      if (used < (sizeof(line) - 3U))
      {
        line[used++] = '\r';
        line[used++] = '\n';
        line[used] = '\0';
      }
      USB_SendString(line);
    }
    else
    {
      USB_SendString("no active ADS1115 in cached set; waiting for discovery\r\n");
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (any_ok != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (((now - g_last_discovery_tick) >= DISCOVERY_INTERVAL_MS) ||
        (had_failure != 0U) ||
        ((g_i2c1_active_mask == 0U) && (g_i2c2_active_mask == 0U)))
    {
      uint8_t new_i2c1 = DiscoverBusMask(&hi2c1);
      uint8_t new_i2c2 = DiscoverBusMask(&hi2c2);
      if ((new_i2c1 != g_i2c1_active_mask) || (new_i2c2 != g_i2c2_active_mask))
      {
        (void)snprintf(line, sizeof(line), "discovery: I2C1 mask=0x%02X I2C2 mask=0x%02X\r\n", new_i2c1, new_i2c2);
        USB_SendString(line);
      }
      g_i2c1_active_mask = new_i2c1;
      g_i2c2_active_mask = new_i2c2;
      g_last_discovery_tick = now;
    }

    HAL_Delay(20);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void I2C_BusReset(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    (void)HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_Delay(1);
    __HAL_RCC_I2C1_CLK_ENABLE();
    
    // I2C1 is remapped to PB8/PB9 in HAL_I2C_MspInit().
    gpio_init.Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C1_SCL_PORT, &gpio_init);
    
    // Toggle SCL to try to release a stuck slave.
    for(int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
        HAL_Delay(5);
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
        HAL_Delay(5);
    }
    
    // Generate a STOP condition before re-enabling the peripheral.
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    
    MX_I2C1_Init();
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
  * @param  line: assert_param error line source number
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
