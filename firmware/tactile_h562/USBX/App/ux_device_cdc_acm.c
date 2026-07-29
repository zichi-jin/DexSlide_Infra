/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CDC_BRIDGE_TX_FIFO_SIZE   65536U
/* USBX owns a 512-byte class buffer and splits this into 64-byte FS packets. */
#define CDC_BRIDGE_TX_TRANSFER_SIZE 512U
#define CDC_BRIDGE_RX_FIFO_SIZE    2048U
#define CDC_BRIDGE_RX_PACKET_SIZE    64U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UX_SLAVE_CLASS_CDC_ACM *g_cdc_acm = UX_NULL;
static UCHAR g_tx_fifo[CDC_BRIDGE_TX_FIFO_SIZE];
static UCHAR g_tx_packet[CDC_BRIDGE_TX_TRANSFER_SIZE];
static UCHAR g_rx_fifo[CDC_BRIDGE_RX_FIFO_SIZE];
static UCHAR g_rx_packet[CDC_BRIDGE_RX_PACKET_SIZE];
static ULONG g_tx_head = 0U;
static ULONG g_tx_tail = 0U;
static ULONG g_tx_active_length = 0U;
static ULONG g_tx_high_water = 0U;
static ULONG g_rx_head = 0U;
static ULONG g_rx_tail = 0U;
static volatile UINT g_host_dtr = UX_FALSE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static UINT USBD_CDC_ACM_IsConfigured(VOID);
static VOID USBD_CDC_ACM_ResetState(VOID);
static ULONG USBD_CDC_ACM_TxUsed(VOID);
static ULONG USBD_CDC_ACM_TxFree(VOID);
static ULONG USBD_CDC_ACM_RxUsed(VOID);
static ULONG USBD_CDC_ACM_RxFree(VOID);
static UINT USBD_CDC_ACM_QueueTxBuffer(const UCHAR *data, ULONG len);
static UINT USBD_CDC_ACM_QueueRxBuffer(const UCHAR *data, ULONG len);
static UINT USBD_CDC_ACM_PopRxBuffer(UCHAR *data, ULONG capacity, ULONG *actual_length);
static VOID USBD_CDC_ACM_ServiceTx(VOID);
static VOID USBD_CDC_ACM_ServiceRx(VOID);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static UINT USBD_CDC_ACM_IsConfigured(VOID)
{
  UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;

  return ((g_cdc_acm != UX_NULL) && (device->ux_slave_device_state == UX_DEVICE_CONFIGURED)) ? UX_TRUE : UX_FALSE;
}

static VOID USBD_CDC_ACM_ResetState(VOID)
{
  g_tx_head = 0U;
  g_tx_tail = 0U;
  g_tx_active_length = 0U;
  g_tx_high_water = 0U;
  g_rx_head = 0U;
  g_rx_tail = 0U;
}

static ULONG USBD_CDC_ACM_TxUsed(VOID)
{
  if (g_tx_head >= g_tx_tail)
  {
    return g_tx_head - g_tx_tail;
  }

  return (CDC_BRIDGE_TX_FIFO_SIZE - g_tx_tail) + g_tx_head;
}

static ULONG USBD_CDC_ACM_TxFree(VOID)
{
  return (CDC_BRIDGE_TX_FIFO_SIZE - 1U) - USBD_CDC_ACM_TxUsed();
}

static ULONG USBD_CDC_ACM_RxUsed(VOID)
{
  if (g_rx_head >= g_rx_tail)
  {
    return g_rx_head - g_rx_tail;
  }

  return (CDC_BRIDGE_RX_FIFO_SIZE - g_rx_tail) + g_rx_head;
}

static ULONG USBD_CDC_ACM_RxFree(VOID)
{
  return (CDC_BRIDGE_RX_FIFO_SIZE - 1U) - USBD_CDC_ACM_RxUsed();
}

static UINT USBD_CDC_ACM_QueueTxBuffer(const UCHAR *data, ULONG len)
{
  ULONG i;
  ULONG used;

  if (len > USBD_CDC_ACM_TxFree())
  {
    return UX_ERROR;
  }

  for (i = 0U; i < len; ++i)
  {
    g_tx_fifo[g_tx_head] = data[i];
    g_tx_head = (g_tx_head + 1U) % CDC_BRIDGE_TX_FIFO_SIZE;
  }

  used = USBD_CDC_ACM_TxUsed();
  if (used > g_tx_high_water)
  {
    g_tx_high_water = used;
  }

  return UX_SUCCESS;
}

static UINT USBD_CDC_ACM_QueueRxBuffer(const UCHAR *data, ULONG len)
{
  ULONG i;

  if (len > USBD_CDC_ACM_RxFree())
  {
    return UX_ERROR;
  }

  for (i = 0U; i < len; ++i)
  {
    g_rx_fifo[g_rx_head] = data[i];
    g_rx_head = (g_rx_head + 1U) % CDC_BRIDGE_RX_FIFO_SIZE;
  }

  return UX_SUCCESS;
}

static UINT USBD_CDC_ACM_PopRxBuffer(UCHAR *data, ULONG capacity, ULONG *actual_length)
{
  ULONG count = 0U;

  while ((count < capacity) && (g_rx_head != g_rx_tail))
  {
    data[count++] = g_rx_fifo[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1U) % CDC_BRIDGE_RX_FIFO_SIZE;
  }

  *actual_length = count;
  return UX_SUCCESS;
}

static VOID USBD_CDC_ACM_ServiceTx(VOID)
{
  UINT status;
  ULONG actual_length = 0U;
  ULONG available;

  if (g_tx_active_length == 0U)
  {
    if (g_tx_head == g_tx_tail)
    {
      return;
    }

    if (g_tx_head > g_tx_tail)
    {
      available = g_tx_head - g_tx_tail;
    }
    else
    {
      available = CDC_BRIDGE_TX_FIFO_SIZE - g_tx_tail;
    }

    g_tx_active_length = (available > CDC_BRIDGE_TX_TRANSFER_SIZE) ? CDC_BRIDGE_TX_TRANSFER_SIZE : available;
    memcpy(g_tx_packet, &g_tx_fifo[g_tx_tail], g_tx_active_length);
  }

  status = ux_device_class_cdc_acm_write_run(g_cdc_acm, g_tx_packet, g_tx_active_length, &actual_length);

  if (status == UX_STATE_NEXT)
  {
    g_tx_tail = (g_tx_tail + g_tx_active_length) % CDC_BRIDGE_TX_FIFO_SIZE;
    g_tx_active_length = 0U;
  }
  else if ((status == UX_STATE_ERROR) || (status == UX_STATE_EXIT))
  {
    g_tx_active_length = 0U;
  }
}

static VOID USBD_CDC_ACM_ServiceRx(VOID)
{
  UINT status;
  ULONG actual_length = 0U;
  ULONG requested_length = USBD_CDC_ACM_RxFree();

  if (requested_length == 0U)
  {
    return;
  }

  if (requested_length > CDC_BRIDGE_RX_PACKET_SIZE)
  {
    requested_length = CDC_BRIDGE_RX_PACKET_SIZE;
  }

  status = ux_device_class_cdc_acm_read_run(g_cdc_acm, g_rx_packet, requested_length, &actual_length);

  if ((status == UX_STATE_NEXT) && (actual_length > 0U))
  {
    (void)USBD_CDC_ACM_QueueRxBuffer(g_rx_packet, actual_length);
  }
}

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
  g_cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;
  g_host_dtr = UX_FALSE;
  USBD_CDC_ACM_ResetState();
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  g_cdc_acm = UX_NULL;
  g_host_dtr = UX_FALSE;
  USBD_CDC_ACM_ResetState();
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;

  if (cdc_acm != UX_NULL)
  {
    g_host_dtr = (cdc_acm->ux_slave_class_cdc_acm_data_dtr_state != 0U) ? UX_TRUE : UX_FALSE;
  }
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */
UINT USBD_CDC_ACM_IsReady(VOID)
{
  return USBD_CDC_ACM_IsConfigured();
}

UINT USBD_CDC_ACM_IsHostOpen(VOID)
{
  return ((USBD_CDC_ACM_IsConfigured() == UX_TRUE) && (g_host_dtr == UX_TRUE)) ? UX_TRUE : UX_FALSE;
}

UINT USBD_CDC_ACM_Write(const UCHAR *data, ULONG len)
{
  if ((data == UX_NULL) || (len == 0U) || (USBD_CDC_ACM_IsConfigured() != UX_TRUE))
  {
    return UX_ERROR;
  }

  return USBD_CDC_ACM_QueueTxBuffer(data, len);
}

UINT USBD_CDC_ACM_Read(UCHAR *data, ULONG capacity, ULONG *actual_length)
{
  if ((actual_length == UX_NULL) || ((data == UX_NULL) && (capacity > 0U)))
  {
    return UX_ERROR;
  }

  *actual_length = 0U;

  if (capacity == 0U)
  {
    return UX_SUCCESS;
  }

  if (USBD_CDC_ACM_IsConfigured() != UX_TRUE)
  {
    return UX_ERROR;
  }

  return USBD_CDC_ACM_PopRxBuffer(data, capacity, actual_length);
}

ULONG USBD_CDC_ACM_GetTxHighWaterMark(VOID)
{
  return g_tx_high_water;
}

ULONG USBD_CDC_ACM_GetTxPending(VOID)
{
  return USBD_CDC_ACM_TxUsed();
}

VOID USBD_CDC_ACM_Run(VOID)
{
  if (USBD_CDC_ACM_IsConfigured() == UX_TRUE)
  {
    USBD_CDC_ACM_ServiceRx();
    USBD_CDC_ACM_ServiceTx();
  }
}

/* USER CODE END 1 */
