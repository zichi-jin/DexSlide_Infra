#include "tactile_stream.h"

#include "tactile_stream_protocol.h"
#include "ux_device_cdc_acm.h"

#include <string.h>

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t port_id;
  volatile uint32_t produced_halves;
  uint32_t consumed_halves;
  volatile uint32_t overflow_count;
  volatile uint32_t uart_error_count;
  volatile uint32_t rx_half_count;
  volatile uint32_t last_uart_error;
  volatile uint32_t last_dma_error;
  volatile uint8_t restart_requested;
} TactileStreamPort;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart11;
extern UART_HandleTypeDef huart12;

static uint8_t g_dma_buffers[TACTILE_STREAM_PORT_COUNT][TACTILE_STREAM_DMA_BUFFER_SIZE]
  __attribute__((aligned(32)));
static TactileStreamPort g_ports[TACTILE_STREAM_PORT_COUNT] = {
  { &huart1, TACTILE_STREAM_PORT_CN1_USART1, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart2, TACTILE_STREAM_PORT_CN2_USART2, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart3, TACTILE_STREAM_PORT_CN3_USART3, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart4, TACTILE_STREAM_PORT_CN4_UART4, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart5, TACTILE_STREAM_PORT_CN5_UART5, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart6, TACTILE_STREAM_PORT_CN6_USART6, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart7, TACTILE_STREAM_PORT_CN7_UART7, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart9, TACTILE_STREAM_PORT_CN8_UART9, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart10, TACTILE_STREAM_PORT_CN9_USART10, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart11, TACTILE_STREAM_PORT_CN10_USART11, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
  { &huart12, TACTILE_STREAM_PORT_CN11_UART12, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
};
static uint8_t g_record_buffer[TACTILE_STREAM_HEADER_SIZE + TACTILE_STREAM_DMA_HALF_SIZE +
                               TACTILE_STREAM_CRC_SIZE];
static uint32_t g_last_status_ms = 0U;
static uint32_t g_status_drop_count = 0U;
static uint8_t g_capture_active = 0U;
static uint8_t g_host_session_active = 0U;

static void TactileStream_ClearUartError(UART_HandleTypeDef *uart)
{
  __HAL_UART_CLEAR_FLAG(uart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
}

static TactileStreamPort *TactileStream_FindPort(const UART_HandleTypeDef *uart)
{
  uint32_t i;

  for (i = 0U; i < TACTILE_STREAM_PORT_COUNT; ++i)
  {
    if (g_ports[i].uart == uart)
    {
      return &g_ports[i];
    }
  }

  return NULL;
}

static uint16_t TactileStream_Crc16Ccitt(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;
  uint32_t i;

  for (i = 0U; i < length; ++i)
  {
    uint32_t bit;

    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }

  return crc;
}

static void TactileStream_PutLe32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
  data[2] = (uint8_t)((value >> 16) & 0xFFU);
  data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void TactileStream_BuildRecord(uint8_t type,
                                      uint8_t port_id,
                                      const uint8_t *payload,
                                      uint16_t payload_length,
                                      uint32_t *record_length)
{
  uint16_t crc;

  g_record_buffer[0] = TACTILE_STREAM_SYNC_0;
  g_record_buffer[1] = TACTILE_STREAM_SYNC_1;
  g_record_buffer[2] = TACTILE_STREAM_VERSION;
  g_record_buffer[3] = type;
  g_record_buffer[4] = port_id;
  g_record_buffer[5] = 0U;
  g_record_buffer[6] = (uint8_t)(payload_length & 0xFFU);
  g_record_buffer[7] = (uint8_t)(payload_length >> 8);
  memcpy(&g_record_buffer[TACTILE_STREAM_HEADER_SIZE], payload, payload_length);

  crc = TactileStream_Crc16Ccitt(&g_record_buffer[2],
                                 (TACTILE_STREAM_HEADER_SIZE - 2U) + payload_length);
  g_record_buffer[TACTILE_STREAM_HEADER_SIZE + payload_length] = (uint8_t)(crc & 0xFFU);
  g_record_buffer[TACTILE_STREAM_HEADER_SIZE + payload_length + 1U] = (uint8_t)(crc >> 8);
  *record_length = TACTILE_STREAM_HEADER_SIZE + payload_length + TACTILE_STREAM_CRC_SIZE;
}

static UINT TactileStream_QueueRecord(uint8_t type, uint8_t port_id, const uint8_t *payload, uint16_t payload_length)
{
  uint32_t record_length;

  if ((payload == NULL) || (payload_length > TACTILE_STREAM_DMA_HALF_SIZE))
  {
    return UX_ERROR;
  }

  TactileStream_BuildRecord(type, port_id, payload, payload_length, &record_length);
  return USBD_CDC_ACM_Write(g_record_buffer, record_length);
}

static void TactileStream_DrainPort(uint32_t index)
{
  TactileStreamPort *port = &g_ports[index];
  uint32_t produced = port->produced_halves;
  uint32_t pending = produced - port->consumed_halves;

  /* At the next HT/TC event, the previous half starts being overwritten. */
  if (pending > 1U)
  {
    uint32_t lost_halves = pending - 1U;

    port->overflow_count += lost_halves;
    port->consumed_halves = produced - 1U;
  }

  while (port->consumed_halves != produced)
  {
    uint32_t half_index = port->consumed_halves & 1U;
    const uint8_t *data = &g_dma_buffers[index][half_index * TACTILE_STREAM_DMA_HALF_SIZE];
    uint32_t record_length;

    TactileStream_BuildRecord(TACTILE_STREAM_RECORD_DATA,
                              port->port_id,
                              data,
                              TACTILE_STREAM_DMA_HALF_SIZE,
                              &record_length);
    if (port->produced_halves != produced)
    {
      uint32_t latest_produced = port->produced_halves;

      port->overflow_count += latest_produced - port->consumed_halves - 1U;
      port->consumed_halves = latest_produced - 1U;
      return;
    }
    if (USBD_CDC_ACM_Write(g_record_buffer, record_length) != UX_SUCCESS)
    {
      return;
    }

    port->consumed_halves++;
  }
}

static void TactileStream_QueueStatus(void)
{
  uint8_t payload[TACTILE_STREAM_STATUS_PAYLOAD_SIZE] = {0};
  uint32_t i;

  payload[0] = TACTILE_STREAM_STATUS_VERSION;
  payload[1] = TACTILE_STREAM_PORT_COUNT;

  for (i = 0U; i < TACTILE_STREAM_PORT_COUNT; ++i)
  {
    TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_OVERFLOW_OFFSET + (i * 4U)],
                          g_ports[i].overflow_count);
    TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_ERROR_COUNT_OFFSET + (i * 4U)],
                          g_ports[i].uart_error_count);
    TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_RX_HALF_OFFSET + (i * 4U)],
                          g_ports[i].rx_half_count);
    TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_LAST_ERROR_OFFSET + (i * 4U)],
                          g_ports[i].last_uart_error);
    TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_DMA_ERROR_OFFSET + (i * 4U)],
                          g_ports[i].last_dma_error);
  }
  TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_USB_HIGH_WATER_OFFSET],
                        USBD_CDC_ACM_GetTxHighWaterMark());
  TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_USB_PENDING_OFFSET],
                        USBD_CDC_ACM_GetTxPending());
  TactileStream_PutLe32(&payload[TACTILE_STREAM_STATUS_DROP_COUNT_OFFSET],
                        g_status_drop_count);

  if (TactileStream_QueueRecord(TACTILE_STREAM_RECORD_STATUS,
                                TACTILE_STREAM_SYSTEM_PORT_ID,
                                payload,
                                sizeof(payload)) != UX_SUCCESS)
  {
    g_status_drop_count++;
  }
}

static void TactileStream_ServiceRestarts(void)
{
  uint32_t i;

  for (i = 0U; i < TACTILE_STREAM_PORT_COUNT; ++i)
  {
    TactileStreamPort *port = &g_ports[i];

    if (port->restart_requested == 0U)
    {
      continue;
    }

    /* 前一次中止完成前，不能改写仍可能执行中的 GPDMA linked-list 节点。 */
    if ((port->uart->hdmarx == NULL) ||
        (port->uart->RxState != HAL_UART_STATE_READY) ||
        (HAL_DMA_GetState(port->uart->hdmarx) != HAL_DMA_STATE_READY) ||
        ((port->uart->hdmarx->LinkedListQueue != NULL) &&
         (port->uart->hdmarx->LinkedListQueue->State == HAL_DMA_QUEUE_STATE_BUSY)))
    {
      continue;
    }

    TactileStream_ClearUartError(port->uart);
    if (HAL_UART_Receive_DMA(port->uart, g_dma_buffers[i], TACTILE_STREAM_DMA_BUFFER_SIZE) == HAL_OK)
    {
      port->produced_halves = 0U;
      port->consumed_halves = 0U;
      port->restart_requested = 0U;
    }
  }
}

static void TactileStream_Stop(void)
{
  uint32_t i;

  for (i = 0U; i < TACTILE_STREAM_PORT_COUNT; ++i)
  {
    (void)HAL_UART_DMAStop(g_ports[i].uart);
    g_ports[i].produced_halves = 0U;
    g_ports[i].consumed_halves = 0U;
    g_ports[i].restart_requested = 0U;
  }

  g_capture_active = 0U;
  g_host_session_active = 0U;
}

static void TactileStream_BeginHostSession(void)
{
  uint32_t i;

  /* Discard data received while the previous CDC client was disconnected. */
  for (i = 0U; i < TACTILE_STREAM_PORT_COUNT; ++i)
  {
    g_ports[i].consumed_halves = g_ports[i].produced_halves;
    g_ports[i].overflow_count = 0U;
    g_ports[i].uart_error_count = 0U;
    g_ports[i].rx_half_count = 0U;
    g_ports[i].last_uart_error = HAL_UART_ERROR_NONE;
    g_ports[i].last_dma_error = HAL_DMA_ERROR_NONE;
  }

  g_status_drop_count = 0U;
  g_last_status_ms = HAL_GetTick() - 1000U;
  g_host_session_active = 1U;
}

void TactileStream_Start(void)
{
  g_capture_active = 0U;
  g_host_session_active = 0U;
  g_last_status_ms = HAL_GetTick();
}

void TactileStream_Run(void)
{
  uint32_t i;
  uint32_t now;

  if (USBD_CDC_ACM_IsReady() != UX_TRUE)
  {
    if (g_capture_active != 0U)
    {
      TactileStream_Stop();
    }
    return;
  }

  if (g_capture_active == 0U)
  {
    for (i = 0U; i < TACTILE_STREAM_PORT_COUNT; ++i)
    {
      g_ports[i].produced_halves = 0U;
      g_ports[i].consumed_halves = 0U;
      g_ports[i].overflow_count = 0U;
      g_ports[i].uart_error_count = 0U;
      g_ports[i].rx_half_count = 0U;
      g_ports[i].last_uart_error = HAL_UART_ERROR_NONE;
      g_ports[i].last_dma_error = HAL_DMA_ERROR_NONE;
      g_ports[i].restart_requested = 0U;
      TactileStream_ClearUartError(g_ports[i].uart);
      if (HAL_UART_Receive_DMA(g_ports[i].uart,
                               g_dma_buffers[i],
                               TACTILE_STREAM_DMA_BUFFER_SIZE) != HAL_OK)
      {
        Error_Handler();
      }
    }
    g_status_drop_count = 0U;
    g_last_status_ms = HAL_GetTick() - 1000U;
    g_capture_active = 1U;
  }

  TactileStream_ServiceRestarts();

  if (USBD_CDC_ACM_IsHostOpen() != UX_TRUE)
  {
    /*
     * Do not abort the 11 GPDMA channels merely because a host application
     * exits. Aborting then restarting all channels is not restart-safe on
     * this H5 linked-list configuration. The next host session is aligned to
     * the newest complete DMA half instead.
     */
    g_host_session_active = 0U;
    return;
  }

  if (g_host_session_active == 0U)
  {
    TactileStream_BeginHostSession();
  }

  now = HAL_GetTick();
  if ((now - g_last_status_ms) >= 1000U)
  {
    TactileStream_QueueStatus();
    g_last_status_ms = now;
  }

  for (i = 0U; i < TACTILE_STREAM_PORT_COUNT; ++i)
  {
    TactileStream_DrainPort(i);
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  TactileStreamPort *port = TactileStream_FindPort(huart);

  if (port != NULL)
  {
    port->produced_halves++;
    port->rx_half_count++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  TactileStreamPort *port = TactileStream_FindPort(huart);

  if (port != NULL)
  {
    port->produced_halves++;
    port->rx_half_count++;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  TactileStreamPort *port = TactileStream_FindPort(huart);

  if (port != NULL)
  {
    port->uart_error_count++;
    port->last_uart_error = huart->ErrorCode;
    port->last_dma_error = (huart->hdmarx != NULL) ? HAL_DMA_GetError(huart->hdmarx) : HAL_DMA_ERROR_NONE;
    TactileStream_ClearUartError(huart);
    port->restart_requested = 1U;
  }
}
