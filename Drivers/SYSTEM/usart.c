/**
 * Serial port initialization code (USART1), support printf.
 *
 */

#include "usart.h"

/* For MDK, redefine the fputc function, and the printf function will eventually
 * call fputc to output strings to the serial port */
int fputc(int ch, FILE *f) {
  /* Wait for the previous character to be sent */
  while ((USART_UX->SR & 0X40) == 0);
  /* Write the character ch to the DR register for sending */
  USART_UX->DR = (uint8_t)ch;
  return ch;
}

int _write(int file, char *ptr, int len) {
  for (int idx = 0; idx < len; idx++) {
    while ((USART_UX->SR & 0X40) == 0);
    USART_UX->DR = (uint8_t)ptr[idx];
  }
  return len;
}

#if USART_EN_RX /* If reception is enabled */

/* Receive buffer, maximum USART_REC_LEN bytes. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/* Receive status:
 * bit15: Receive complete flag
 * bit14: Received 0x0d (carriage return)
 * bit13~0: Number of valid received bytes
 */
uint16_t g_usart_rx_state = 0;

/* USART receive buffer used by HAL library */
uint8_t g_rx_buffer[RXBUFFERSIZE];

UART_HandleTypeDef g_uart1_handle; /* UART handle */

/**
 * @brief       UART X initialization function
 * @param       baudrate: Baud rate, set the appropriate baud rate value
 *              according to your needs
 * @note        Note: You must set the correct clock source, otherwise the UART
 *              baud rate will be set incorrectly.
 *              The USART clock source has been set in the
 * sys_stm32_clock_init() function.
 * @retval
 */
void usart_init(uint32_t baudrate) {
  /* UART initialization settings */
  g_uart1_handle.Instance = USART_UX;                  /* USART_UX */
  g_uart1_handle.Init.BaudRate = baudrate;             /* Baud rate */
  g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B; /* 8-bit data format */
  g_uart1_handle.Init.StopBits = UART_STOPBITS_1;      /* 1 stop bit */
  g_uart1_handle.Init.Parity = UART_PARITY_NONE;       /* No parity bit */
  g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE; /* No hardware flow control */
  g_uart1_handle.Init.Mode = UART_MODE_TX_RX;          /* Transmit and receive mode */
  HAL_UART_Init(&g_uart1_handle);                /* HAL_UART_Init() will enable UART1 */

  /* This function enables the receive interrupt: flag UART_IT_RXNE, and sets
   * the receive buffer and the maximum data size */
  HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief       UART low-level initialization function
 * @param       huart: UART handle pointer
 * @note        This function is called by HAL_UART_Init()
 *              Completes clock enable, pin configuration, and interrupt
 * configuration
 * @retval
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef gpio_init_struct;

  /* If it is USART1, perform USART1 MSP initialization */
  if (huart->Instance == USART_UX) {
    USART_TX_GPIO_CLK_ENABLE();
    USART_RX_GPIO_CLK_ENABLE();
    USART_UX_CLK_ENABLE();

    gpio_init_struct.Pin = USART_TX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = USART_RX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);

#if USART_EN_RX
    HAL_NVIC_EnableIRQ(USART_UX_IRQn); /* Enable USART1 interrupt channel */
    /* Group 2, lowest priority: preemption priority 3, subpriority 3 */
    HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);
#endif
  }
}

/**
 * @brief       USART data reception callback function
 * @param       huart:UART handle
 * @retval
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART_UX) /* If USART1 */
  {
    if ((g_usart_rx_state & 0x8000) == 0) /* Receive not complete */
    {
      if (g_usart_rx_state & 0x4000) /* Received 0x0d (carriage return) */
      {
        if (g_rx_buffer[0] != 0x0a) /* If it is not 0x0a (line feed) */
        {
          g_usart_rx_state = 0; /* Receive error, start over */
        } else                  /* If it is 0x0a (line feed) */
        {
          g_usart_rx_state |= 0x8000; /* Receive complete */
        }
      } else /* Not received 0x0d (carriage return) yet */
      {
        if (g_rx_buffer[0] == 0x0d)
          g_usart_rx_state |= 0x4000;
        else {
          g_usart_rx_buf[g_usart_rx_state & 0X3FFF] = g_rx_buffer[0];
          g_usart_rx_state++;

          if (g_usart_rx_state > (USART_REC_LEN - 1)) {
            /* Receive data error, start receiving again */
            g_usart_rx_state = 0;
          }
        }
      }
    }
  }
}

/**
 * @brief       USART interrupt service function
 * @note        Reading USARTx->SR can avoid inexplicable errors
 * @param
 * @retval
 */
void USART_UX_IRQHandler(void) {
  HAL_UART_IRQHandler(&g_uart1_handle);
  /* Restart the interrupt and receive data */
  while (HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer,
                             RXBUFFERSIZE) != HAL_OK) {
    /* If there is an error, it will be stuck here */
  }
}
#endif
