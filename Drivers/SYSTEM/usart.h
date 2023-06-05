/**
 * Serial port initialization code (USART1), support printf.
 *
 */

#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"

/** Pin and UART Definitions
 *  By default, it is for USART1.
 *  Note: By modifying these macros, you can support any UART from USART1 to
 *  USART5.
 *
 */
#define USART_TX_GPIO_PORT GPIOA
#define USART_TX_GPIO_PIN GPIO_PIN_9
#define USART_TX_GPIO_CLK_ENABLE()                                             \
  do {                                                                         \
    __HAL_RCC_GPIOA_CLK_ENABLE();                                              \
  } while (0) /* Enable clock for GPIO Port A */

#define USART_RX_GPIO_PORT GPIOA
#define USART_RX_GPIO_PIN GPIO_PIN_10
#define USART_RX_GPIO_CLK_ENABLE()                                             \
  do {                                                                         \
    __HAL_RCC_GPIOA_CLK_ENABLE();                                              \
  } while (0) /* Enable clock for GPIO Port A */

#define USART_UX USART1
#define USART_UX_IRQn USART1_IRQn
#define USART_UX_IRQHandler USART1_IRQHandler
#define USART_UX_CLK_ENABLE()                                                  \
  do {                                                                         \
    __HAL_RCC_USART1_CLK_ENABLE();                                             \
  } while (0) /* Enable clock for USART1 */

/******************************************************************************/

#define USART_REC_LEN 200 /* Define maximum receive bytes as 200 */
#define USART_EN_RX 1     /* Enable (1) / Disable (0) USART1 reception */
#define RXBUFFERSIZE 1    /* Buffer size */

extern UART_HandleTypeDef g_uart1_handle; /* HAL UART handle */

/* Receive buffer, maximum USART_REC_LEN bytes. Last byte is a line feed. */
extern uint8_t g_usart_rx_buf[USART_REC_LEN];
extern uint16_t g_usart_rx_state;  /* Receive status flag  */
extern uint8_t g_rx_buffer[RXBUFFERSIZE]; /* HAL library USART receive buffer */

void usart_init(uint32_t bound); /* USART initialization function */

int fputc(int, FILE *);
int _write(int, char *, int);

#endif
