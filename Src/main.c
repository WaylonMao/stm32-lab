/* USER CODE BEGIN Header */
/**
 * @name           : 5-UART-demo
 ******************************************************************************
 * @file           : main.c
 * @brief          : This is my project for learning STM32 development.
 *                   This project is based on STM32F103ZET6. The board is
 *                   manufactured by ALIENTEK tech. co., LTD. (Zhengdian atom)
 *
 *                   Tools-chain: cmake, arm-none-eabi-gcc, openOCD, st-link
 *                   IDE: CLion
 *                   Debugger & Programmer: ST-Link V2
 *
 * @author         : Weilong Mao (https://github.com/WaylonMao)
 * @date           : 2022-05-31
 * @version        : 0.1
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "./SYSTEM/sys.h"
#include "./LED_Driver/led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY0_PIN GPIO_PIN_4
#define KEY0_PORT GPIOE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_HandleTypeDef g_uart1_handle;
uint8_t g_rx_buffer[1];
uint8_t g_uart1_rx_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void key_init(void);
void uart_init(uint32_t);
void HAL_UART_MspInit(UART_HandleTypeDef *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  sys_stm32_clock_init(RCC_PLL_MUL9);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  led_init();
  key_init();
  uart_init(115200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (g_uart1_rx_state == 1) {
      /* Method 1*/
      printf("Your message: %s\r\n", g_rx_buffer);
      /* Method 2*/
      HAL_UART_Transmit(&g_uart1_handle, g_rx_buffer, 1, 1000);

      while (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_TC) != 1);
      printf("\r\n");
      g_uart1_rx_state = 0;
    }
    if (HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN) == GPIO_PIN_RESET) {
      HAL_Delay(10);
      if (HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN) == GPIO_PIN_RESET) {
        printf("Hello World!\r\n");
        do {
          HAL_Delay(10);
        } while (HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN) == GPIO_PIN_RESET);
      }
    }
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_1 */

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void key_init(void) {
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.Pin = KEY0_PIN;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEY0_PORT, &gpio_init_struct);
}

void uart_init(uint32_t baudrate) {
  /* UART initialization settings */
  g_uart1_handle.Instance = USART1;
  g_uart1_handle.Init.BaudRate = baudrate;              /* Baud rate */
  g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;  /* 8-bit data format */
  g_uart1_handle.Init.StopBits = UART_STOPBITS_1;       /* 1 stop bit */
  g_uart1_handle.Init.Parity = UART_PARITY_NONE;        /* No parity bit */
  g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;  /* No hardware flow control */
  g_uart1_handle.Init.Mode = UART_MODE_TX_RX;           /* Transmit and receive mode */
  HAL_UART_Init(&g_uart1_handle);                 /* HAL_UART_Init() will enable UART1 */

  HAL_UART_Receive_IT(&g_uart1_handle, g_rx_buffer, 1);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef gpio_init_struct;
  if (huart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_init_struct.Pin = GPIO_PIN_9;                        /* TX Pin */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;            /* IO speed set to high speed */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_10;                        /* RX Pin */
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

void USART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&g_uart1_handle);
  HAL_UART_Receive_IT(&g_uart1_handle, g_rx_buffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  g_uart1_rx_state = 1;

}

/* For MDK, redefine the fputc function, and the printf function will eventually
 * call fputc to output strings to the serial port
 */

int fputc(int ch, FILE *f) {
  while ((USART1->SR & 0X40) == 0); /* Wait for the previous character to be sent */
  USART1->DR = (uint8_t) ch; /* Write the character ch to the DR register for sending */
  return ch;
}
/*
 * For STM32CubeMX project, redefine the _write function, and the printf function will eventually
 * call _write to output strings to the serial port
 */
int _write(int file, char *ptr, int len) {
  for (int idx = 0; idx < len; idx++) {
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = (uint8_t) ptr[idx];
  }
  return len;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
