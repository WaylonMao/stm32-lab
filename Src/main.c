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
 * @date           : 2023-06-06
 * @version        : 0.3
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./SYSTEM/sys.h"
#include "./LED_Driver/led.h"
#include "./SYSTEM/usart.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void key_init(void);
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
  uint8_t len;

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
  usart_init(115200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Input 1 to turn on the LED, 0 to turn off the LED.\r\n");
  while (1) {
    if (g_usart_rx_state & 0x8000) {
      len = g_usart_rx_state & 0x3fff;
      printf("Your input:\r\n");
      HAL_UART_Transmit(&g_uart1_handle, g_usart_rx_buf, len, 1000);
      printf("\r\n");
      switch (g_usart_rx_buf[0]) {
      case '1':
        printf("LED0 ON\r\n");
        LED0(0);
        break;
      case '0':
        printf("LED0 OFF\r\n");
        LED0(1);
        break;
      default:
        printf("Input error!\r\n");
        break;
      }

      while (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_TC) != 1)
        ;
      printf("\r\n");
      g_usart_rx_state = 0;
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

