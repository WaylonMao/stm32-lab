/* USER CODE BEGIN Header */
/**
 * @name           : 10-time-capture
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
 * @date           : 2023-06-13
 * @version        : 0.1
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED_Driver/led.h"
#include "SYSTEM/sys.h"
#include "SYSTEM/usart.h"
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
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t start_time = 0;
uint32_t end_time = 0;
uint8_t cap_state = 0;
uint16_t cap_val = 0;
uint32_t overflow_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  sys_stm32_clock_init(RCC_PLL_MUL9);
  led_init();
  usart_init(115200);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  HAL_TIM_IC_MspInit(&htim5);
  /* USER CODE BEGIN 2 */
  uint32_t total_time = 0;
  uint8_t count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (cap_state & 0X80) {           // Get a complete capture.
      LED1_TOGGLE();
      total_time = cap_state & 0X3F;        // Get the times of overflows.
      total_time *= 65536;                  // Calculate the total overflow time.
      total_time += cap_val;                // Calculate the total time.
      printf("Press Time: %d us\r\n", total_time); // Print the total time.
      cap_state = 0;                  // Reset the capture state.
    }
    count++;
    if (count > 20) {
      count = 0;
      LED0_TOGGLE();                  // Toggle the LED0 every 200ms.
    }
    HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
// void SystemClock_Config(void)
//{
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }
//
//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void) {

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 71;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
/* USER CODE BEGIN MX_GPIO_Init_1 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM5) {
    GPIO_InitTypeDef gpio_init_struct;

    gpio_init_struct.Pin = GPIO_PIN_0;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }
}

void TIM5_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim5);
}

/**
 * @brief Callback function for TIM5.
 * @param htim Handle of the timer.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM5) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      if ((cap_state & 0X80) == 0) {                                                // Haven't captured.
        if (cap_state & 0X40) {                                                     // Captured the falling edge.
          cap_state |= 0X80;                                                        // Mark captured.
          cap_val = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);  // Get the value.
          TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1);                         // Clear the polarity.
          TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);    // Set the polarity to rising edge.
        } else {                                                                    // First capture.
          cap_state = 0;
          cap_val = 0;
          cap_state |= 0X40;                                                // Mark the rising edge has been captured.
          __HAL_TIM_DISABLE(&htim5);                                                // Disable the timer.
          __HAL_TIM_SET_COUNTER(&htim5, 0);                                         // Clear the counter.
          TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1);                         // Clear the polarity.
          TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);   // Set the polarity to falling edge.
          __HAL_TIM_ENABLE(&htim5);                                                 // Enable the timer.
        }
      }
    }
  }
}

/**
 * @brief Callback function for timer overflow.
 * @param htim Handle of the timer.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM5) {
    if ((cap_state & 0X80) == 0) {                          // Haven't captured.
      if (cap_state & 0X40) {                               // Captured the rising edge.
        if ((cap_state & 0X3F) == 0X3F) {                   // Overflowed 63 times or more.
          TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1); // Clear the polarity.
          TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);    // Set the polarity to rising edge.
          cap_state |= 0X80;                                // Mark captured.
          cap_val = 0XFFFF;
        } else {
          cap_state++;                                      // Increment the overflow counter.
        }
      }
    }
  }
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
