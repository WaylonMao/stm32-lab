/* USER CODE BEGIN Header */
/**
 * @name           : 2-key-input
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
 * @date           : 2022-05-29
 * @version        : 0.1
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED0_PIN GPIO_PIN_5
#define LED0_PORT GPIOB
#define LED1_PIN GPIO_PIN_5
#define LED1_PORT GPIOE
#define KEY0_PIN GPIO_PIN_4
#define KEY0_PORT GPIOE
#define KEY1_PIN GPIO_PIN_3
#define KEY1_PORT GPIOE
#define KEY_UP_PIN GPIO_PIN_0
#define KEY_UP_PORT GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void key_led(GPIO_TypeDef *, uint16_t, GPIO_TypeDef *, uint16_t, uint8_t);
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
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    key_led(KEY0_PORT, KEY0_PIN, LED0_PORT, LED0_PIN, GPIO_PIN_RESET);
    key_led(KEY1_PORT, KEY1_PIN, LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
    key_led(KEY_UP_PORT, KEY_UP_PIN, LED0_PORT, LED0_PIN, GPIO_PIN_SET);
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
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
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
  /**
   * Initialize the LED ports.
   */
  /*
   * There are two LEDs on PB5 and PE5, both negative poles are connected to GPIO pins,
   * and their positive poles are pulled up to 3.3V
   */
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.Pin = LED0_PIN; /* LED0 and LED1 both are PIN_5 */
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_PORT, &gpio_init_struct);
  HAL_GPIO_Init(LED1_PORT, &gpio_init_struct);

  /**
    * Initialize the KEY ports.
    */
  /*
   * There are 3 keys on KEY_UP(PA0), KEY_1(PE3), KEY_0(PE4). The KEY_UP is connected to high level (3.3 V),
   * KEY_0 and KEY_1 are connected to low level (GND).
   *
   * So Set KEY_UP pull down, KEY_0 and KEY_1 are pulled up.
   * And KEY_0 and KEY_1 are connected to the same GPIO port, so they can be initialized together.
   */
  gpio_init_struct.Pin = KEY0_PIN | KEY1_PIN;
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  /*
   * These two members are not changed, so no need to set again.
   * gpio_init_struct.Pull = GPIO_PULLUP;
   * gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
   */
  HAL_GPIO_Init(KEY0_PORT, &gpio_init_struct);

  gpio_init_struct.Pin = KEY_UP_PIN;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_UP_PORT, &gpio_init_struct);

  /**
   * Set the initial level of the two LEDs to high level,
   * so that the two LEDs are off at the beginning.
   */
  HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Key and LED control function
 * @param buttonPort GPIO port of the button
 * @param buttonPin GPIO pin of the button
 * @param ledPort GPIO port of the LED
 * @param ledPin GPIO pin of the LED
 * @param pressLevel The level of the button when pressed
 * @retval None
 */
void key_led(GPIO_TypeDef *buttonPort,
             uint16_t buttonPin,
             GPIO_TypeDef *ledPort,
             uint16_t ledPin,
             uint8_t pressLevel) {
  if (HAL_GPIO_ReadPin(buttonPort, buttonPin) == pressLevel) {
    HAL_Delay(10); /* Debounce */
    if (HAL_GPIO_ReadPin(buttonPort, buttonPin) == pressLevel) {
      HAL_GPIO_TogglePin(ledPort, ledPin);
      do {
        HAL_Delay(10);
      } while (HAL_GPIO_ReadPin(buttonPort, buttonPin) == pressLevel);
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
