/* USER CODE BEGIN Header */
// ... (your existing header)
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "bmp280.h" // YOUR PROVIDED BMP280 DRIVER HEADER
#include <stdio.h>
#include <string.h>
#include <stdbool.h> // For bool type
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
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp_handle;    // BMP280 device handle
bmp280_params_t bmp_sensor_params;  // Parameters for BMP280

char oled_buf[32]; // Buffer for OLED strings

bool first_display_occurred = false; // Flag to handle immediate first display
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize OLED display - essential for it to work.
  // OLED screen will remain blank until the first data is ready.
  ssd1306_Init();

  printf("System Booting... Initializing BMP280...\r\n"); // UART only for diagnostics

  // --- BMP280 Initialization ---
  bmp_handle.i2c = &hi2c1;
  bmp_handle.addr = BMP280_I2C_ADDRESS_0; // Ensure this is defined in bmp280.h (e.g. 0x76)

  bmp280_init_default_params(&bmp_sensor_params);

  bmp_sensor_params.mode = BMP280_MODE_NORMAL;
  bmp_sensor_params.filter = BMP280_FILTER_OFF;
  bmp_sensor_params.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
  bmp_sensor_params.oversampling_temperature = BMP280_LOW_POWER;
  bmp_sensor_params.standby = BMP280_STANDBY_62;

  if (!bmp280_init(&bmp_handle, &bmp_sensor_params)) {
      printf("BMP280 Initialization Failed! Driver returned false. Chip ID read: 0x%02X. Addr: 0x%02X\r\n", bmp_handle.id, bmp_handle.addr << 1); // UART only
      while (1) { // Halt with LED blink on critical error
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          HAL_Delay(200);
      }
  } else {
      printf("BMP280 Initialized Successfully. Chip ID from handle: 0x%02X. Addr: 0x%02X\r\n", bmp_handle.id, bmp_handle.addr << 1); // UART only
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  #define BUFFER_SIZE 5
  float temp_buffer[BUFFER_SIZE] = {0};
  float press_buffer[BUFFER_SIZE] = {0};
  uint8_t buffer_index = 0;
  uint8_t sample_count = 0;

  uint32_t last_sample_time = HAL_GetTick(); // Initialize to start sampling immediately
  uint32_t last_display_time = HAL_GetTick(); // Used for 5s interval *between* displays

  while (1)
  {
    uint32_t now = HAL_GetTick();

    // Sample data every 1000ms (1 second)
    if (now - last_sample_time >= 1000)
    {
        last_sample_time = now;

        float current_temp = 0.0f, current_press = 0.0f, current_hum = 0.0f; // hum is dummy for BMP280
        if (bmp280_read_float(&bmp_handle, &current_temp, &current_press, &current_hum))
        {
            // Convert pressure from Pa to hPa
            current_press /= 100.0f;

            temp_buffer[buffer_index] = current_temp;
            press_buffer[buffer_index] = current_press;

            buffer_index = (buffer_index + 1) % BUFFER_SIZE;
            if (sample_count < BUFFER_SIZE)
            {
                sample_count++;
            }
        }
        else
        {
            printf("Failed to read from BMP280\r\n"); // UART only
        }
    }

    // Conditions for display update
    bool buffer_full = (sample_count == BUFFER_SIZE);
    bool display_interval_elapsed = (now - last_display_time >= 5000);

    // Display on OLED if buffer is full AND (it's the first display OR 5s interval has passed)
    if (buffer_full && (!first_display_occurred || display_interval_elapsed))
    {
        last_display_time = now; // Reset display timer
        if (!first_display_occurred) {
            first_display_occurred = true;
        }

        float temp_avg = 0.0f, press_avg = 0.0f;
        for (int i = 0; i < BUFFER_SIZE; i++)
        {
            temp_avg += temp_buffer[i];
            press_avg += press_buffer[i];
        }
        temp_avg /= BUFFER_SIZE;
        press_avg /= BUFFER_SIZE;

        // OLED Display: ONLY Temperature and Pressure
        ssd1306_Fill(Black); // Clear previous content

        snprintf(oled_buf, sizeof(oled_buf), "T: %.2f C", temp_avg);
        ssd1306_SetCursor(0, 0); // Position for Temperature
        ssd1306_WriteString(oled_buf, Font_7x10, White);

        snprintf(oled_buf, sizeof(oled_buf), "P: %.2f hPa", press_avg);
        ssd1306_SetCursor(0, 12); // Position for Pressure
        ssd1306_WriteString(oled_buf, Font_7x10, White);

        ssd1306_UpdateScreen();

        // UART Transmission (Temperature and Pressure)
        char uart_buf[64];
        int len = snprintf(uart_buf, sizeof(uart_buf), "AVG T: %.2f C, P: %.2f hPa\r\n", temp_avg, press_avg);
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);
    }
    // If 5s interval elapsed but buffer is not yet full (e.g. initial collection taking time, or sensor read issues)
    // OR if it's simply not time for the next display yet but we want periodic UART updates for collection status.
    // This 'else if' is primarily for UART status updates during initial collection if it spans a 5s mark.
    else if (display_interval_elapsed && !buffer_full)
    {
        last_display_time = now; // Reset this timer to avoid spamming UART if collection is slow
        printf("Collecting data (%d/%d samples)...\r\n", sample_count, BUFFER_SIZE); // UART only
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  hi2c1.Init.Timing = 0x00000608;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("!!! HAL Error Handler Called !!!\r\n"); // UART only
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  printf("Wrong parameters value: file %s on line %lu\r\n", file, line); // UART only
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
