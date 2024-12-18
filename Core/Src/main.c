/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L0X.h"
#include "stdio.h"
#include "stm32l4xx_hal_uart.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//green bouncy ball
//float Kp = 0.12;  // Proportional gain
//float Ki = 0.02;  // Integral gain
//float Kd = 0.08; // Derivative gain

//foam golf ball
float Kp = 0.14;  // Proportional gain
float Ki = 0.01;  // Integral gain
float Kd = 0.07; // Derivative gain

//golf ball
//float Kp = 0.18;  // Proportional gain
//float Ki = 0.03;  // Integral gain
//float Kd = 0.05; // Derivative gain

//ping pong ball
//float Kp = 0.08;  // Proportional gain
//float Ki = 0.02;  // Integral gain
//float Kd = 0.06; // Derivative gain

// Set a target setpoint for the distance (e.g., 100 mm)
uint16_t setpoint = 123;

// PID variables
float integral = 0.0;
float prev_error = 0.0;
uint32_t previous_time = 0;  // Previous time for derivative calculation

// Declare a static variable to store the filtered derivative value
static float derivative_filtered = 0.0f;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD

#define TOLERANCE 5.0 // 5mm tolerance for setpoint aquisition

#define DERIVATIVE_FILTER_CONSTANT 0.1f  // Adjust this to control how much filtering is applied

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t backlight_state = 1;
uint32_t lastPrintUpdate = 0;
const uint32_t printUpdateInterval = 1000;  // Print to UART every 1000 ms

uint32_t updateInterval = 20;  // Time interval for servo update (in ms)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
// Function prototype for computePID
float computePID(uint16_t setpoint, uint16_t measured_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
 uint8_t data = nibble << D4_BIT;
 data |= rs << RS_BIT;
 data |= backlight_state << BL_BIT; // Include backlight state in data
 data |= 1 << EN_BIT;
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
 HAL_Delay(1);
 data &= ~(1 << EN_BIT);
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}
void lcd_send_cmd(uint8_t cmd) {
 uint8_t upper_nibble = cmd >> 4;
 uint8_t lower_nibble = cmd & 0x0F;
 lcd_write_nibble(upper_nibble, 0);
 lcd_write_nibble(lower_nibble, 0);
 if (cmd == 0x01 || cmd == 0x02) {
 HAL_Delay(2);
 }
}
void lcd_send_data(uint8_t data) {
 uint8_t upper_nibble = data >> 4;
 uint8_t lower_nibble = data & 0x0F;
 lcd_write_nibble(upper_nibble, 1);
 lcd_write_nibble(lower_nibble, 1);
}
void lcd_init() {
 HAL_Delay(50);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(5);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x02, 0);
 lcd_send_cmd(0x28);
 lcd_send_cmd(0x0C);
 lcd_send_cmd(0x06);
 lcd_send_cmd(0x01);
 HAL_Delay(2);
}
void lcd_write_string(char *str) {
 while (*str) {
 lcd_send_data(*str++);
 }
}
void lcd_set_cursor(uint8_t row, uint8_t column) {
 uint8_t address;
 switch (row) {
 case 0:
 address = 0x00;
 break;
 case 1:
 address = 0x40;
 break;
 default:
 address = 0x00;
 }
 address += column;
 lcd_send_cmd(0x80 | address);
}
void lcd_clear(void) {
lcd_send_cmd(0x01);
 HAL_Delay(2);
}
void lcd_backlight(uint8_t state) {
 if (state) {
 backlight_state = 1;
 } else {
 backlight_state = 0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Initialise a message buffer.
//  	char msgBuffer[32];
//  	for (uint8_t i = 0; i < 32; i++) {
//  		msgBuffer[i] = ' ';
//  	}

  	// Initialise the VL53L0X
  	statInfo_t_VL53L0X distanceStr;
  	initVL53L0X(1, &hi2c1);

  	// Configure the sensor for high accuracy and speed in 20 cm.
  	setSignalRateLimit(200);
  	setVcselPulsePeriod(VcselPeriodPreRange, 10);
  	setVcselPulsePeriod(VcselPeriodFinalRange, 14);
  	//setMeasurementTimingBudget(300 * 1000UL);

  	// Set a faster measurement timing budget (lower value = faster measurements)
  	setMeasurementTimingBudget(347 * 1000UL); // measurement budget for faster updates

  	uint16_t distance;


  	// I2C pull-up resistors
  	GPIOB->PUPDR |= 0b01 << (8*2);
    GPIOB->PUPDR |= 0b01 << (9*2);
  	// Initialize the LCD
  	lcd_init();
  	lcd_backlight(1); // Turn on backlight
  	// Write a string to the LCD
  	//lcd_write_string("Hello, world!");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	uint32_t lastTime = HAL_GetTick();
    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

        // Get the current time in milliseconds
        uint32_t currentTime = HAL_GetTick();

        // Only update sensor and PID every 20ms
        if (currentTime - lastTime >= updateInterval) {
            lastTime = currentTime;

            // Read the distance from VL53L0X
            distance = readRangeSingleMillimeters(&distanceStr);

            // Calculate the PID output
            float pid_output = computePID(setpoint, distance);

            // Reverse sign of PID output for orientation of servo
            pid_output = -pid_output;

            // Convert PID output to a PWM value (ensure it's within the valid range for the servo/motor)
            uint16_t pwm_value = (uint16_t)(75 + pid_output); //75 is PWM servo midpoint
            if (pwm_value > 100) {
                pwm_value = 100;  // Maximum PWM servo limit
            } else if (pwm_value < 50) {
                pwm_value = 50;   // Minimum PWM servo limit
            }

            // Update the PWM output (control the servo)
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
        }

        // LCD update (every 1000 ms or whatever interval makes sense)
        if (currentTime - lastPrintUpdate >= printUpdateInterval) {
            lastPrintUpdate = currentTime;

            // Prepare LCD buffer
            char lcdBuffer[32];  // Buffer to hold 32 characters for each line

            // Clear the LCD first
            lcd_clear();

            // Display the Kp and Ki values on the first line
            snprintf(lcdBuffer, sizeof(lcdBuffer), "Kp:%d.%02d Ki:%d.%02d",
                     (int)(Kp), (int)(Kp * 100) % 100,
                     (int)(Ki), (int)(Ki * 100) % 100);
            lcd_set_cursor(0, 0);  // Set cursor to the beginning of the first line
            lcd_write_string(lcdBuffer);


            // Display the Kd and Setpoint (SP) on the second line
            snprintf(lcdBuffer, sizeof(lcdBuffer), "Kd:%d.%02d D:%dmm",
                     (int)(Kd), (int)(Kd * 100) % 100,
                     (distance));
            lcd_set_cursor(1, 0);  // Set cursor to the beginning of the second line
            lcd_write_string(lcdBuffer);


        }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Function to compute PID output with derivative filtering
float computePID(uint16_t setpoint, uint16_t measured_value) {
    uint32_t current_time = HAL_GetTick();  // Get the current time in ms
    float delta_time = (current_time - previous_time) / 1000.0;  // Calculate time difference in seconds
    previous_time = current_time;

    // Calculate the error
    float error = setpoint - measured_value;

    // Check if the error is within the tolerance range
    if (error >= -TOLERANCE && error <= TOLERANCE) {
        // If within tolerance, no need to adjust, set the output to 0 (or any value that makes sense)
        return 0.0;
    }

    // Proportional term
    float Pout = Kp * error;

    // Integral term
    integral += error * delta_time;
    float Iout = Ki * integral;

    // Derivative term with filtering
    float derivative_raw = (error - prev_error) / delta_time;  // Raw derivative
    // Low-pass filter to smooth the derivative term
    derivative_filtered = derivative_filtered + DERIVATIVE_FILTER_CONSTANT * (derivative_raw - derivative_filtered);
    float Dout = Kd * derivative_filtered;

    // Combine terms
    float output = Pout + Iout + Dout;

    // Store the current error for the next derivative calculation
    prev_error = error;

    return output;
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

#ifdef  USE_FULL_ASSERT
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
