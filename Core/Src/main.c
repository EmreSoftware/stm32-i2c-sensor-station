/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  *
  * @project        : I2C Multi-Sensor Project
  * @author         : EmreSoftware
  *
  * @description    : Reads MPU6050 and BMP180 sensors via I2C with DMA,
  *                   displays on 16x2 LCD with PCF8574 backpack.
  *
  * @hardware       : STM32F103C8T6, MPU6050, BMP180, 16x2 LCD
  * @pins           : PB6 (SCL), PB7 (SDA), 4.7kΩ pull-ups
  *
  ******************************************************************************

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t i2c_rx_done = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LCD_ADDR (0x27 << 1)
#define LCD_BL 0x08 // Bit 3. When this bit is 1, backlight turns on.
#define LCD_EN 0x04 // Bit 2. This is read now signal for LCD.
#define LCD_RS 0x01 // Bit 0. When 0 = sending command.

void lcd_send_nibble(uint8_t nibble, uint8_t rs) // This sends 4 bits to the LCD.
{
    uint8_t data = (nibble & 0xF0) | LCD_BL | rs;

    data |= LCD_EN; // set enable bit high
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, 10);

    data &= ~LCD_EN; // set enable bit low
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, 10);
}

void lcd_send_byte(uint8_t byte, uint8_t rs)
{
    lcd_send_nibble(byte & 0xF0, rs);
    lcd_send_nibble((byte << 4) & 0xF0, rs);
}

void lcd_cmd(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
    HAL_Delay(2);
}

void lcd_char(char c)
{
    lcd_send_byte(c, LCD_RS); // LCD_RS = char data
}

void lcd_string(char *str)
{
    while (*str) lcd_char(*str++);
}

void lcd_init(void)
{
    HAL_Delay(50);
    lcd_send_nibble(0x30, 0); HAL_Delay(5); // Datasheet says send wake up command three times to reset the LCD
    lcd_send_nibble(0x30, 0); HAL_Delay(1); // and wake it up.
    lcd_send_nibble(0x30, 0); HAL_Delay(1);

    lcd_send_nibble(0x20, 0); HAL_Delay(1); // switch to 4 bit mode

    lcd_cmd(0x28);  // 4-bit, 2 lines, 5x8 font
    lcd_cmd(0x0C);  // display on, cursor off
    lcd_cmd(0x06);  // entry mode: increment
    lcd_cmd(0x01);  // clear display
    HAL_Delay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? col : (0x40 + col);
    lcd_cmd(0x80 | addr);
}

// BMP180 calibration data
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;

void bmp180_read_calibration(void)
{
    uint8_t data[22];
    HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0xAA, 1, data, 22, 100);

    AC1 = (data[0] << 8) | data[1];
    AC2 = (data[2] << 8) | data[3];
    AC3 = (data[4] << 8) | data[5];
    AC4 = (data[6] << 8) | data[7];
    AC5 = (data[8] << 8) | data[9];
    AC6 = (data[10] << 8) | data[11];
    B1 = (data[12] << 8) | data[13];
    B2 = (data[14] << 8) | data[15];
    MB = (data[16] << 8) | data[17];
    MC = (data[18] << 8) | data[19];
    MD = (data[20] << 8) | data[21];
}

long bmp180_read_temp(void)
{
    uint8_t cmd = 0x2E;
    uint8_t data[2];

    HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0xF4, 1, &cmd, 1, 100);
    HAL_Delay(5);
    HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0xF6, 1, data, 2, 100);

    return (data[0] << 8) | data[1];
}

long bmp180_read_pressure(void)
{
    uint8_t cmd = 0x34;
    uint8_t data[3];

    HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0xF4, 1, &cmd, 1, 100);
    HAL_Delay(5);
    HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0xF6, 1, data, 3, 100);

    return ((data[0] << 16) | (data[1] << 8) | data[2]) >> 8;
}

void bmp180_get_temp_pressure(long *temp, long *pressure)
{
    long UT = bmp180_read_temp();
    long UP = bmp180_read_pressure();

    // Temperature calculation
    long X1 = ((UT - AC6) * AC5) >> 15;
    long X2 = (MC << 11) / (X1 + MD);
    long B5 = X1 + X2;
    *temp = (B5 + 8) >> 4;  // temperature in 0.1°C

    // Pressure calculation
    long B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    long X3 = X1 + X2;
    long B3 = (((AC1 * 4 + X3)) + 2) >> 2;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    unsigned long B4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
    unsigned long B7 = ((unsigned long)UP - B3) * 50000;

    if (B7 < 0x80000000)
        *pressure = (B7 * 2) / B4;
    else
        *pressure = (B7 / B4) * 2;

    X1 = (*pressure >> 8) * (*pressure >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * *pressure) >> 16;
    *pressure = *pressure + ((X1 + X2 + 3791) >> 4);  // pressure in Pa
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
  {
      i2c_rx_done = 1;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Wake up MPU6050
  uint8_t data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, 1, &data, 1, 100);
  HAL_Delay(100);

  // Variables for sensor data
  volatile int16_t accel_x, accel_y, accel_z;
  volatile int16_t gyro_x, gyro_y, gyro_z;
  uint8_t buffer[6];

  long temperature, pressure;

  // Init LCD
  lcd_init();

  // Intro animation
  lcd_set_cursor(0, 0);
  lcd_string("  I2C Scanner  ");
  HAL_Delay(1000);

  lcd_cmd(0x01);  // clear
  HAL_Delay(5);

  // Check MPU6050
  lcd_set_cursor(0, 0);
  lcd_string("MPU6050...");
  HAL_Delay(300);
  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x68 << 1, 1, 100) == HAL_OK)
  {
      lcd_string(" OK!");
  }
  else
  {
      lcd_string(" FAIL");
  }
  HAL_Delay(500);

  // Check LCD (always OK if you see this)
  lcd_set_cursor(1, 0);
  lcd_string("LCD.......");
  HAL_Delay(300);
  lcd_string(" OK!");
  HAL_Delay(500);

  lcd_cmd(0x01);
  HAL_Delay(5);

  // Check BMP180
  lcd_set_cursor(0, 0);
  lcd_string("BMP180....");
  HAL_Delay(300);
  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x77 << 1, 1, 100) == HAL_OK)
  {
      lcd_string(" OK!");
  }
  else
  {
      lcd_string(" FAIL");
  }
  HAL_Delay(500);


  lcd_set_cursor(1, 0);
  lcd_string("All systems GO!");
  HAL_Delay(1500);

  lcd_cmd(0x01);
  HAL_Delay(5);

  // Wake up MPU6050
  HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, 1, &data, 1, 100);
  HAL_Delay(100);

  // Read BMP180 calibration
  bmp180_read_calibration();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	i2c_rx_done = 0;
	    HAL_I2C_Mem_Read_DMA(&hi2c1, 0x68 << 1, 0x3B, 1, buffer, 6);
	    while (!i2c_rx_done); // wait DMA to read the data.

	      accel_x = (buffer[0] << 8) | buffer[1];
	      accel_y = (buffer[2] << 8) | buffer[3];
	      accel_z = (buffer[4] << 8) | buffer[5];

	      if (accel_x == 0 && accel_y == 0 && accel_z == 0)
	      {
	          uint8_t wake = 0x00;
	          HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, 1, &wake, 1, 100);
	          HAL_Delay(50);
	      }
	      i2c_rx_done = 0;
	      HAL_I2C_Mem_Read_DMA(&hi2c1, 0x68 << 1, 0x43, 1, buffer, 6);
	      while (!i2c_rx_done);
	      gyro_x = (buffer[0] << 8) | buffer[1];
	      gyro_y = (buffer[2] << 8) | buffer[3];
	      gyro_z = (buffer[4] << 8) | buffer[5];

	      bmp180_get_temp_pressure(&temperature, &pressure);

	      char line[17];

	      lcd_set_cursor(0, 0);
	      sprintf(line, "T:%ld.%ldC P:%ld", temperature/10, temperature%10, pressure/100);
	      lcd_string(line);

	      lcd_set_cursor(1, 0);
	      sprintf(line, "A:%+06d %+06d", accel_x, accel_y);
	      lcd_string(line);

	      HAL_Delay(500);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
