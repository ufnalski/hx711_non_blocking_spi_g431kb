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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_UPDATE_PERIOD 200

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define CALIBRATION_X1 78070 // tare
#define CALIBRATION_Y1 0.0 // grams
#define CALIBRATION_X2 -81080 // loaded
#define CALIBRATION_Y2 222.22 // grams
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t TxData[8] =
{ 0b0000000, 0x00000001, 0b01010101, 0b01010101, 0b01010101, 0b01010101,
		0b01010101, 0b01010101 };  // clock, 25 ticks
uint8_t RxData[8];
uint32_t raw_reading;
double converted_reading;

union
{
	uint32_t uint32_value;
	int32_t int32_value;
} uint2int_converter;

volatile uint8_t sensor_read_flag = 0;
volatile uint8_t yellow_button_flag = 0;

double measured_weight;

uint8_t UartMessage[256];
uint16_t MessageLength;

char lcd_line[32];
uint32_t SoftTimerLCD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	ssd1306_Init();

	ssd1306_Fill(Black);
	ssd1306_SetCursor(10, 2);
	ssd1306_WriteString("ufnalski.edu.pl", Font_7x10, White);
	ssd1306_SetCursor(5, 15);
	ssd1306_WriteString("HX711 (DMA)", Font_11x18, White);
	ssd1306_UpdateScreen();

	SoftTimerLCD = HAL_GetTick();
	HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*) (&TxData),
			(uint8_t*) (&RxData), sizeof(TxData));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		if (((HAL_GetTick() - SoftTimerLCD) > OLED_UPDATE_PERIOD)
				&& (sensor_read_flag == 1))
		{
			SoftTimerLCD = HAL_GetTick();

			MessageLength =
					sprintf((char*) UartMessage,
							"HX711 raw reading: 0b %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c \r\n",
							BYTE_TO_BINARY(RxData[0]),
							BYTE_TO_BINARY(RxData[1]),
							BYTE_TO_BINARY(RxData[2]),
							BYTE_TO_BINARY(RxData[3]),
							BYTE_TO_BINARY(RxData[4]),
							BYTE_TO_BINARY(RxData[5]),
							BYTE_TO_BINARY(RxData[6]), BYTE_TO_BINARY(RxData[7])
							);

			HAL_UART_Transmit(&huart2, UartMessage, MessageLength, 100);

			raw_reading = (((uint32_t) (RxData[7] & 0b00000010)) >> 1)
					| (((uint32_t) (RxData[7] & 0b00001000)) >> 2)
					| (((uint32_t) (RxData[7] & 0b00100000)) >> 3)
					| (((uint32_t) (RxData[7] & 0b10000000)) >> 4)
					| (((uint32_t) (RxData[6] & 0b00000010)) << 3)
					| (((uint32_t) (RxData[6] & 0b00001000)) << 2)
					| (((uint32_t) (RxData[6] & 0b00100000)) << 1)
					| (((uint32_t) (RxData[6] & 0b10000000)) << 0) |

					(((uint32_t) (RxData[5] & 0b00000010)) << 7)
					| (((uint32_t) (RxData[5] & 0b00001000)) << 6)
					| (((uint32_t) (RxData[5] & 0b00100000)) << 5)
					| (((uint32_t) (RxData[5] & 0b10000000)) << 4)
					| (((uint32_t) (RxData[4] & 0b00000010)) << 11)
					| (((uint32_t) (RxData[4] & 0b00001000)) << 10)
					| (((uint32_t) (RxData[4] & 0b00100000)) << 9)
					| (((uint32_t) (RxData[4] & 0b10000000)) << 8) |

					(((uint32_t) (RxData[3] & 0b00000010)) << 15)
					| (((uint32_t) (RxData[3] & 0b00001000)) << 14)
					| (((uint32_t) (RxData[3] & 0b00100000)) << 13)
					| (((uint32_t) (RxData[3] & 0b10000000)) << 12)
					| (((uint32_t) (RxData[2] & 0b00000010)) << 19)
					| (((uint32_t) (RxData[2] & 0b00001000)) << 18)
					| (((uint32_t) (RxData[2] & 0b00100000)) << 17)
					| (((uint32_t) (RxData[2] & 0b10000000)) << 16) |

					(((uint32_t) (RxData[1] & 0b00000010)) << 23)
					| (((uint32_t) (RxData[1] & 0b00001000)) << 22)
					| (((uint32_t) (RxData[1] & 0b00100000)) << 21)
					| (((uint32_t) (RxData[1] & 0b10000000)) << 20);

			raw_reading = raw_reading << 8; // 24-bit two's complement in 32-bit variable
			uint2int_converter.uint32_value = raw_reading;
			converted_reading = ((double) uint2int_converter.int32_value)
					/ 256.0;

			measured_weight =
					((double) (CALIBRATION_Y2 - CALIBRATION_Y1))
							/ ((double) (CALIBRATION_X2 - CALIBRATION_X1))
							* (converted_reading - (double) CALIBRATION_X2)+ CALIBRATION_Y2;

			sprintf(lcd_line, "%.0f g    ", round(measured_weight));
			ssd1306_SetCursor(10, 37);
			ssd1306_WriteString(lcd_line, Font_16x26, White);
			ssd1306_UpdateScreen();
		}

		if ((sensor_read_flag == 1)
				&& (HAL_GPIO_ReadPin(SPI2_MISO_GPIO_Port, SPI2_MISO_Pin)
						== GPIO_PIN_RESET))
		{
			sensor_read_flag = 0;
			HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*) (&TxData),
					(uint8_t*) (&RxData), sizeof(TxData));
		}
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
		sensor_read_flag = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == YELLOW_BUTTON_Pin)
	{
		yellow_button_flag = 1;
	}
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
