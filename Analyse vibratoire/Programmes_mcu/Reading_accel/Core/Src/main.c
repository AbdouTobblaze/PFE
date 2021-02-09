/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
uint8_t buffer[2048] = {0};
float buffer_float[1024] = {0};
volatile uint16_t tap = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t wait = 0;
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
  MX_SPI2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
    MPU9250_intance_init();
  //MPU9250_Config_fifo_int(&MPU9250);
  MPU9250_Config_accel_scale(&MPU9250);
  MPU9250_Write_Register(&MPU9250, MPU9250_PWR_MGMT_2, 0x37);
  //MPU9250_Write_Register(&MPU9250, MPU9250_INT_ENABLE, 0x01);
  char message[64];


//	 uint8_t data = 0;
//	 uint8_t reg = MPU9250_INT_PIN_CFG | 0x80;
//	 uint8_t dummy = 0;
//	 HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
//	 HAL_SPI_TransmitReceive(&hspi2, &reg, &dummy, 1, 10);
//	 HAL_SPI_TransmitReceive(&hspi2, &dummy, &data, 1, 10);
//	 HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
//	 sprintf(message,"int_conf = %x\n\r", data);
//	 HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);
  sprintf(message,"Hello\n\r");
  HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);
  //HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	 MPU9250_Write_Register(&MPU9250, MPU9250_PWR_MGMT_1, 0x80);
//	 HAL_Delay(2);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_PWR_MGMT_1, 0x01);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_PWR_MGMT_2, 0x37);
//	 //HAL_Delay(200);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_INT_PIN_CFG, 0x10);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_INT_ENABLE, 0x01);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_FIFO_EN, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_PWR_MGMT_1, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_I2C_MST_CTRL, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_USER_CTRL, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_USER_CTRL, 0x0C);
//	 HAL_Delay(15);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_CONFIG_REG, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_SMPLRT_DIV, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_GYRO_CONFIG, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_ACCEL_CONFIG, 0x00);
//	 MPU9250_Write_Register(&MPU9250, MPU9250_USER_CTRL, 0x40);
	  memset(buffer, 0, sizeof(buffer));
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	  MPU9250_Write_Register(&MPU9250, MPU9250_INT_ENABLE, 0x01);
	  wait = 0; tap = 0;
	  while(wait == 0);
	  for(int i = 0; i < 1024; i++)
	  {
		  int16_t v = (uint16_t)(buffer[2*i] << 8) | buffer[(2*i)+1];
		  buffer_float[i] = (float)v/2048.0;
	  }
	  sprintf(message,"S, ");
	  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message),100);
	  for(int i = 0; i < 1024; i++)
	  {
	  		  sprintf(message,"%.2f, ",buffer_float[i]);
	  		  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message),100);
	  }
	  sprintf(message,"\n\r");
	  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message),100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void MPU9250_intance_init(void)
{
	MPU9250.hspi = &hspi2;
	MPU9250.GPIOx = MPU_CS_GPIO_Port;
	MPU9250.GPIO_PIN = MPU_CS_Pin;
	MPU9250.ACCEL_SCALE = ACCEL_SCALE_16G;
	MPU9250.ARES = 1.0/16384.0;
	//MPU9250_Initialize(&MPU9250);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 MPU9250_Read_Data_Registers(&MPU9250, MPU9250_ACCEL_ZOUT_H, 2, &buffer[2*tap]);
	 tap++;
	 if(tap == 1024)
	 {
	 	 wait = 1;
	 	MPU9250_Write_Register(&MPU9250, MPU9250_INT_ENABLE, 0x00);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
