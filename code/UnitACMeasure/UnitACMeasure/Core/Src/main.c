/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "i2c_ex.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS 0x42
#define FLASH_DATA_SIZE 8
#define FIRMWARE_VERSION 1
#define APPLICATION_ADDRESS     ((uint32_t)0x08001800) 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile uint8_t fm_version = FIRMWARE_VERSION;
uint8_t flash_data[FLASH_DATA_SIZE] = {0};
uint8_t i2c_address[1] = {0};

uint8_t buffer[30] = {0};

uint8_t voltage_factor = 188;
uint8_t current_factor = 100;

float voltage_factor_float = 1.88;
float current_factor_float = 1.00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);

	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void user_i2c_init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = i2c_address[0]<<1;
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

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;
    flash_data[0] = I2C_ADDRESS;
    flash_data[1] = voltage_factor;
    flash_data[2] = current_factor;
    flash_data[3] = 0;
    flash_data[4] = 0;
    flash_data[5] = 0;
    flash_data[6] = 0;
    flash_data[7] = 0;
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    voltage_factor = flash_data[1];
    current_factor = flash_data[2];
    voltage_factor_float = (float)voltage_factor / 100.0;
    current_factor_float = (float)current_factor / 100.0;
  }
}

void flash_data_write_back(void)
{
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[0] = i2c_address[0];
    flash_data[1] = voltage_factor;
    flash_data[2] = current_factor;
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  }     
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len)
{
  uint32_t uint32_dis_value = 0;
  char temp_integer_part[8] = {0};
  char temp_decimal_part[4] = {0};
  char temp_tx_char_1[7] = {0};
  char temp_tx_char_2[4] = {0};
  char temp_tx_char_3[11] = {0};

  if (len == 1) {
    if((rx_data[0] >= 0x60) && (rx_data[0] <= 0x61)) {
      uint32_dis_value = (uint32_t)(DisplayTable[0] * 100);
      i2c1_set_send_data((uint8_t *)&uint32_dis_value, 2);
    }     
    else if((rx_data[0] >= 0x70) && (rx_data[0] <= 0x71)) {
      uint32_dis_value = (uint32_t)(DisplayTable[1] * 100);
      i2c1_set_send_data((uint8_t *)&uint32_dis_value, 2);
    }     
    else if((rx_data[0] >= 0x80) && (rx_data[0] <= 0x81)) {
      uint32_dis_value = (uint32_t)(DisplayTable[2] * 100);
      i2c1_set_send_data((uint8_t *)&uint32_dis_value, 2);
    }     
    else if((rx_data[0] >= 0x90) && (rx_data[0] <= 0x91)) {
      uint32_dis_value = (uint32_t)(DisplayTable[5] * 100);
      i2c1_set_send_data((uint8_t *)&uint32_dis_value, 2);
    }     
    else if(rx_data[0] == 0xA0) {
      uint32_dis_value = (uint32_t)(DisplayTable[3] * 100);
      if (uint32_dis_value > 100)
        uint32_dis_value = 100;
      i2c1_set_send_data((uint8_t *)&uint32_dis_value, 1);
    }
    else if((rx_data[0] >= 0xB0) && (rx_data[0] <= 0xB3)) {
      uint32_dis_value = (uint32_t)(DisplayTable[4] * 100);
      i2c1_set_send_data((uint8_t *)&uint32_dis_value, 4);
    }          
    else if(rx_data[0] <= 0x06) {
      int32_t temp, dec;

      temp = ((int32_t)(DisplayTable[0] * 100));
      dec = temp;
      temp = temp / 100;
      dec = dec - temp * 100;
      if (temp < 9999)
        sprintf(temp_integer_part, "%04d", temp);
      if (dec < 99)
        sprintf(temp_decimal_part, "%c%02d", '.', dec);
      memcpy(temp_tx_char_1, temp_integer_part, 4);
      memcpy(temp_tx_char_1+4, temp_decimal_part, 3);
      i2c1_set_send_data((uint8_t *)temp_tx_char_1, sizeof(temp_tx_char_1)); 
    }          
    else if((rx_data[0] >= 0x10) && (rx_data[0] <= 0x16)) {
      int32_t temp, dec;

      temp = ((int32_t)(DisplayTable[1] * 100));
      dec = temp;
      temp = temp / 100;
      dec = dec - temp * 100;
      if (temp < 9999)
        sprintf(temp_integer_part, "%04d", temp);
      if (dec < 99)
        sprintf(temp_decimal_part, "%c%02d", '.', dec);
      memcpy(temp_tx_char_1, temp_integer_part, 4);
      memcpy(temp_tx_char_1+4, temp_decimal_part, 3);
      i2c1_set_send_data((uint8_t *)temp_tx_char_1, sizeof(temp_tx_char_1)); 
    }          
    else if((rx_data[0] >= 0x20) && (rx_data[0] <= 0x26)) {
      int32_t temp, dec;

      temp = ((int32_t)(DisplayTable[2] * 100));
      dec = temp;
      temp = temp / 100;
      dec = dec - temp * 100;
      if (temp < 9999)
        sprintf(temp_integer_part, "%04d", temp);
      if (dec < 99)
        sprintf(temp_decimal_part, "%c%02d", '.', dec);
      memcpy(temp_tx_char_1, temp_integer_part, 4);
      memcpy(temp_tx_char_1+4, temp_decimal_part, 3);
      i2c1_set_send_data((uint8_t *)temp_tx_char_1, sizeof(temp_tx_char_1)); 
    }          
    else if((rx_data[0] >= 0x30) && (rx_data[0] <= 0x36)) {
      int32_t temp, dec;

      temp = ((int32_t)(DisplayTable[5] * 100));
      dec = temp;
      temp = temp / 100;
      dec = dec - temp * 100;
      if (temp < 9999)
        sprintf(temp_integer_part, "%04d", temp);
      if (dec < 99)
        sprintf(temp_decimal_part, "%c%02d", '.', dec);
      memcpy(temp_tx_char_1, temp_integer_part, 4);
      memcpy(temp_tx_char_1+4, temp_decimal_part, 3);
      i2c1_set_send_data((uint8_t *)temp_tx_char_1, sizeof(temp_tx_char_1)); 
    }          
    else if((rx_data[0] >= 0x40) && (rx_data[0] <= 0x43)) {
      int32_t temp, dec;

      temp = ((int32_t)(DisplayTable[3] * 100));
      if (temp >= 100) {
        sprintf(temp_integer_part, "1");
        sprintf(temp_decimal_part, ".00");
      } else {
        dec = temp;
        sprintf(temp_integer_part, "0");
        sprintf(temp_decimal_part, "%c%02d", '.', dec);
      }
      memcpy(temp_tx_char_2, temp_integer_part, 1);
      memcpy(temp_tx_char_2+1, temp_decimal_part, 3);
      i2c1_set_send_data((uint8_t *)temp_tx_char_2, sizeof(temp_tx_char_2)); 
    } 
    else if((rx_data[0] >= 0x50) && (rx_data[0] <= 0x5A)) {
      int32_t temp, dec;

      temp = ((int32_t)(DisplayTable[4] * 100));
      dec = temp;
      temp = temp / 100;
      dec = dec - temp * 100;
      if (temp < 99999999)
        sprintf(temp_integer_part, "%08d", temp);
      if (dec < 99)
        sprintf(temp_decimal_part, "%c%02d", '.', dec);
      memcpy(temp_tx_char_3, temp_integer_part, 8);
      memcpy(temp_tx_char_3+8, temp_decimal_part, 3);
      i2c1_set_send_data((uint8_t *)temp_tx_char_3, sizeof(temp_tx_char_3)); 
    }  
    else if(rx_data[0] == 0xC0) {
      i2c1_set_send_data((uint8_t *)&voltage_factor, 1); 
    }                 
    else if(rx_data[0] == 0xD0) {
      i2c1_set_send_data((uint8_t *)&current_factor, 1); 
    }
    else if (rx_data[0] == 0xFE)
    {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);
    }                     
    else if (rx_data[0] == 0xFF)
    {
      i2c1_set_send_data(i2c_address, 1);
    }                     
  } else if (len > 1) {
    if(rx_data[0] == 0xC0) {
      voltage_factor = rx_data[1];
      voltage_factor_float = (float)voltage_factor / 100.0;
    }
    else if(rx_data[0] == 0xD0) {
      current_factor = rx_data[1];
      current_factor_float = (float)current_factor / 100.0;
    }
    else if (rx_data[0] == 0xFF)
    {
      if (rx_data[1] < 128) {
        i2c_address[0] = rx_data[1];
        flash_data_write_back();
        user_i2c_init();
      }  
    }    
    else if(rx_data[0] == 0xE0) {
      if (rx_data[1]) {
        flash_data_write_back();
      }
    }    
    else if ((rx_data[0] >= 0xB0) && (rx_data[0] <= 0xB3)) {
      if (len == 5) {
        uint32_t temp = 0;
        temp = (rx_data[1] | (rx_data[2] << 8) | (rx_data[3] << 16) | (rx_data[4] << 24));
        DisplayTable[4] = (float)temp / 100.0;
      }
    }    
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
  IAP_Set();
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
  // MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  init_flash_data();  
  user_i2c_init();
  HAL_I2C_EnableListen_IT(&hi2c1);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart2, buffer, 24);    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 0x55<<1;
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
  huart2.Init.BaudRate = 4800;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
