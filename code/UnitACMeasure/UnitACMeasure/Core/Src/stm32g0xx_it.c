/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
unsigned long Voltage_Parameter_REG=0; //电压参数
unsigned long Voltage_REG=0;//电压
unsigned long Current_Parameter_REG=0; //电流参数
unsigned long Current_REG=0;//电流
unsigned long Power_Parameter_REG=0; //功率参数
unsigned long Power_REG=0;//功率
unsigned int Energy_count=0;
unsigned char DATA_REG_BIT7=0;  //数据更新寄存器BIT7

float DisplayTable[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Mycallback_handler(void)
{
	unsigned long Energy_CNT=0;
  uint8_t HLW8032_SumCheck=0;
  uint16_t hlw8032_check = 0;
  float voltage_current_factor;

  voltage_current_factor = voltage_factor_float * current_factor_float;
  
  if (buffer[0] == 0x55) {
    for (int i = 1; i < 24; i++) {
      hlw8032_check += buffer[i];
    }
    if (!hlw8032_check) {
      HAL_NVIC_SystemReset();
    }
    for (int i = 2; i < 23; i++) {
      HLW8032_SumCheck += buffer[i];
    }
    if (HLW8032_SumCheck == buffer[23]) {
      Voltage_Parameter_REG = buffer[2]*65536 + buffer[3]*256 + buffer[4];
      Voltage_REG = buffer[5]*65536 + buffer[6]*256 + buffer[7];
      Current_Parameter_REG = buffer[8]*65536 + buffer[9]*256 + buffer[10];
      Current_REG = buffer[11]*65536 + buffer[12]*256 + buffer[13];
      Power_Parameter_REG = buffer[14]*65536 + buffer[15]*256 + buffer[16];
      Power_REG = buffer[17]*65536 + buffer[18]*256 + buffer[19];	

      DisplayTable[0] = (float)Voltage_Parameter_REG * voltage_factor_float / Voltage_REG; //电压
      {
        DisplayTable[1] = (float)Current_Parameter_REG / Current_REG * current_factor_float;  //电流
        DisplayTable[2] = (float)Power_Parameter_REG * voltage_current_factor / Power_REG;//功率
        DisplayTable[5] = DisplayTable[0] * DisplayTable[1];//视在功率
        DisplayTable[3] = DisplayTable[2] / DisplayTable[5];//功率因数
      }
      if(DATA_REG_BIT7!=((buffer[20]>>7)&1))
      {
        Energy_count++;
        DATA_REG_BIT7=(buffer[20]>>7)&1;
      }
      Energy_CNT=Energy_count*65536+buffer[21]*256+buffer[22];
      DisplayTable[4]=(float)Energy_CNT/1000000000*Power_Parameter_REG*voltage_current_factor/3600;//电能 
      memset(buffer, 0, sizeof(buffer));
    }
  }
}

void Usart_Receive_Data(UART_HandleTypeDef *huart)
{
  if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
    Mycallback_handler();                                 //调用中断处理函数,
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, buffer, 24);             //重启DMA
  }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */
  if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c1);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c1);
  }
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  Usart_Receive_Data(&huart2);
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
