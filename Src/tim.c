/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "stdbool.h"
#include "jnet.h"
#include "string.h"
#include "cb.h"
#include "config.h"

extern uint8_t CalcCRC(uint8_t * buf, uint8_t len);
extern void cbInit(cb *cb, size_t capacity, size_t sz);
extern void cbFree(cb *cb);
extern void cbPushBack(cb *cb, const uint8_t * item);
extern int cbPopFront(cb *cb, uint8_t * item);

extern cb cbJNetRxBuffer;
extern cb cbJNetTxBuffer;

#define DEBUG 1
#define DEBUG_GPIO_PERIPHERAL GPIOB
#define DEBUG_GPIO_PIN GPIO_PIN_3

#define BIT_BAND_SRAM(RAM,BIT) (*(volatile uint32_t*)(SRAM_BB_BASE + 32 * ((uint32_t)((void*)(RAM)) - SRAM_BASE) + 4 * ((uint32_t)(BIT))))

void DSET(void);
void DRESET(void);
void DBLINK(void);

void jNetSendBytes(uint8_t * buf, uint8_t len);
void jNetSendFrame(uint8_t * buf, uint8_t len);

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 86;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  
    /**TIM2 GPIO Configuration    
    PA1     ------> TIM2_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  
    /**TIM2 GPIO Configuration    
    PA1     ------> TIM2_CH2 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

struct JFrame inFrame;
struct JFrame outFrame;
uint8_t * inFramePtr = (uint8_t *) &inFrame;
uint8_t * outFramePtr = (uint8_t *) &outFrame;

bool bFrameRx = false;
bool bFrameTx = false;

uint8_t inFrameBit = 7;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		if (!htim->Instance->CNT && htim->Instance->ARR == TP7_MIN && bFrameRx) { // EOD detection
			uint8_t inFrameLen = inFramePtr - (uint8_t *) &inFrame;
			
			// calc CRC is OK
			if (*(inFramePtr - 1) == CalcCRC((uint8_t *) &inFrame, inFrameLen - 1)) {
				inFrame.sz = inFrameLen <= 11 ? inFrameLen : 11;
				cbPushBack(&cbJNetRxBuffer,(uint8_t *) &inFrame);
				
				if (AUTOIFR
					&& (!BIT_BAND_SRAM((uint8_t *) &inFrame, 4)) // Three bytes hdr
					&& inFrame.dst == MYID 
					&& inFrame.src == SWID
				) {
					uint8_t ifr = MYID;
					jNetSendBytes(&ifr, sizeof(ifr));

					htim3.Instance->CNT = 0;
					htim3.Instance->ARR = TP9_MIN;
					HAL_TIM_Base_Start_IT(&htim3);
				}
			}
			bFrameRx = false;
		} else if (!bFrameRx && !htim->Instance->CNT && htim->Instance->ARR == TP9_MIN) {
			HAL_TIM_Base_Stop_IT(&htim3);

			if(cbJNetTxBuffer.count) {
				cbPopFront(&cbJNetTxBuffer, (uint8_t *) &outFrame);
				jNetSendFrame((uint8_t *) &outFrame, outFrame.sz);
			}

			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(!bFrameTx && htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) { // RISING
			uint16_t ucFall = htim->Instance->CCR1 / TIM2CLK;
			uint16_t ucRise = htim->Instance->CCR2 / TIM2CLK;

			// Detect SOF
			if(ucFall >= TP4_MIN && ucRise <= TP5_MAX && ucRise >= TP5_MIN && ucFall <= TP4_MAX) {
				bFrameRx = true;
				memset(&inFrame, 0x00, sizeof(inFrame));
				
				inFramePtr = (uint8_t *) &inFrame;
				inFrameBit = 7;
				
				// TIM3 timeout is EOD
				HAL_TIM_Base_Stop_IT(&htim3);
				htim3.Instance->CNT = 0;
				htim3.Instance->ARR = TP7_MIN;
				HAL_TIM_Base_Start_IT(&htim3);
			}
			
			if(bFrameRx) {
				if(ucRise <= TP1_MAX) { // Got bit's rise
					htim3.Instance->CNT = 0;
				} else if(ucRise >= TP7_MAX && ucRise <= TP8_MAX) { //Got pulse beetween EOD and EOF - stop frame write
					bFrameRx = false;
				}
			}
		}

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { // FALLING
			uint16_t ucFall = htim->Instance->CCR2 / TIM2CLK;
			uint16_t ucRise = htim->Instance->CCR1 / TIM2CLK;
			

			if(bFrameRx) {
				if(ucRise <= TP3_MAX && ucFall >= TP2_MIN) {
					if(ucRise <= TP2_MAX) BIT_BAND_SRAM(inFramePtr, inFrameBit) = 0x01;
					else if(ucFall >= TP3_MIN) BIT_BAND_SRAM(inFramePtr, inFrameBit) = 0x00;
					
					if (!inFrameBit) {
						inFramePtr++;
						inFrameBit = 7;
					} else inFrameBit--;
				}
			}	
		}
	}
}

void DSET(void) {
	if (DEBUG) HAL_GPIO_WritePin(DEBUG_GPIO_PERIPHERAL, DEBUG_GPIO_PIN, GPIO_PIN_SET);
}
void DRESET(void) {
	if (DEBUG) HAL_GPIO_WritePin(DEBUG_GPIO_PERIPHERAL, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
}
void DBLINK(void) {
	
		if (DEBUG) {
			HAL_GPIO_WritePin(DEBUG_GPIO_PERIPHERAL, DEBUG_GPIO_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DEBUG_GPIO_PERIPHERAL, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
		}
}


void jNetSendBytes(uint8_t * buf, uint8_t len) {
	const uint8_t * ptr = buf;
	uint8_t ucPulseDominantTime;
	
	while(len--) {
		int i = 8;
		uint8_t val = *(ptr++);
		
		while(i--) {
			
			if(val & (1 << i)) ucPulseDominantTime = TP2_MIN; //bit 1
			else ucPulseDominantTime = TP3_MIN; //bit 0
			
			//Dominant
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
			
			htim3.Instance->ARR = ucPulseDominantTime; 
			HAL_TIM_Base_Start(&htim3);
			while ((TIM3->CR1 & TIM_CR1_CEN) != 0){}
			
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
			
			htim3.Instance->ARR = (TP1_MIN - ucPulseDominantTime); 
			HAL_TIM_Base_Start(&htim3);
			while ((TIM3->CR1 & TIM_CR1_CEN) != 0){}
		}
	}
	
	htim3.Instance->ARR = TP9_MIN;
}

void jNetSendFrame(uint8_t * buf, uint8_t len) {
	bool _htim_it = false;
	
	if (htim3.Instance->DIER & (1 << TIM_IT_UPDATE)) {
		__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
		_htim_it = true;
	}
	bFrameTx = true;
	
	// SOF Dominant
	htim3.Instance->ARR = TP4_NOM; 
	
	GPIOB->BSRR = GPIO_PIN_10; //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	GPIOB->BSRR = GPIO_PIN_11; //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_TIM_Base_Start(&htim3);
	while ((TIM3->CR1 & TIM_CR1_CEN) != 0) {}

	// SOF Passive
	GPIOB->BSRR = (uint32_t)GPIO_PIN_10 << 16;
	GPIOB->BSRR = (uint32_t)GPIO_PIN_11 << 16;
	TIM3->ARR = TP5_NOM - TP4_NOM; 
	HAL_TIM_Base_Start(&htim3);
	while ((TIM3->CR1 & TIM_CR1_CEN) != 0) {}

	TIM3->ARR = TP9_MIN;
	
	jNetSendBytes(buf, len);
	
	if (_htim_it) __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	bFrameTx = false;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
