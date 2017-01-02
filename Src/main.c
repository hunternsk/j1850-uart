/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "usbd_cdc_if.h"
#include "jnet.h"
#include "sw_btn.h"
#include "cb.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBUG 0
#define DEBUG_GPIO_PERIPHERAL GPIOE
#define DEBUG_GPIO_PIN GPIO_PIN_6

#define MAXFRAMES 255

char usbTransmit[1024];

uint8_t crcTable[256];
cb cbSWRxBuffer;
cb cbSWTxBuffer;

cb cbCDCRxBuffer;
bool bCDCRxBufferCplt;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
uint8_t CalcCRC(uint8_t * buf, uint8_t len);
void CRCInit(void);
void cbInit(cb *cb, size_t capacity, size_t sz);
void cbFree(cb *cb);
void cbPushBack(cb *cb, const uint8_t * item);
int cbPopFront(cb *cb, uint8_t * item);
extern int SendFrame(uint8_t * buf, uint8_t len);

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	CRCInit();

	cbInit(&cbSWRxBuffer , 16, sizeof(jFrame));
	cbInit(&cbSWTxBuffer , 16, sizeof(jFrame));
	cbInit(&cbCDCRxBuffer, 256, sizeof(uint8_t));
	
	struct JFrame jSWRxFrame = {0};
	struct JFrame jSWTxFrame = {0};
	uint8_t * jSWTxFramePtr = (uint8_t *) &jSWTxFrame;

	if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	

	extern bool bHUReady;
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		/*
		* If something received ftom J_NET 
		* Pop frame from CB to CDC
		*/ 
		if(cbSWRxBuffer.count) {
			if (cbPopFront(&cbSWRxBuffer, (uint8_t *) &jSWRxFrame)) {
				CDC_Transmit_FS((uint8_t *) &jSWRxFrame, 11); //TODO: jSWRxFrame.sz
			}
		}
		
		
		
		/*if(!bTxFrameReady && cbCDCRxBuffer.count) { 
			uint8_t rChar;
			if(cbPopFront(&cbCDCRxBuffer,(uint8_t *) &rChar)) {
				if(jSWTxFramePtr - (uint8_t *) &jSWTxFrame > 10 || rChar == 0x0D) { // If LF
					bTxFrameReady = true;
					jSWTxFramePtrShift = 4;
				} else {
					unsigned short tmpByte = 0;
					sscanf((const char *) &rChar, "%hx", &tmpByte);
					*jSWTxFramePtr |= tmpByte << jSWTxFramePtrShift;
					
					if(!jSWTxFramePtrShift) {
						jSWTxFramePtrShift = 4;
						jSWTxFramePtr++;
					} else jSWTxFramePtrShift = 0;
				}
			}
		}
		*/
		
		/*if(bHUReady && bTxFrameReady) {
			bTxFrameReady = false;
			
			uint8_t len = jSWTxFramePtr - ((uint8_t *) &jSWTxFrame); //Get len
			*jSWTxFramePtr = CalcCRC((uint8_t *) &jSWTxFrame, len); //Append CRC
			
			SendFrame((uint8_t *) &jSWTxFrame, len+1); //Send to J_NET
			
			memset(&jSWTxFrame, 0, sizeof(jSWTxFrame)); //Clear
			jSWTxFramePtr = (uint8_t *) &jSWTxFrame;
			
			
		}*/
		if(bCDCRxBufferCplt && cbCDCRxBuffer.count) {
			memset(&jSWTxFrame, 0x00, sizeof(jSWTxFrame));
			jSWTxFramePtr = (uint8_t *) &jSWTxFrame;
			
			while (cbCDCRxBuffer.count) {
				cbPopFront(&cbCDCRxBuffer, jSWTxFramePtr++);
				if (!cbCDCRxBuffer.count) bCDCRxBufferCplt = false;
			}
			
			jSWTxFrame.sz = jSWTxFramePtr - ((uint8_t *) &jSWTxFrame);
			*jSWTxFramePtr = CalcCRC((uint8_t *) &jSWTxFrame, jSWTxFrame.sz);
			jSWTxFrame.sz++;
			
			//SendFrame((uint8_t *) &jSWTxFrame, jSWTxFrame.sz);
			cbPushBack(&cbSWTxBuffer, (uint8_t *) &jSWTxFrame);
			
		} else if (bCDCRxBufferCplt && !cbCDCRxBuffer.count) bCDCRxBufferCplt = false;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
uint8_t CalcCRC(uint8_t * buf, uint8_t len) {
	const uint8_t * ptr = buf;
	uint8_t _crc = 0xFF;
	
	while(len--) _crc = crcTable[_crc ^ *ptr++];
		
	return ~_crc;
}

void CRCInit(void) {
	uint8_t _crc;
	for (int i = 0; i < 0x100; i++) {
		_crc = i;
		
		for (uint8_t bit = 0; bit < 8; bit++) _crc = (_crc & 0x80) ? ((_crc << 1) ^ 0x1D) : (_crc << 1);
		
		crcTable[i] = _crc;
  }
}

void cbInit(cb *cb, size_t capacity, size_t sz) {
	cb->buffer = malloc(capacity * sz);
	if(cb->buffer == NULL) {
		Error_Handler();
	} else {
		cb->buffer_end = (uint8_t *)cb->buffer + capacity * sz;
		cb->capacity = capacity;
		cb->count = 0;
		cb->sz = sz;
		cb->head = cb->buffer;
		cb->tail = cb->buffer;
	}
}

void cbFree(cb *cb) {
	free(cb->buffer);
}

void cbPushBack(cb *cb, const uint8_t * item) {
	if(cb->count == cb->capacity) {
		Error_Handler();
	} else {
		memcpy(cb->head, item, cb->sz);
		cb->head = (uint8_t *) cb->head + cb->sz;
		if(cb->head == cb->buffer_end) cb->head = cb->buffer;
		cb->count++;
	}
}
int cbPopFront(cb *cb, uint8_t * item) {
	if(cb->count == 0) {
		return 0;
	} else {
		memcpy(item, cb->tail, cb->sz);
		cb->tail = (uint8_t *) cb->tail + cb->sz;
		if(cb->tail == cb->buffer_end) cb->tail = cb->buffer;
		cb->count--;
		return 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
