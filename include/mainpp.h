/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

/////////////////////////////////////           mod thêm         ///////////////////
#include "main.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;



void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_TIM2_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/////////////////////////////////////////////////////////////////////////////////////
void setup(void);
void loop(void);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
