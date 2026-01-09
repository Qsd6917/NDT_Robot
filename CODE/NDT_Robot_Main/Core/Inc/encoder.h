/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   编码器驱动模块头文件
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
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  编码器结构体
  */
typedef struct {
    TIM_HandleTypeDef *htim;      // 定时器句柄
    int32_t count_last;           // 上次计数
    int32_t count_total;          // 累计计数（处理溢出）
    float speed_rps;              // 当前速度（转/秒）
    uint16_t encoder_ppr;          // 编码器每转脉冲数（PPR）
} Encoder_TypeDef;

/* Exported constants --------------------------------------------------------*/
/* 编码器参数 */
#define ENCODER_PPR_DEFAULT       13      // MC130编码器默认PPR（每转13个脉冲，4倍频后52）
#define ENCODER_FILTER             6      // 输入滤波器值（抗干扰）

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Encoder_Init(Encoder_TypeDef *enc, TIM_HandleTypeDef *htim, uint16_t ppr);
int32_t Encoder_GetCount(Encoder_TypeDef *enc);
void Encoder_Reset(Encoder_TypeDef *enc);
float Encoder_GetSpeed(Encoder_TypeDef *enc, float dt);
int32_t Encoder_GetTotalCount(Encoder_TypeDef *enc);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
