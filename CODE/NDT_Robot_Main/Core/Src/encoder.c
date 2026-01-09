/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   编码器驱动模块实现
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
/* USER CODE BEGIN Header */

/* Includes ------------------------------------------------------------------*/
#include "encoder.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  初始化编码器
  * @param  enc: 编码器结构体指针
  * @param  htim: 定时器句柄指针（用于编码器模式）
  * @param  ppr: 编码器每转脉冲数（PPR）
  * @retval None
  */
void Encoder_Init(Encoder_TypeDef *enc, TIM_HandleTypeDef *htim, uint16_t ppr)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    // 保存定时器句柄和PPR
    enc->htim = htim;
    enc->encoder_ppr = ppr;
    enc->count_last = 0;
    enc->count_total = 0;
    enc->speed_rps = 0.0f;
    
    // 配置编码器接口
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;  // 4倍频模式
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = ENCODER_FILTER;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = ENCODER_FILTER;
    
    // 配置定时器基础参数（在调用HAL_TIM_Encoder_Init之前）
    htim->Init.Period = 0xFFFF;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_Encoder_Init(htim, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    // 启动编码器
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    
    // 复位计数器
    __HAL_TIM_SET_COUNTER(htim, 0);
    enc->count_last = 0;
}

/**
  * @brief  获取编码器当前计数值
  * @param  enc: 编码器结构体指针
  * @retval 当前计数值（16位，有符号扩展）
  */
int32_t Encoder_GetCount(Encoder_TypeDef *enc)
{
    int16_t count_16bit = (int16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    return (int32_t)count_16bit;
}

/**
  * @brief  获取编码器累计计数值（处理溢出）
  * @param  enc: 编码器结构体指针
  * @retval 累计计数值
  */
int32_t Encoder_GetTotalCount(Encoder_TypeDef *enc)
{
    int32_t count_now = Encoder_GetCount(enc);
    int32_t count_diff = count_now - enc->count_last;
    
    // 处理16位计数器溢出（从65535到0或从0到65535）
    if (count_diff > 32767)
    {
        count_diff -= 65536;
    }
    else if (count_diff < -32767)
    {
        count_diff += 65536;
    }
    
    enc->count_total += count_diff;
    enc->count_last = count_now;
    
    return enc->count_total;
}

/**
  * @brief  计算编码器速度
  * @param  enc: 编码器结构体指针
  * @param  dt: 采样时间间隔（秒）
  * @retval 速度（转/秒，RPS）
  */
float Encoder_GetSpeed(Encoder_TypeDef *enc, float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }
    
    // 获取当前计数
    int32_t count_now = Encoder_GetCount(enc);
    int32_t count_diff = count_now - enc->count_last;
    
    // 处理16位计数器溢出
    if (count_diff > 32767)
    {
        count_diff -= 65536;
    }
    else if (count_diff < -32767)
    {
        count_diff += 65536;
    }
    
    // 更新累计计数
    enc->count_total += count_diff;
    enc->count_last = count_now;
    
    // 计算速度：脉冲数 / (采样时间 * 每转脉冲数 * 4倍频)
    // 编码器模式为TI12，4倍频，所以实际每转脉冲数 = ppr * 4
    float pulses_per_rev = (float)(enc->encoder_ppr * 4);
    enc->speed_rps = (float)count_diff / (dt * pulses_per_rev);
    
    return enc->speed_rps;
}

/**
  * @brief  复位编码器计数
  * @param  enc: 编码器结构体指针
  * @retval None
  */
void Encoder_Reset(Encoder_TypeDef *enc)
{
    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->count_last = 0;
    enc->count_total = 0;
    enc->speed_rps = 0.0f;
}

/* USER CODE END 1 */
