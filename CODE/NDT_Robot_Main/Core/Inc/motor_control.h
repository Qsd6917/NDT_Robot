/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   电机控制模块头文件（整合编码器、PID和PWM）
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
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "encoder.h"
#include "pid_controller.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  电机控制结构体
  */
typedef struct {
    Encoder_TypeDef encoder;                    // 编码器
    PID_Controller_TypeDef pid;                 // PID控制器
    TIM_HandleTypeDef *pwm_tim;                 // PWM定时器句柄
    uint32_t pwm_channel;                       // PWM通道
    float target_speed_rps;                    // 目标速度（转/秒）
    float current_speed_rps;                   // 当前速度（转/秒）
    float speed_filter_alpha;                   // 速度滤波系数（0-1）
    uint8_t enable;                             // 使能标志
} Motor_Control_TypeDef;

/* Exported constants --------------------------------------------------------*/
/* 速度滤波参数 */
#define MOTOR_SPEED_FILTER_ALPHA    0.8f        // 一阶低通滤波系数

/* 速度映射参数 */
#define MOTOR_SPEED_MAX_RPS         10.0f       // 最大速度（转/秒）
#define MOTOR_SPEED_MIN_RPS         -10.0f      // 最小速度（转/秒）

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Control_Init(Motor_Control_TypeDef *motor, 
                        TIM_HandleTypeDef *encoder_tim,
                        TIM_HandleTypeDef *pwm_tim,
                        uint32_t pwm_channel,
                        uint16_t encoder_ppr);
void Motor_Control_SetTargetSpeed(Motor_Control_TypeDef *motor, float speed_rps);
void Motor_Control_Update(Motor_Control_TypeDef *motor, float dt);
void Motor_Control_Stop(Motor_Control_TypeDef *motor);
void Motor_Control_Enable(Motor_Control_TypeDef *motor, uint8_t enable);
float Motor_Control_GetCurrentSpeed(Motor_Control_TypeDef *motor);
float Motor_Control_MapRemoteSpeed(uint8_t remote_speed);
void Motor_Control_GetDebugInfo(Motor_Control_TypeDef *motor, float *target, float *current, float *pid_output);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
