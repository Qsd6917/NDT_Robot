/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pid_controller.h
  * @brief   PID控制器模块头文件
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
#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  PID控制器结构体
  */
typedef struct {
    float kp;                  // 比例系数
    float ki;                  // 积分系数
    float kd;                  // 微分系数
    float target;              // 目标值
    float current;             // 当前值
    float integral;            // 积分累积
    float last_error;          // 上次误差
    float output;              // 输出值
    float output_max;          // 输出上限
    float output_min;          // 输出下限
    float integral_max;        // 积分限幅（抗饱和）
    float integral_min;
    uint8_t enable;            // 使能标志
} PID_Controller_TypeDef;

/* Exported constants --------------------------------------------------------*/
/* PID默认参数 */
#define PID_KP_DEFAULT         2.0f
#define PID_KI_DEFAULT         0.1f
#define PID_KD_DEFAULT         0.5f
#define PID_OUTPUT_MAX         999.0f    // 对应PWM最大值
#define PID_OUTPUT_MIN         -999.0f   // 对应PWM最小值（负值表示反转）

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void PID_Init(PID_Controller_TypeDef *pid, float kp, float ki, float kd, 
              float output_max, float output_min);
void PID_SetTarget(PID_Controller_TypeDef *pid, float target);
void PID_SetParams(PID_Controller_TypeDef *pid, float kp, float ki, float kd);
float PID_Compute(PID_Controller_TypeDef *pid, float current, float dt);
void PID_Reset(PID_Controller_TypeDef *pid);
void PID_Enable(PID_Controller_TypeDef *pid, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */
