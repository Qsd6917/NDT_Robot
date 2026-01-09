/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pid_controller.c
  * @brief   PID控制器模块实现
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
#include "pid_controller.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  初始化PID控制器
  * @param  pid: PID控制器结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @param  output_max: 输出上限
  * @param  output_min: 输出下限
  * @retval None
  */
void PID_Init(PID_Controller_TypeDef *pid, float kp, float ki, float kd,
              float output_max, float output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->integral_max = output_max * 0.5f;  // 积分限幅为输出的50%
    pid->integral_min = output_min * 0.5f;
    pid->enable = 1;
}

/**
  * @brief  设置PID目标值
  * @param  pid: PID控制器结构体指针
  * @param  target: 目标值
  * @retval None
  */
void PID_SetTarget(PID_Controller_TypeDef *pid, float target)
{
    pid->target = target;
}

/**
  * @brief  设置PID参数
  * @param  pid: PID控制器结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @retval None
  */
void PID_SetParams(PID_Controller_TypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
  * @brief  PID计算（位置式PID算法）
  * @param  pid: PID控制器结构体指针
  * @param  current: 当前值
  * @param  dt: 采样时间间隔（秒）
  * @retval PID输出值
  */
float PID_Compute(PID_Controller_TypeDef *pid, float current, float dt)
{
    if (!pid->enable || dt <= 0.0f)
    {
        return pid->output;
    }
    
    pid->current = current;
    
    // 计算误差
    float error = pid->target - pid->current;
    
    // 比例项
    float p_term = pid->kp * error;
    
    // 积分项（带抗饱和）
    pid->integral += error * dt;
    
    // 积分限幅（抗饱和处理）
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < pid->integral_min)
    {
        pid->integral = pid->integral_min;
    }
    
    float i_term = pid->ki * pid->integral;
    
    // 微分项
    float d_term = 0.0f;
    if (dt > 0.0f)
    {
        d_term = pid->kd * (error - pid->last_error) / dt;
    }
    
    // PID输出
    pid->output = p_term + i_term + d_term;
    
    // 输出限幅
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }
    
    // 保存本次误差
    pid->last_error = error;
    
    return pid->output;
}

/**
  * @brief  复位PID控制器
  * @param  pid: PID控制器结构体指针
  * @retval None
  */
void PID_Reset(PID_Controller_TypeDef *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->current = 0.0f;
}

/**
  * @brief  使能/禁用PID控制器
  * @param  pid: PID控制器结构体指针
  * @param  enable: 使能标志（1=使能，0=禁用）
  * @retval None
  */
void PID_Enable(PID_Controller_TypeDef *pid, uint8_t enable)
{
    pid->enable = enable;
    if (!enable)
    {
        PID_Reset(pid);
    }
}

/* USER CODE END 1 */
