/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   电机控制模块实现（整合编码器、PID和PWM）
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
#include "motor_control.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void Motor_Control_UpdatePWM(Motor_Control_TypeDef *motor, float pwm_value);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  初始化电机控制
  * @param  motor: 电机控制结构体指针
  * @param  encoder_tim: 编码器定时器句柄
  * @param  pwm_tim: PWM定时器句柄
  * @param  pwm_channel: PWM通道
  * @param  encoder_ppr: 编码器每转脉冲数
  * @retval None
  */
void Motor_Control_Init(Motor_Control_TypeDef *motor,
                        TIM_HandleTypeDef *encoder_tim,
                        TIM_HandleTypeDef *pwm_tim,
                        uint32_t pwm_channel,
                        uint16_t encoder_ppr)
{
    // 初始化编码器
    Encoder_Init(&motor->encoder, encoder_tim, encoder_ppr);
    
    // 初始化PID控制器
    PID_Init(&motor->pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
             PID_OUTPUT_MAX, PID_OUTPUT_MIN);
    
    // 保存PWM相关参数
    motor->pwm_tim = pwm_tim;
    motor->pwm_channel = pwm_channel;
    
    // 初始化其他参数
    motor->target_speed_rps = 0.0f;
    motor->current_speed_rps = 0.0f;
    motor->speed_filter_alpha = MOTOR_SPEED_FILTER_ALPHA;
    motor->enable = 1;
    
    // 启动PWM
    HAL_TIM_PWM_Start(pwm_tim, pwm_channel);
}

/**
  * @brief  设置目标速度
  * @param  motor: 电机控制结构体指针
  * @param  speed_rps: 目标速度（转/秒）
  * @retval None
  */
void Motor_Control_SetTargetSpeed(Motor_Control_TypeDef *motor, float speed_rps)
{
    // 限幅
    if (speed_rps > MOTOR_SPEED_MAX_RPS)
    {
        speed_rps = MOTOR_SPEED_MAX_RPS;
    }
    else if (speed_rps < MOTOR_SPEED_MIN_RPS)
    {
        speed_rps = MOTOR_SPEED_MIN_RPS;
    }
    
    motor->target_speed_rps = speed_rps;
    PID_SetTarget(&motor->pid, speed_rps);
}

/**
  * @brief  更新电机控制（在定时器中断中调用）
  * @param  motor: 电机控制结构体指针
  * @param  dt: 采样时间间隔（秒）
  * @retval None
  */
void Motor_Control_Update(Motor_Control_TypeDef *motor, float dt)
{
    if (!motor->enable || dt <= 0.0f)
    {
        return;
    }
    
    // 读取编码器速度
    float speed_raw = Encoder_GetSpeed(&motor->encoder, dt);
    
    // 速度滤波（一阶低通滤波）
    motor->current_speed_rps = motor->speed_filter_alpha * motor->current_speed_rps + 
                               (1.0f - motor->speed_filter_alpha) * speed_raw;
    
    // PID计算
    float pid_output = PID_Compute(&motor->pid, motor->current_speed_rps, dt);
    
    // 更新PWM输出
    Motor_Control_UpdatePWM(motor, pid_output);
}

/**
  * @brief  更新PWM输出
  * @param  motor: 电机控制结构体指针
  * @param  pwm_value: PWM值（-999到999，负值表示反转）
  * @retval None
  */
static void Motor_Control_UpdatePWM(Motor_Control_TypeDef *motor, float pwm_value)
{
    // 限幅
    if (pwm_value > PID_OUTPUT_MAX)
    {
        pwm_value = PID_OUTPUT_MAX;
    }
    else if (pwm_value < PID_OUTPUT_MIN)
    {
        pwm_value = PID_OUTPUT_MIN;
    }
    
    // 转换为PWM占空比（0-999对应0-100%）
    uint16_t pwm_duty = (uint16_t)fabsf(pwm_value);
    
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, pwm_duty);
    
    // 注意：这里假设使用单路PWM + 方向控制
    // 如果需要双路PWM实现正反转，需要根据pwm_value的正负来控制两个通道
}

/**
  * @brief  停止电机
  * @param  motor: 电机控制结构体指针
  * @retval None
  */
void Motor_Control_Stop(Motor_Control_TypeDef *motor)
{
    motor->target_speed_rps = 0.0f;
    PID_SetTarget(&motor->pid, 0.0f);
    PID_Reset(&motor->pid);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, 0);
}

/**
  * @brief  使能/禁用电机控制
  * @param  motor: 电机控制结构体指针
  * @param  enable: 使能标志（1=使能，0=禁用）
  * @retval None
  */
void Motor_Control_Enable(Motor_Control_TypeDef *motor, uint8_t enable)
{
    motor->enable = enable;
    PID_Enable(&motor->pid, enable);
    if (!enable)
    {
        Motor_Control_Stop(motor);
    }
}

/**
  * @brief  获取当前速度
  * @param  motor: 电机控制结构体指针
  * @retval 当前速度（转/秒）
  */
float Motor_Control_GetCurrentSpeed(Motor_Control_TypeDef *motor)
{
    return motor->current_speed_rps;
}

/**
  * @brief  将遥控速度值（0-255）映射到目标速度（转/秒）
  * @param  remote_speed: 遥控速度值（0-255，128为静止）
  * @retval 目标速度（转/秒）
  */
float Motor_Control_MapRemoteSpeed(uint8_t remote_speed)
{
    // 将0-255映射到-MOTOR_SPEED_MAX_RPS到+MOTOR_SPEED_MAX_RPS
    // 128对应0（静止）
    float speed_rps;
    
    if (remote_speed == 128)
    {
        speed_rps = 0.0f;
    }
    else if (remote_speed < 128)
    {
        // 反转：0-127映射到-MOTOR_SPEED_MAX_RPS到0
        speed_rps = -((128.0f - (float)remote_speed) / 128.0f) * MOTOR_SPEED_MAX_RPS;
    }
    else
    {
        // 正转：129-255映射到0到+MOTOR_SPEED_MAX_RPS
        speed_rps = (((float)remote_speed - 128.0f) / 127.0f) * MOTOR_SPEED_MAX_RPS;
    }
    
    return speed_rps;
}

/**
  * @brief  获取调试信息
  * @param  motor: 电机控制结构体指针
  * @param  target: 输出目标速度指针
  * @param  current: 输出当前速度指针
  * @param  pid_output: 输出PID输出值指针
  * @retval None
  */
void Motor_Control_GetDebugInfo(Motor_Control_TypeDef *motor, float *target, float *current, float *pid_output)
{
    if (target != NULL)
    {
        *target = motor->target_speed_rps;
    }
    if (current != NULL)
    {
        *current = motor->current_speed_rps;
    }
    if (pid_output != NULL)
    {
        *pid_output = motor->pid.output;
    }
}

/* USER CODE END 1 */
