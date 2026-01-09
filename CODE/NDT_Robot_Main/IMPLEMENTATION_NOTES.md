# NDT_Robot 编码器电机闭环控制实现说明

## 实现概述

已成功实现基于STM32F103的编码器TT马达PID闭环控制系统，包括：

1. **编码器驱动模块** (`encoder.c/h`)
2. **PID控制器模块** (`pid_controller.c/h`)
3. **电机控制模块** (`motor_control.c/h`)
4. **定时器中断配置** (TIM6用于PID计算)
5. **遥控通信集成**
6. **实时性优化**

## 硬件配置

### 定时器分配
- **TIM2**: 编码器接口（PA0/PA1，编码器模式）
- **TIM3**: PWM输出（PA6，电机速度控制）
- **TIM6**: PID计算定时器（10ms周期，100Hz）

### 引脚配置
- **PA0**: TIM2_CH1（编码器A相）
- **PA1**: TIM2_CH2（编码器B相）
- **PA6**: TIM3_CH1（PWM输出）
- **PA9**: USART1_TX（串口发送）
- **PA10**: USART1_RX（串口接收）
- **PE1**: GPIO输出（升降控制）

## 中断优先级配置

- **TIM6中断**: 优先级1（PID计算，次高优先级）
- **USART1中断**: 优先级2（串口接收，较低优先级）

## 关键参数

### 编码器参数
- **PPR**: 13（MC130编码器默认值，4倍频后为52脉冲/转）
- **滤波器**: 6（抗干扰）

### PID参数（默认值）
- **Kp**: 2.0
- **Ki**: 0.1
- **Kd**: 0.5
- **输出限幅**: -999 到 +999（对应PWM占空比）

### 速度映射
- **最大速度**: ±10转/秒
- **遥控值映射**: 0-255 → -10到+10转/秒（128为静止）

## 使用说明

### 1. PID参数调整

在 `main.c` 的初始化部分，可以修改PID参数：

```c
// 在Motor_Control_Init之后
PID_SetParams(&motor1.pid, 2.0f, 0.1f, 0.5f);  // Kp, Ki, Kd
```

### 2. 调试输出

启用调试输出（在 `main.c` 中）：
```c
#define DEBUG_ENABLE 1  // 改为1启用
```

然后实现串口输出函数（需要重定向printf或使用HAL_UART_Transmit）。

### 3. 编码器PPR调整

如果使用不同PPR的编码器，修改初始化：
```c
Motor_Control_Init(&motor1, &htim2, &htim3, TIM_CHANNEL_1, 您的PPR值);
```

## 注意事项

1. **编码器接线**: 确保编码器A/B相正确连接到PA0/PA1
2. **PWM频率**: TIM3配置为约1kHz PWM频率（Prescaler=71, Period=999）
3. **看门狗**: 500ms无遥控信号自动停止电机
4. **速度滤波**: 使用一阶低通滤波（alpha=0.8）平滑速度反馈

## 文件结构

```
CODE/NDT_Robot_Main/
├── Core/
│   ├── Inc/
│   │   ├── encoder.h              # 编码器驱动
│   │   ├── pid_controller.h        # PID控制器
│   │   ├── motor_control.h         # 电机控制
│   │   └── main.h
│   └── Src/
│       ├── encoder.c
│       ├── pid_controller.c
│       ├── motor_control.c
│       ├── main.c                  # 主程序
│       ├── stm32f1xx_hal_msp.c    # HAL MSP配置
│       └── stm32f1xx_it.c          # 中断处理
```

## 下一步优化建议

1. **双路PWM控制**: 当前使用单路PWM，如需正反转控制，可扩展为双路PWM
2. **DMA串口接收**: 可优化为DMA模式降低CPU占用
3. **多电机支持**: 当前实现单电机，可扩展为多电机控制
4. **参数存储**: 可将PID参数存储到Flash，支持掉电保存

## 测试建议

1. **开环测试**: 先测试编码器计数和PWM输出
2. **闭环测试**: 逐步调整PID参数，观察响应速度
3. **稳定性测试**: 长时间运行验证系统稳定性
