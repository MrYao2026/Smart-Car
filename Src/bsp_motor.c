/**
 * @file    bsp_motor.c
 * @brief   小车左右直流电机驱动实现。
 */

#include "bsp_motor.h" /* 引入电机驱动接口声明。 */
#include "main.h"      /* 引入 HAL、GPIO 定义以及 Error_Handler() 声明。 */

/* 引用 main.c 中由 CubeMX 创建并初始化的 TIM1、TIM2 句柄。 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* 定义 PWM 比较值最大值，与定时器 ARR=19999 对应。 */
#define MOTOR_PWM_MAX 19999U

/**
 * @brief 将速度限制在驱动支持的 -100 到 100 范围内。
 * @param speed 待限制的速度值。
 * @return 限制后的速度值。
 */
static int16_t Motor_ClampSpeed(int16_t speed)
{
    /* 大于最大速度时限制为最大速度，避免 PWM 比较值超出预期范围。 */
    if (speed > 100)
    {
        return 100;
    }

    /* 小于最小速度时限制为最小速度，避免 PWM 比较值超出预期范围。 */
    if (speed < -100)
    {
        return -100;
    }

    /* 输入已在支持范围内，直接返回原速度。 */
    return speed;
}

/**
 * @brief 设置单个电机的方向 GPIO 和 PWM 占空比。
 * @param speed     经过范围限制的电机速度。
 * @param in1_port  方向控制 GPIO 的第一个端口。
 * @param in1_pin   方向控制 GPIO 的第一个引脚。
 * @param in2_port  方向控制 GPIO 的第二个端口。
 * @param in2_pin   方向控制 GPIO 的第二个引脚。
 * @param htim      电机 PWM 所使用的定时器句柄。
 * @param channel   电机 PWM 所使用的定时器通道。
 */
static void Motor_SetOne(int16_t speed,
                         GPIO_TypeDef *in1_port,
                         uint16_t in1_pin,
                         GPIO_TypeDef *in2_port,
                         uint16_t in2_pin,
                         TIM_HandleTypeDef *htim,
                         uint32_t channel)
{
    /* 先限制速度，确保后续取绝对值和占空比计算处于安全范围内。 */
    speed = Motor_ClampSpeed(speed);

    /* 速度为零时关闭 PWM，并将两个方向引脚拉低使电机停止。 */
    if (speed == 0)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(htim, channel, 0U);
        return;
    }

    /* 正速度设置为前进方向：第一个引脚为高，第二个引脚为低。 */
    if (speed > 0)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }
    /* 负速度设置为后退方向：第一个引脚为低，第二个引脚为高。 */
    else
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    }

    /* 取速度绝对值作为占空比比例；速度已限制到 -100 至 100，因此取反安全。 */
    const uint32_t magnitude = (uint32_t)((speed < 0) ? -speed : speed);

    /* 将速度绝对值从 0–100 线性映射到 PWM 比较值 0–19999。 */
    const uint32_t compare = (magnitude * MOTOR_PWM_MAX) / 100U;

    /* 更新对应定时器通道的 PWM 比较值，从而改变电机驱动占空比。 */
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}

/**
 * @brief 初始化电机驱动，并启动左右电机 PWM 输出。
 */
void Motor_Init(void)
{
    /* 上电初始化时将左电机方向控制引脚设置为低电平。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    /* 上电初始化时将右电机方向控制引脚设置为低电平。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

    /* 在启动 PWM 前先将左电机 TIM1_CH1 的比较值清零，避免意外转动。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);

    /* 在启动 PWM 前先将右电机 TIM2_CH3 的比较值清零，避免意外转动。 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);

    /* 启动左电机使用的 TIM1 通道 1 PWM 输出，失败时进入工程错误处理函数。 */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 启动右电机使用的 TIM2 通道 3 PWM 输出，失败时进入工程错误处理函数。 */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 设置左右电机速度和转动方向。
 * @param left_speed  左电机速度，范围为 -100 到 100。
 * @param right_speed 右电机速度，范围为 -100 到 100。
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    /* 设置左电机：PB0/PB1 控制方向，TIM1_CH1 输出 PWM。 */
    Motor_SetOne(left_speed,
                 GPIOB,
                 GPIO_PIN_0,
                 GPIOB,
                 GPIO_PIN_1,
                 &htim1,
                 TIM_CHANNEL_1);

    /* 设置右电机：PB10/PB11 控制方向，TIM2_CH3 输出 PWM。 */
    Motor_SetOne(right_speed,
                 GPIOB,
                 GPIO_PIN_10,
                 GPIOB,
                 GPIO_PIN_11,
                 &htim2,
                 TIM_CHANNEL_3);
}
