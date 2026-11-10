/**
 * @file    bsp_motor.h
 * @brief   小车左右直流电机驱动接口。
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机驱动，并启动左右电机 PWM 输出。
 */
void Motor_Init(void);

/**
 * @brief 设置左右电机速度和转动方向。
 * @param left_speed  左电机速度，范围为 -100 到 100；正数前进，负数后退，0 停止。
 * @param right_speed 右电机速度，范围为 -100 到 100；正数前进，负数后退，0 停止。
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
