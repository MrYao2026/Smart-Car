/**
 * @file    bsp_servo.h
 * @brief   SG90 舵机驱动接口，使用 TIM2_CH4（PA3）输出 PWM。
 */

#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 SG90 舵机 PWM 输出。
 *
 * 依赖 main.c 中已配置好的 TIM2_CH4：PWM 频率 50 Hz，周期 20 ms。
 * 初始化时将舵机目标位置设置为 0 度（推板放下）并启动 PWM。
 */
void Servo_Init(void);

/**
 * @brief 设置舵机目标角度。
 * @param angle 目标角度，范围为 0 至 180 度；超出范围时限制到 180 度。
 *
 * 角度 0、90、180 度分别对应约 0.5 ms、1.5 ms、2.5 ms 高电平脉宽。
 */
void Servo_SetAngle(uint8_t angle);

/**
 * @brief 平滑转动舵机至目标角度。
 * @param target_angle 目标角度，范围为 0 至 180 度；超出范围时限制到 180 度。
 *
 * 此函数只设置目标位置，不阻塞；主循环需周期调用 Servo_Update()。
 */
void Servo_RotateSmoothly(uint8_t target_angle);

/**
 * @brief 按 20 ms 节拍将舵机角度向目标位置推进 2 度。
 *
 * 应在主循环中周期调用；调用本函数不会等待，不会阻塞蓝牙接收或其他状态机。
 */
void Servo_Update(void);

/**
 * @brief 获取驱动当前记录的舵机目标角度。
 * @return 最近设置的舵机角度，范围为 0 至 180 度。
 */
uint8_t Servo_GetAngle(void);

/**
 * @brief 将舵机转到 0 度，推板放下。
 */
void Servo_PushDown(void);

/**
 * @brief 将舵机转到 90 度，推板抬起。
 */
void Servo_PushUp(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
