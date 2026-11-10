/**
 * @file    app_follow.h
 * @brief   小车跟随控制 PID 模块接口。
 */

#ifndef APP_FOLLOW_H
#define APP_FOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 跟随控制器的 PID 参数和运行状态。
 */
typedef struct
{
    float Kp;             /* 比例系数：按当前距离误差立即调整基础速度。 */
    float Ki;             /* 积分系数：累积长期误差，补偿持续存在的偏差。 */
    float Kd;             /* 微分系数：根据误差变化趋势抑制快速变化和超调。 */
    float error;          /* 当前距离误差，单位为厘米。 */
    float integral;       /* 累积误差，用于积分项，并由积分限幅保护。 */
    float derivative;     /* 本次误差变化量，用于微分项。 */
    float previous_error; /* 上一次计算得到的误差，用于计算微分项。 */
    float output;         /* 最近一次 PID 输出，范围为 -100 到 100。 */
    unsigned char has_previous_error; /* 是否已有有效的上次误差。 */
} FollowPID_t;

/**
 * @brief 跟随 PID 控制器的全局参数及状态，可在初始化后按需调整 Kp、Ki、Kd。
 */
extern FollowPID_t g_follow_pid;

/**
 * @brief 初始化跟随 PID 参数和运行状态。
 */
void Follow_Init(void);

/**
 * @brief 根据当前距离计算跟随基础速度。
 * @param current_distance 当前测得的距离，单位为厘米。
 * @return 基础速度，范围为 -100 到 100；正值表示前进，负值表示后退。
 * @note  距离无效（小于等于 0 或大于 400 厘米）时返回 0 并清除积分状态。
 * @note  PID 增益按每次调用计算；应保持调用周期基本固定，以获得稳定效果。
 */
float Follow_Compute(float current_distance);

#ifdef __cplusplus
}
#endif

#endif /* APP_FOLLOW_H */
