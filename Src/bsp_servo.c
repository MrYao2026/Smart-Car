/**
 * @file    bsp_servo.c
 * @brief   SG90 舵机驱动实现，使用 TIM2_CH4（PA3）输出 PWM。
 */

#include "bsp_servo.h"
#include "main.h"

/* TIM2 预分频后计数频率为 1 MHz，因此一个计数对应 1 微秒。 */
#define SERVO_PWM_MIN_PULSE_US 500U
#define SERVO_PWM_MAX_PULSE_US 2500U
#define SERVO_ANGLE_MAX        180U
#define SERVO_INITIAL_ANGLE    0U
#define SERVO_ANGLE_STEP       2U
#define SERVO_STEP_DELAY_MS    20U

/* TIM2 在 main.c 中定义；TIM2_CH3 电机 PWM 与舵机共用同一计数器。 */
extern TIM_HandleTypeDef htim2;

/* 记录最近一次写入 PWM 的角度，供平滑运动和搬运状态判断使用。 */
static volatile uint8_t servo_current_angle = SERVO_INITIAL_ANGLE;
static volatile uint8_t servo_target_angle = SERVO_INITIAL_ANGLE;
static volatile uint32_t servo_step_tick = 0U;

/**
 * @brief 按当前角度更新 PWM 脉宽，但不修改平滑运动的目标角度。
 */
static void Servo_WriteCurrentAngle(void)
{
    const uint8_t angle = servo_current_angle;
    const uint32_t pulse_us =
        SERVO_PWM_MIN_PULSE_US +
        (((uint32_t)angle *
          (SERVO_PWM_MAX_PULSE_US - SERVO_PWM_MIN_PULSE_US) +
          (SERVO_ANGLE_MAX / 2U)) /
         SERVO_ANGLE_MAX);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_us);
}

/**
 * @brief 初始化 SG90 舵机 PWM 输出。
 *
 * TIM2 的预分频系数为 72，计数频率为 1 MHz；自动重装载值为 19999，
 * 因而周期为 20000 微秒，即 50 Hz。先设定中位脉宽，再启动通道输出。
 */
void Servo_Init(void)
{
    /* 上电先将推板放下到 0 度，作为自动搬运流程的安全起始位置。 */
    Servo_SetAngle(SERVO_INITIAL_ANGLE);

    /* 启动 TIM2_CH4；若启动失败，使用工程统一错误处理函数报告故障。 */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 设置舵机目标角度并更新 TIM2_CH4 的 PWM 脉宽。
 * @param angle 目标角度，uint8_t 参数大于 180 时按 180 度处理。
 */
void Servo_SetAngle(uint8_t angle)
{
    /* 限制角度范围，保证脉宽始终落在 SG90 的约 0.5 至 2.5 ms 范围内。 */
    if (angle > SERVO_ANGLE_MAX)
    {
        angle = SERVO_ANGLE_MAX;
    }
    servo_current_angle = angle;
    servo_target_angle = angle;

    /* TIM2 每计数为 1 微秒；只更新 PWM，不等待舵机机械运动完成。 */
    Servo_WriteCurrentAngle();
}

/**
 * @brief 以小步进和固定间隔平滑转动舵机。
 * @param target_angle 目标角度，超出机械控制范围时限制到 180 度。
 */
void Servo_RotateSmoothly(uint8_t target_angle)
{
    if (target_angle > SERVO_ANGLE_MAX)
    {
        target_angle = SERVO_ANGLE_MAX;
    }

    servo_target_angle = target_angle;
    servo_step_tick = HAL_GetTick();
}

/**
 * @brief 每到 20 ms 节拍最多推进舵机 2 度，不使用忙等或 HAL_Delay。
 */
void Servo_Update(void)
{
    const uint32_t now = HAL_GetTick();
    const uint8_t target_angle = servo_target_angle;
    uint8_t current_angle = servo_current_angle;

    if ((current_angle == target_angle) ||
        ((uint32_t)(now - servo_step_tick) < SERVO_STEP_DELAY_MS))
    {
        return;
    }

    servo_step_tick = now;
    if (current_angle < target_angle)
    {
        const uint16_t next_angle =
            (uint16_t)current_angle + SERVO_ANGLE_STEP;
        current_angle =
            (next_angle > target_angle) ? target_angle : (uint8_t)next_angle;
    }
    else
    {
        const uint8_t angle_difference =
            (uint8_t)(current_angle - target_angle);
        current_angle =
            (angle_difference > SERVO_ANGLE_STEP)
                ? (uint8_t)(current_angle - SERVO_ANGLE_STEP)
                : target_angle;
    }

    /* 更新一步 PWM 脉宽；等待下一次 20 ms 节拍，不在此处忙等舵机。 */
    servo_current_angle = current_angle;
    Servo_WriteCurrentAngle();
}

/**
 * @brief 获取驱动当前记录的舵机角度。
 * @return 最近写入 PWM 的舵机角度。
 */
uint8_t Servo_GetAngle(void)
{
    return (uint8_t)servo_current_angle;
}

/**
 * @brief 将舵机转到 0 度，推板放下。
 */
void Servo_PushDown(void)
{
    Servo_RotateSmoothly(0U);
}

/**
 * @brief 将舵机转到 90 度，推板抬起。
 */
void Servo_PushUp(void)
{
    Servo_RotateSmoothly(90U);
}
