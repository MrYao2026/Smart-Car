/**
 * @file    app_follow.c
 * @brief   小车跟随控制 PID 模块实现。
 */

#include "app_follow.h" /* 引入跟随 PID 参数结构体和函数声明。 */

/* 设定小车与前方目标之间期望保持的距离，单位为厘米。 */
#define FOLLOW_TARGET_DISTANCE_CM 30.0f
/* 限制积分累积范围，避免长期误差导致积分饱和。 */
#define FOLLOW_INTEGRAL_LIMIT 100.0f
/* 限制控制器输出速度，匹配电机驱动支持的 -100 到 100 范围。 */
#define FOLLOW_OUTPUT_LIMIT 100.0f
/* HC-SR04 的常见有效测距上限约为 400 厘米，超出则视为无效测量。 */
#define FOLLOW_MAX_VALID_DISTANCE_CM 400.0f

/* 定义并存储全局跟随 PID 参数及其运行状态。 */
FollowPID_t g_follow_pid;

/**
 * @brief 初始化跟随 PID 参数和运行状态。
 */
void Follow_Init(void)
{
    /* 设置比例系数：距离误差每增加 1 厘米，输出按 Kp 增加相应速度。 */
    g_follow_pid.Kp = 2.0f;
    /* 设置积分系数：对持续存在的距离误差逐次补偿。 */
    g_follow_pid.Ki = 0.05f;
    /* 设置微分系数：按每次调用之间的误差变化量提供阻尼效果。 */
    g_follow_pid.Kd = 0.5f;

    /* 清零当前距离误差，等待首次有效测距后再计算。 */
    g_follow_pid.error = 0.0f;
    /* 清零积分累积，避免上次运行状态影响本次控制。 */
    g_follow_pid.integral = 0.0f;
    /* 清零微分变量，首次调用不计算误差变化率。 */
    g_follow_pid.derivative = 0.0f;
    /* 清零上次误差，为首次 PID 计算建立初始状态。 */
    g_follow_pid.previous_error = 0.0f;
    /* 将上次输出初始化为零，表示启动时不请求电机速度。 */
    g_follow_pid.output = 0.0f;
    /* 标记尚无有效历史误差，避免首次计算产生微分突变。 */
    g_follow_pid.has_previous_error = 0U;
}

/**
 * @brief 根据当前距离计算跟随基础速度。
 * @param current_distance 当前测得的距离，单位为厘米。
 * @return 限幅后的基础速度，范围为 -100 到 100。
 */
float Follow_Compute(float current_distance)
{
    /* 无效测距时停止输出并清除积分，避免传感器超时导致车辆误动作。 */
    if ((current_distance <= 0.0f) ||
        (current_distance > FOLLOW_MAX_VALID_DISTANCE_CM) ||
        (current_distance != current_distance))
    {
        /* 无效输入不能继续累积积分，以免恢复测量后产生突发输出。 */
        g_follow_pid.integral = 0.0f;
        /* 清除微分状态，避免无效测量前后的距离跳变形成微分冲击。 */
        g_follow_pid.derivative = 0.0f;
        /* 重置上次误差；恢复有效输入后将视为首次计算。 */
        g_follow_pid.previous_error = 0.0f;
        /* 清除历史误差有效标志，抑制恢复时的微分突变。 */
        g_follow_pid.has_previous_error = 0U;
        /* 无有效距离反馈时要求基础速度为零。 */
        g_follow_pid.output = 0.0f;
        /* 返回零速度，避免测距失败时继续驱动车辆。 */
        return 0.0f;
    }

    /* 误差定义为当前距离减目标距离：距离过远时误差为正，输出趋向前进。 */
    g_follow_pid.error = current_distance - FOLLOW_TARGET_DISTANCE_CM;

    /* 仅在已有有效历史误差时计算微分项，首个样本的微分量设为零。 */
    if (g_follow_pid.has_previous_error != 0U)
    {
        /* 计算本次与上次误差的变化量，供微分项按 Kd 加权使用。 */
        g_follow_pid.derivative = g_follow_pid.error - g_follow_pid.previous_error;
    }
    else
    {
        /* 首次计算时没有可比较的上次误差，因此不产生微分作用。 */
        g_follow_pid.derivative = 0.0f;
        /* 从本次开始保存有效误差，后续调用即可计算微分项。 */
        g_follow_pid.has_previous_error = 1U;
    }

    /* 累加当前误差，积分项用于逐步消除长期保持的距离偏差。 */
    g_follow_pid.integral += g_follow_pid.error;

    /* 将积分累积限制在正向上限内，防止持续正误差造成积分饱和。 */
    if (g_follow_pid.integral > FOLLOW_INTEGRAL_LIMIT)
    {
        /* 超出正向上限时钳位到最大积分值。 */
        g_follow_pid.integral = FOLLOW_INTEGRAL_LIMIT;
    }
    /* 将积分累积限制在负向下限内，避免持续负误差使积分无限减小。 */
    else if (g_follow_pid.integral < -FOLLOW_INTEGRAL_LIMIT)
    {
        /* 低于负向下限时钳位到最小积分值。 */
        g_follow_pid.integral = -FOLLOW_INTEGRAL_LIMIT;
    }

    /* 根据 PID 公式计算速度：比例响应当前误差，积分补偿长期偏差，微分抑制快速变化。 */
    g_follow_pid.output =
        (g_follow_pid.Kp * g_follow_pid.error) +
        (g_follow_pid.Ki * g_follow_pid.integral) +
        (g_follow_pid.Kd * g_follow_pid.derivative);

    /* 对控制输出做正向限幅，保证速度不会超过 100。 */
    if (g_follow_pid.output > FOLLOW_OUTPUT_LIMIT)
    {
        /* 输出过大时钳位为最大前进速度。 */
        g_follow_pid.output = FOLLOW_OUTPUT_LIMIT;
    }
    /* 对控制输出做负向限幅，保证速度不会低于 -100。 */
    else if (g_follow_pid.output < -FOLLOW_OUTPUT_LIMIT)
    {
        /* 输出过小时钳位为最大反向速度。 */
        g_follow_pid.output = -FOLLOW_OUTPUT_LIMIT;
    }

    /* 保存当前误差，供下一次计算微分项使用。 */
    g_follow_pid.previous_error = g_follow_pid.error;

    /* 返回限制在 -100 到 100 的基础速度值。 */
    return g_follow_pid.output;
}
