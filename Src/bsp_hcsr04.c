/**
 * @file    bsp_hcsr04.c
 * @brief   HC-SR04 超声波测距模块驱动实现。
 */

#include "bsp_hcsr04.h" /* 引入 HC-SR04 驱动接口声明。 */
#include "main.h"       /* 引入 STM32 HAL、GPIO 定义及系统时钟变量。 */

/* 定义 Trig 引脚所在端口，按硬件连接使用 GPIOB。 */
#define HCSR04_TRIG_PORT GPIOB
/* 定义 Trig 引脚编号，按硬件连接使用 PB12。 */
#define HCSR04_TRIG_PIN GPIO_PIN_12
/* 定义 Echo 引脚所在端口，按硬件连接使用 GPIOB。 */
#define HCSR04_ECHO_PORT GPIOB
/* 定义 Echo 引脚编号，按硬件连接使用 PB13。 */
#define HCSR04_ECHO_PIN GPIO_PIN_13
/* 定义 Trig 脉冲宽度，HC-SR04 要求至少 10 微秒。 */
#define HCSR04_TRIG_PULSE_US 10U
/* 定义等待 Echo 上升沿及下降沿的超时时间，防止程序无限等待。 */
#define HCSR04_ECHO_TIMEOUT_US 30000U
/* 相邻触发至少间隔 65 ms，包含跨越滤波批次的触发。 */
#define HCSR04_TRIGGER_INTERVAL_MS 65U

/* 保存每微秒对应的 CPU 时钟周期数，初始化时由 SystemCoreClock 计算。 */
static uint32_t hcsr04_cycles_per_us = 0U;
static uint32_t hcsr04_last_trigger_tick = 0U;
static uint8_t hcsr04_has_triggered = 0U;
static uint8_t hcsr04_valid_samples = 0U;
static HCSR04_Status hcsr04_last_status = HCSR04_STATUS_NOT_READY;

/**
 * @brief 读取 DWT 周期计数器的当前值。
 * @return 当前累计 CPU 周期数；32 位计数器溢出后会自动回绕。
 */
static uint32_t HCSR04_GetCycleCount(void)
{
    /* 读取 Cortex-M3 的 DWT 周期计数器，用于微秒级脉宽测量。 */
    return DWT->CYCCNT;
}

/**
 * @brief 使用 DWT 周期计数器忙等待指定微秒数。
 * @param delay_us 需要等待的微秒数。
 */
static void HCSR04_DelayUs(uint32_t delay_us)
{
    /* 记录延时开始时的周期数，供后续计算已经经过的周期。 */
    const uint32_t start_cycle = HCSR04_GetCycleCount();
    /* 将微秒数转换成 CPU 周期数，使用无符号运算可处理计数器回绕。 */
    const uint32_t delay_cycles = delay_us * hcsr04_cycles_per_us;

    /* 持续等待至经过所需的周期数，10 微秒延时只会短暂占用 CPU。 */
    while ((uint32_t)(HCSR04_GetCycleCount() - start_cycle) < delay_cycles)
    {
        /* 空循环仅用于精确等待，不依赖毫秒精度的 HAL_GetTick()。 */
    }
}

/**
 * @brief 判断从起始周期至当前周期是否已达到超时值。
 * @param start_cycle 起始周期计数。
 * @param timeout_us  超时时间，单位为微秒。
 * @return 已超时返回 1，未超时返回 0。
 */
static uint8_t HCSR04_IsTimedOut(uint32_t start_cycle, uint32_t timeout_us)
{
    /* 按无符号差值计算已过周期数，即使 DWT 计数器回绕也能正确判断。 */
    const uint32_t elapsed_cycles = (uint32_t)(HCSR04_GetCycleCount() - start_cycle);
    /* 将超时时间换算为 CPU 周期并比较，返回是否达到超时门限。 */
    return (elapsed_cycles >= (timeout_us * hcsr04_cycles_per_us)) ? 1U : 0U;
}

/**
 * @brief 初始化 HC-SR04 的 Trig、Echo 引脚及微秒级计时器。
 */
void HCSR04_Init(void)
{
    /* 使能 GPIOB 时钟，保证 PB12 和 PB13 可以被配置和访问。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 创建 GPIO 初始化结构体，并清零未使用的配置成员。 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 将 PB12 配置为推挽输出，用作 HC-SR04 的 Trig 触发信号。 */
    GPIO_InitStruct.Pin = HCSR04_TRIG_PIN;
    /* 配置为通用推挽输出模式，输出高低电平控制超声波触发。 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    /* 设置引脚无内部上拉或下拉电阻。 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    /* Trig 仅需低速翻转即可，使用低速输出以减少边沿噪声。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    /* 将上述配置应用到 PB12。 */
    HAL_GPIO_Init(HCSR04_TRIG_PORT, &GPIO_InitStruct);

    /* 将 PB13 配置为输入，用于检测 HC-SR04 返回的 Echo 脉冲。 */
    GPIO_InitStruct.Pin = HCSR04_ECHO_PIN;
    /* 配置为普通输入模式。 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    /* Echo 输入不启用内部上拉或下拉，电平由传感器驱动。 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    /* 输入模式下 Speed 不起作用，无需修改其当前值。 */
    HAL_GPIO_Init(HCSR04_ECHO_PORT, &GPIO_InitStruct);

    /* 先将 Trig 拉低，确保初始化后不会意外触发测距。 */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    /* 开启 Cortex-M3 跟踪功能，使能 DWT 周期计数器所需的访问权限。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* 将 DWT 周期计数器清零，方便后续周期差值测量。 */
    DWT->CYCCNT = 0U;
    /* 使能 DWT 的 CPU 周期计数器。 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 根据当前系统时钟计算每微秒对应的 CPU 周期数。 */
    hcsr04_cycles_per_us = SystemCoreClock / 1000000U;
    hcsr04_has_triggered = 0U;
    hcsr04_valid_samples = 0U;
    hcsr04_last_status = HCSR04_STATUS_OK;

    /* 防止异常低系统时钟导致除零或无法计时；正常 STM32 时钟不会进入此分支。 */
    if (hcsr04_cycles_per_us == 0U)
    {
        /* 关闭 DWT 周期计数器，避免后续延时和超时逻辑失效。 */
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
        /* 使用工程统一错误处理函数报告不可用的微秒计时基础。 */
        Error_Handler();
    }
}

/**
 * @brief 触发一次测距并读取距离。
 * @return 距离，单位为厘米；超时返回 0.0f。
 */
float HCSR04_ReadDistance(void)
{
    /* 若尚未成功初始化计时器，则不能安全测距，返回超时约定值。 */
    if (hcsr04_cycles_per_us == 0U)
    {
        hcsr04_last_status = HCSR04_STATUS_NOT_READY;
        return 0.0f;
    }

    /*
     * 五次滤波不能紧挨着发射，否则可能读到上一发回波或触发失败。
     * 使用毫秒节拍控制间隔；仅从主循环调用，中断保持开启。
     */
    if (hcsr04_has_triggered != 0U)
    {
        const uint32_t elapsed =
            (uint32_t)(HAL_GetTick() - hcsr04_last_trigger_tick);
        if (elapsed < HCSR04_TRIGGER_INTERVAL_MS)
        {
            HAL_Delay(HCSR04_TRIGGER_INTERVAL_MS - elapsed);
        }
    }

    /* 必须先等 Echo 回到低电平，不能把残留高电平当成新回波。 */
    const uint32_t idle_wait_start = HCSR04_GetCycleCount();
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        if (HCSR04_IsTimedOut(idle_wait_start, HCSR04_ECHO_TIMEOUT_US) != 0U)
        {
            hcsr04_last_status = HCSR04_STATUS_ECHO_HIGH;
            return 0.0f;
        }
    }

    hcsr04_last_trigger_tick = HAL_GetTick();
    hcsr04_has_triggered = 1U;

    /* 先将 Trig 拉低，为随后产生的触发脉冲建立明确的低电平起始状态。 */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    /* 保持 Trig 低电平 2 微秒，满足触发脉冲前的稳定时间。 */
    HCSR04_DelayUs(2U);
    /* 将 Trig 拉高，通知 HC-SR04 开始一次测距。 */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    /* 保持 Trig 高电平至少 10 微秒，满足 HC-SR04 的触发要求。 */
    HCSR04_DelayUs(HCSR04_TRIG_PULSE_US);
    /* 将 Trig 拉低，结束触发脉冲并等待 Echo 响应。 */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    /* 记录开始等待 Echo 上升沿的周期数，用作上升沿超时基准。 */
    const uint32_t rise_wait_start = HCSR04_GetCycleCount();
    /* 当 Echo 仍为低电平时持续轮询，并在达到超时后返回。 */
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_RESET)
    {
        /* 没有收到 Echo 上升沿且等待已超时，避免主循环永久卡死。 */
        if (HCSR04_IsTimedOut(rise_wait_start, HCSR04_ECHO_TIMEOUT_US) != 0U)
        {
            /* 没有有效回波时按照接口约定返回 0 厘米。 */
            hcsr04_last_status = HCSR04_STATUS_NO_RISE;
            return 0.0f;
        }
    }

    /* Echo 已变高，记录上升沿时刻，开始测量高电平脉冲宽度。 */
    const uint32_t echo_high_start = HCSR04_GetCycleCount();
    /* 当 Echo 维持高电平时持续轮询，并对异常长脉冲实施超时保护。 */
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        /* Echo 高电平持续超过限制时认为测量无效，避免无限等待。 */
        if (HCSR04_IsTimedOut(echo_high_start, HCSR04_ECHO_TIMEOUT_US) != 0U)
        {
            /* 异常长脉冲不能换算为有效距离，按照接口约定返回 0 厘米。 */
            hcsr04_last_status = HCSR04_STATUS_NO_FALL;
            return 0.0f;
        }
    }

    /* Echo 变低后记录结束时刻，并以无符号差值获得脉冲周期数。 */
    const uint32_t echo_high_cycles =
        (uint32_t)(HCSR04_GetCycleCount() - echo_high_start);

    /* 零宽度脉冲属于无效回波，绝不能作为 0 厘米障碍物返回给上层。 */
    if (echo_high_cycles == 0U)
    {
        hcsr04_last_status = HCSR04_STATUS_RANGE;
        return 0.0f;
    }

    /* 将周期数转换为微秒，再按 HC-SR04 常用换算比例得到厘米数。 */
    const float distance_cm =
        ((float)echo_high_cycles / (float)hcsr04_cycles_per_us) / 58.0f;

    /* 0 厘米、负值或超出传感器量程均按测距失败处理，统一返回 0。 */
    if ((distance_cm <= 0.0f) || (distance_cm > 400.0f))
    {
        hcsr04_last_status = HCSR04_STATUS_RANGE;
        return 0.0f;
    }

    hcsr04_last_status = HCSR04_STATUS_OK;
    return distance_cm;
}

/**
 * @brief 连续读取五次距离，剔除最大值与最小值后计算平均值。
 * @return 去极值平均距离（厘米）；有效样本少于三个时返回 0.0f。
 */
float HCSR04_GetFilteredDistance(void)
{
    float samples[5];
    uint8_t valid_count = 0U;
    HCSR04_Status last_failure = HCSR04_STATUS_TOO_FEW;
    hcsr04_valid_samples = 0U;

    /* 固定触发五次测距；剔除超时、非正值和超出传感器有效范围的读数。 */
    for (uint8_t sample_index = 0U; sample_index < 5U; sample_index++)
    {
        const float sample = HCSR04_ReadDistance();
        if ((sample > 0.0f) && (sample <= 400.0f))
        {
            samples[valid_count] = sample;
            valid_count++;
        }
        else
        {
            last_failure = hcsr04_last_status;
        }
    }

    hcsr04_valid_samples = valid_count;
    /* 有效值不足三笔时无法执行去极值平均，返回超时约定值。 */
    if (valid_count < 3U)
    {
        /* 保留失败阶段；有效样本数另行显示，不把所有失败都混成 Timeout。 */
        hcsr04_last_status = last_failure;
        return 0.0f;
    }

    /* 对有效样本排序，排序首尾即最大值与最小值。 */
    for (uint8_t sort_index = 0U; sort_index < valid_count; sort_index++)
    {
        for (uint8_t compare_index = 0U;
             compare_index <
                 (uint8_t)(valid_count - sort_index - 1U);
             compare_index++)
        {
            if (samples[compare_index] > samples[compare_index + 1U])
            {
                const float temporary = samples[compare_index];
                samples[compare_index] = samples[compare_index + 1U];
                samples[compare_index + 1U] = temporary;
            }
        }
    }

    /* 排除排序后的首尾值，对中间样本（五次有效时正好三笔）求平均。 */
    float sum = 0.0f;
    for (uint8_t sample_index = 1U; sample_index + 1U < valid_count;
         sample_index++)
    {
        sum += samples[sample_index];
    }

    hcsr04_last_status = HCSR04_STATUS_OK;
    return sum / (float)(valid_count - 2U);
}

HCSR04_Status HCSR04_GetLastStatus(void)
{
    return hcsr04_last_status;
}

uint8_t HCSR04_GetValidSampleCount(void)
{
    return hcsr04_valid_samples;
}
