/**
 * @file    bsp_hcsr04.h
 * @brief   HC-SR04 超声波测距模块驱动接口。
 */

#ifndef BSP_HCSR04_H
#define BSP_HCSR04_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 测距失败的具体阶段；便于 OLED 和调试器定位持续 Timeout。 */
typedef enum
{
    HCSR04_STATUS_OK = 0,
    HCSR04_STATUS_NOT_READY,
    HCSR04_STATUS_ECHO_HIGH,
    HCSR04_STATUS_NO_RISE,
    HCSR04_STATUS_NO_FALL,
    HCSR04_STATUS_RANGE,
    HCSR04_STATUS_TOO_FEW
} HCSR04_Status;

/**
 * @brief 初始化 HC-SR04 的 Trig、Echo 引脚及微秒级计时器。
 * @note  应在 HAL_Init() 和系统时钟配置完成后调用。
 */
void HCSR04_Init(void);

/**
 * @brief 触发一次测距并读取距离。
 * @return 距离，单位为厘米；等待 Echo 超时或 Echo 脉冲超时时返回 0.0f。
 */
float HCSR04_ReadDistance(void);

/**
 * @brief 连续测距五次，去掉最大值和最小值后对其余有效值求平均。
 * @return 滤波距离，单位为厘米；有效样本不足三个时返回 0.0f。
 */
float HCSR04_GetFilteredDistance(void);

/* 最近一次测距/滤波状态，以及最近一批五次测距中的有效样本数。 */
HCSR04_Status HCSR04_GetLastStatus(void);
uint8_t HCSR04_GetValidSampleCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_HCSR04_H */
