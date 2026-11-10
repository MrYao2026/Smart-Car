/**
 * @file    bsp_ina219.h
 * @brief   INA219 总线电压检测模块接口。
 */

#ifndef BSP_INA219_H
#define BSP_INA219_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 INA219，并配置为连续总线电压测量模式。
 * @return 配置写入成功返回 HAL_OK；I2C 通信失败返回 HAL_ERROR。
 */
HAL_StatusTypeDef INA219_Init(void);

/**
 * @brief 读取 INA219 的总线电压。
 * @return 总线电压，单位为伏特；I2C 通信失败时返回 NAN。
 */
float INA219_ReadBusVoltage(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_INA219_H */
