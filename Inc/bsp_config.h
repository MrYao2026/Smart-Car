/**
 * @file    bsp_config.h
 * @brief   用户运行参数及 STM32F1 内部 Flash 持久化接口。
 */

#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include <stdint.h>

/* 可掉电保存的参数结构；字段均使用半字，适配 STM32F1 Flash 编程宽度。 */
typedef struct
{
    uint16_t max_speed;
    uint16_t avoid_distance;
} Config_t;

/**
 * @brief 从保留的 Flash 页读取运行参数，无效时返回默认参数。
 * @return 已验证的配置：最大速度 100，避障基准距离 15 cm 为默认值。
 */
Config_t Config_Load(void);

/**
 * @brief 将参数写入专用 Flash 页。
 * @param config 要保存的参数结构。
 * @return HAL_OK 表示保存并校验成功；参数无效或 Flash 操作失败返回 HAL_ERROR。
 */
int Config_Save(const Config_t *config);

#endif /* BSP_CONFIG_H */
