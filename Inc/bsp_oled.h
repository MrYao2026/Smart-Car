/**
 * @file    bsp_oled.h
 * @brief   SSD1306 OLED 软件 I2C 显示接口。
 */

#ifndef BSP_OLED_H
#define BSP_OLED_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 PB6/PB7 软件 I2C 及 SSD1306 显示屏。
 * @return 初始化成功返回 HAL_OK；I2C 无应答时返回 HAL_ERROR。
 */
HAL_StatusTypeDef OLED_Init(void);

/**
 * @brief 在指定页显示 ASCII 字符串。
 * @param page OLED 页编号，范围为 0 至 7；页 0 是屏幕第一行。
 * @param text 以空字符结尾的 ASCII 字符串。
 * @return 显示成功返回 HAL_OK；参数无效或 I2C 无应答时返回 HAL_ERROR。
 */
HAL_StatusTypeDef OLED_ShowString(uint8_t page, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* BSP_OLED_H */
